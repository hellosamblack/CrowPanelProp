/* prop_csi — SIGNAL ENVIRONMENT panel data source.
 *
 * Real Wi-Fi CSI now runs ON the C6 (ESPectre motion detection); the verdict
 * (motion / movement metric / threshold) arrives over esp-hosted custom RPC and
 * is cached by prop_coproc. This module turns that into the panel's display: a
 * scrolling movement-history waterfall + a live MOTION/IDLE state. When the C6
 * isn't delivering (no STA link / detector down) it falls back to the synthetic
 * RSSI-driven trace — real data when we have it, honest filler when we don't.
 */
#include "prop_csi.h"
#include "prop_net.h"
#include "prop_coproc.h"
#include "prop_audio.h"
#include "prop_calib.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"

#define CSI_TAG "PROP_CSI"

static bool s_available;
static volatile bool s_live;

/* s_bins: published column the UI reads. s_hist: scrolling movement history
 * (one bar per C6 sample). s_synth: synthetic smoothing state for the fallback. */
static uint8_t s_bins[PROP_CSI_BINS];
static uint8_t s_hist[PROP_CSI_BINS];
static float   s_synth[PROP_CSI_BINS];

/* Latest real verdict from the C6 (for the UI motion readout). */
static volatile bool s_motion;
static volatile int  s_movement_milli;
static volatile int  s_threshold_milli;

/* Extra RF datapoints (turbulence + receiver gain state) from the C6. */
static volatile int  s_turbulence_milli;
static volatile int  s_agc_gain;
static volatile int  s_fft_gain;
static volatile bool s_gain_locked;
static uint8_t       s_subcarriers[12];   /* NBVI band (fingerprint) */

/* SPECTRE GEIGER: audible clicks at a rate proportional to detected movement. */
static volatile bool s_geiger_on;

static uint32_t s_last_seq = 0xFFFFFFFFu;   /* last C6 heartbeat consumed */

/* C6 heartbeats every ~1 s; allow a couple of misses before declaring not-live. */
#define CSI_LIVE_MS 2600

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

/* Synthetic column: a believable signal-environment trace from link RSSI plus a
 * drifting per-bin random walk and a slow spatial hump. Used when the C6 isn't
 * delivering motion data (idle/AP-only link, or detector not yet up). */
static void synthesize(uint8_t *col, uint32_t now)
{
    int rssi = prop_net_get_rssi();      /* negative dBm, or 0 if no STA link */
    int base = rssi ? (rssi + 95) * 100 / 60 : 32;   /* -95 -> 0, -35 -> 100 */
    if (base < 12) base = 12;
    if (base > 88) base = 88;

    float phase = now * 0.0015f;
    for (int k = 0; k < PROP_CSI_BINS; k++) {
        float u = (float)k / (PROP_CSI_BINS - 1);
        float hump = 14.0f * sinf(u * 3.1416f + phase);          /* slow moving envelope */
        int noise = (int)(esp_random() % 21) - 10;               /* -10..10 */
        float target = (float)base + hump + noise;
        if (target < 2.0f)   target = 2.0f;
        if (target > 100.0f) target = 100.0f;
        s_synth[k] = s_synth[k] * 0.7f + target * 0.3f;          /* smooth */
        col[k] = (uint8_t)s_synth[k];
    }
}

static void csi_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(66));   /* ~15 Hz publish (smooth UI) */

        prop_csi_stats_t st;
        uint32_t age_ms = 0;
        bool have = prop_coproc_get_csi_stats(&st, &age_ms);
        bool live = have && st.csi_enabled && age_ms < CSI_LIVE_MS;

        if (live) {
            /* Motion decision is HOST-side: compare the raw movement metric to our
             * own uncapped adaptive threshold (the C6 detector's threshold is
             * capped at 10.0, too low for the raw metric). Hysteresis avoids
             * chatter: trip at the threshold, release at 80% of it. */
            s_movement_milli = st.movement_milli;
            int hthr = prop_calib_threshold();
            s_threshold_milli = hthr;
            if (!s_motion && st.movement_milli > hthr) {
                s_motion = true;
            } else if (s_motion && st.movement_milli < hthr * 4 / 5) {
                s_motion = false;
            }
            s_turbulence_milli = st.turbulence_milli;
            s_agc_gain = st.agc_gain;
            s_fft_gain = st.fft_gain;
            s_gain_locked = st.gain_locked != 0;
            memcpy(s_subcarriers, st.subcarriers, sizeof(s_subcarriers));

            /* Advance the movement-history waterfall once per new C6 sample. The
             * bar is movement as a % of the threshold (threshold == full height),
             * so motion (movement > threshold) pins the newest bars to the top. */
            if (st.seq != s_last_seq) {
                s_last_seq = st.seq;
                int thr = s_threshold_milli > 0 ? s_threshold_milli : 1000;
                int bar = st.movement_milli * 100 / thr;
                if (bar < 0)   bar = 0;
                if (bar > 100) bar = 100;
                memmove(s_hist, s_hist + 1, PROP_CSI_BINS - 1);
                s_hist[PROP_CSI_BINS - 1] = (uint8_t)bar;
            }
            memcpy(s_bins, s_hist, PROP_CSI_BINS);
        } else {
            uint8_t col[PROP_CSI_BINS];
            synthesize(col, now_ms());
            memcpy(s_bins, col, PROP_CSI_BINS);
            s_motion = false;
        }
        s_live = live;
    }
}

/* SPECTRE GEIGER: emit clicks at a rate that rises with detected movement (the
 * click probability scales with (movement/threshold)², so idle ticks rarely and
 * motion crackles). A novelty audible presence meter. */
static void geiger_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60));
        if (!s_geiger_on || !s_live) {
            continue;
        }
        int thr = s_threshold_milli > 0 ? s_threshold_milli : 1000;
        float ratio = (float)s_movement_milli / (float)thr;
        float prob = ratio * ratio * 0.15f;
        if (prob > 0.7f) { prob = 0.7f; }
        if ((float)(esp_random() & 0xFFFF) / 65535.0f < prob) {
            prop_audio_play(PA_DIAL_TICK);
        }
    }
}

esp_err_t prop_csi_init(void)
{
    for (int i = 0; i < PROP_CSI_BINS; i++) {
        s_synth[i] = 30.0f;
        s_hist[i] = 0;
    }

    /* Real CSI is captured + processed on the C6 (see c6_slave/.../prop_csi_slave.c)
     * and delivered via prop_coproc; nothing to enable host-side. The instrument
     * runs regardless and falls back to synthetic when the C6 isn't delivering. */
    ESP_LOGI(CSI_TAG, "SIGNAL ENV: consuming on-C6 motion verdict (synthetic fallback)");

    if (xTaskCreatePinnedToCore(csi_task, "prop_csi", 4096, NULL, 4, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    xTaskCreatePinnedToCore(geiger_task, "prop_geiger", 2560, NULL, 3, NULL, 0);
    s_available = true;
    return ESP_OK;
}

bool prop_csi_available(void) { return s_available; }
bool prop_csi_is_live(void)   { return s_live; }

void prop_csi_get_column(uint8_t *out)
{
    if (out) {
        memcpy(out, s_bins, PROP_CSI_BINS);   /* benign race: it's a meter */
    }
}

bool prop_csi_get_motion(bool *motion, int *movement_milli, int *threshold_milli)
{
    if (motion)          { *motion = s_motion; }
    if (movement_milli)  { *movement_milli = s_movement_milli; }
    if (threshold_milli) { *threshold_milli = s_threshold_milli; }
    return s_live;
}

bool prop_csi_get_rf(int *turbulence_milli, int *agc_gain, int *fft_gain, bool *gain_locked)
{
    if (turbulence_milli) { *turbulence_milli = s_turbulence_milli; }
    if (agc_gain)         { *agc_gain = s_agc_gain; }
    if (fft_gain)         { *fft_gain = s_fft_gain; }
    if (gain_locked)      { *gain_locked = s_gain_locked; }
    return s_live;
}

void prop_csi_get_subcarriers(uint8_t out[12])
{
    if (out) { memcpy(out, s_subcarriers, sizeof(s_subcarriers)); }
}

void prop_csi_set_geiger(bool on) { s_geiger_on = on; }
bool prop_csi_geiger(void)        { return s_geiger_on; }
