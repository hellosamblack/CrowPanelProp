/* prop_csi — WiFi CSI capture (real or synthetic) for the SIGNAL ENVIRONMENT panel. See prop_csi.h. */
#include "prop_csi.h"
#include "prop_net.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_wifi.h"   /* esp_wifi_set_csi* (forwarded to the C6 by esp_wifi_remote) */

#define CSI_TAG "PROP_CSI"

static bool s_available;
static volatile bool s_live;

/* s_stage: folded amplitudes from the most recent real CSI frame (writer = CSI cb).
 * s_bins:  published column the UI reads. s_synth: synthetic smoothing state. */
static uint8_t s_stage[PROP_CSI_BINS];
static uint8_t s_bins[PROP_CSI_BINS];
static float   s_synth[PROP_CSI_BINS];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t s_last_real_ms;   /* last real CSI frame */
static uint32_t s_last_fold_ms;            /* decimation of the fold work */

#define CSI_FOLD_MIN_MS 40     /* cap CSI fold rate (~25 Hz) regardless of frame rate */
#define CSI_LIVE_MS     800    /* "live" if a real frame arrived within this window */

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

/* CSI receive callback — runs in the WiFi/RPC task context, so keep it cheap.
 * Folds the per-subcarrier I/Q amplitudes into PROP_CSI_BINS, decimated in time. */
static void csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (!info || !info->buf || info->len < 4) {
        return;
    }
    uint32_t now = now_ms();
    if (now - s_last_fold_ms < CSI_FOLD_MIN_MS) {
        s_last_real_ms = now;   /* still "live", just not folding this one */
        return;
    }
    s_last_fold_ms = now;

    int sub = info->len / 2;            /* interleaved I,Q per subcarrier */
    const int8_t *b = info->buf;

    uint8_t tmp[PROP_CSI_BINS];
    for (int k = 0; k < PROP_CSI_BINS; k++) {
        int lo = k * sub / PROP_CSI_BINS;
        int hi = (k + 1) * sub / PROP_CSI_BINS;
        if (hi <= lo) hi = lo + 1;
        if (hi > sub) hi = sub;
        float acc = 0.0f;
        int c = 0;
        for (int s = lo; s < hi; s++) {
            int i = b[2 * s], q = b[2 * s + 1];
            acc += sqrtf((float)(i * i + q * q));
            c++;
        }
        float amp = c ? acc / c : 0.0f;
        int v = (int)(amp * 100.0f / 90.0f);   /* full-scale ~90 magnitude -> 100 */
        if (v < 0)   v = 0;
        if (v > 100) v = 100;
        tmp[k] = (uint8_t)v;
    }

    portENTER_CRITICAL(&s_mux);
    memcpy(s_stage, tmp, PROP_CSI_BINS);
    portEXIT_CRITICAL(&s_mux);
    s_last_real_ms = now;
}

/* Synthetic column: a believable signal-environment trace from link RSSI plus a
 * drifting per-bin random walk and a slow spatial hump. Used when no real CSI is
 * arriving (idle/AP-only link, or slave CSI unavailable). Real data when we have
 * it, honest motion when we don't. */
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
        vTaskDelay(pdMS_TO_TICKS(66));   /* ~15 Hz publish */
        uint32_t now = now_ms();
        bool live = (now - s_last_real_ms) < CSI_LIVE_MS;

        uint8_t col[PROP_CSI_BINS];
        if (live) {
            portENTER_CRITICAL(&s_mux);
            memcpy(col, s_stage, PROP_CSI_BINS);
            portEXIT_CRITICAL(&s_mux);
        } else {
            synthesize(col, now);
        }
        memcpy(s_bins, col, PROP_CSI_BINS);   /* publish (benign meter race with UI) */
        s_live = live;
    }
}

esp_err_t prop_csi_init(void)
{
    for (int i = 0; i < PROP_CSI_BINS; i++) s_synth[i] = 30.0f;

    /* Best-effort real CSI: register the sink, configure acquisition, enable.
     * Any failure just means we run synthetic — the slave may not deliver CSI over
     * the hosted link, which is exactly what the fallback is for. */
    esp_err_t err = esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL);
    if (err == ESP_OK) {
        wifi_csi_config_t cfg = {
            .enable             = 1,
            .acquire_csi_legacy = 1,
            .acquire_csi_ht20   = 1,
            .acquire_csi_ht40   = 1,
            .val_scale_cfg      = 2,
        };
        esp_err_t cerr = esp_wifi_set_csi_config(&cfg);
        if (cerr != ESP_OK) {
            ESP_LOGW(CSI_TAG, "set_csi_config: %s (running synthetic)", esp_err_to_name(cerr));
        }
        err = esp_wifi_set_csi(true);
    }
    if (err != ESP_OK) {
        ESP_LOGW(CSI_TAG, "real CSI unavailable (%s) — synthetic signal environment",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(CSI_TAG, "CSI requested from C6 (falls back to synthetic if no frames)");
    }

    if (xTaskCreate(csi_task, "prop_csi", 4096, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_available = true;
    return err;   /* report the real-CSI enable result; instrument runs regardless */
}

bool prop_csi_available(void) { return s_available; }
bool prop_csi_is_live(void)   { return s_live; }

void prop_csi_get_column(uint8_t *out)
{
    if (out) {
        memcpy(out, s_bins, PROP_CSI_BINS);   /* benign race: it's a meter */
    }
}
