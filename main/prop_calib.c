/* prop_calib — continuous adaptive CSI threshold (sliding-window). See header.
 *
 * A sliding window of the movement metric is the baseline estimator: the
 * threshold is the window's 90th percentile plus two inter-percentile spans, so
 * it sits clear of the room's recent quiet movement. Because it's a WINDOW, it
 * re-baselines on its own: sit and work for hours and the threshold rises to your
 * presence (you become the baseline); walk away and within the window the quiet
 * samples pull it back down — "idle drops → threshold drops". No quiet-at-boot
 * assumption: it learns whatever it sees and corrects over the window.
 */
#include "prop_calib.h"
#include "prop_csi.h"
#include "prop_coproc.h"
#include "prop_audio.h"
#include "prop_settings.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "PROP_CALIB"

#define SAMPLE_MS    250   /* 4 Hz */
#define WIN          240   /* sliding window: 60 s @ 4 Hz */
#define ADAPT_TICKS    8   /* re-evaluate + maybe push every 2 s */
#define NEED          60   /* min samples (~15 s) before trusting the baseline */
#define MIN_THR     1500   /* floor (raw metric scale; avoids all-motion w/ no data) */
#define MAX_THR    80000   /* wide: the raw moving-variance metric runs large/noisy */
#define HYST_PCT       6   /* push only when the threshold moves > this % */

static int   s_win[WIN];
static int   s_wn, s_wi;          /* count, write index */
static bool  s_auto = true;
static int   s_threshold = MIN_THR;
static int   s_baseline;
static bool  s_pushed_once;       /* force the first push to override the persisted value */
static volatile bool s_reset_req;

static prop_calib_status_t s_st;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static int cmp_int(const void *a, const void *b) { return *(const int *)a - *(const int *)b; }

static void percentiles(int *p50, int *p90)
{
    static int tmp[WIN];
    int n = s_wn;
    if (n <= 0) { *p50 = *p90 = 0; return; }
    memcpy(tmp, s_win, n * sizeof(int));
    qsort(tmp, n, sizeof(int), cmp_int);
    *p50 = tmp[n / 2];
    *p90 = tmp[(n * 9) / 10];
}

static void publish(const char *msg, bool live, int fill)
{
    portENTER_CRITICAL(&s_mux);
    s_st.auto_on = s_auto;
    s_st.live = live;
    s_st.fill_pct = fill;
    s_st.baseline_milli = s_baseline;
    s_st.threshold_milli = s_threshold;
    strncpy(s_st.message, msg, sizeof(s_st.message) - 1);
    s_st.message[sizeof(s_st.message) - 1] = '\0';
    portEXIT_CRITICAL(&s_mux);
}

static void calib_task(void *arg)
{
    (void)arg;
    uint32_t v;
    prop_settings_get_u32("csi_auto", &v, 1);
    s_auto = v != 0;

    int tick = 0;
    for (;;) {
        if (s_reset_req) {
            s_reset_req = false;
            /* 5 s audible countdown, then drop the window to re-converge. */
            for (int c = 5; c > 0; c--) {
                portENTER_CRITICAL(&s_mux);
                s_st.countdown = c;
                strncpy(s_st.message, "RE-BASELINE", sizeof(s_st.message) - 1);
                portEXIT_CRITICAL(&s_mux);
                prop_audio_play_pitched(PA_BUTTON, c <= 2 ? 7 : (5 - c));
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            portENTER_CRITICAL(&s_mux);
            s_st.countdown = 0;
            portEXIT_CRITICAL(&s_mux);
            s_wn = s_wi = 0;       /* drop history → re-converge fast */
            s_pushed_once = false; /* re-push once the new window fills */
            prop_audio_play(PA_SIGNAL);
        }

        bool motion; int move_mi, thr_mi;
        bool live = prop_csi_get_motion(&motion, &move_mi, &thr_mi);
        if (live) {
            s_win[s_wi] = move_mi;
            s_wi = (s_wi + 1) % WIN;
            if (s_wn < WIN) { s_wn++; }
        }
        int fill = s_wn * 100 / NEED;
        if (fill > 100) { fill = 100; }

        if (++tick >= ADAPT_TICKS) {
            tick = 0;
            const char *msg;
            if (!s_auto) {
                msg = "AUTO OFF  //  MANUAL THR";
            } else if (!live) {
                msg = "NO CSI FEED";
            } else if (s_wn < NEED) {
                msg = "LEARNING BASELINE";
            } else {
                int p50, p90;
                percentiles(&p50, &p90);
                int span = p90 - p50;
                int thr = p90 + 2 * span;
                if (thr < MIN_THR) { thr = MIN_THR; }
                if (thr > MAX_THR) { thr = MAX_THR; }
                s_baseline = p50;
                int delta = thr > s_threshold ? thr - s_threshold : s_threshold - thr;
                /* Push on the first valid estimate (to override the persisted
                 * boot value), then only when it moves more than the hysteresis. */
                if (!s_pushed_once || delta * 100 > s_threshold * HYST_PCT) {
                    s_pushed_once = true;
                    s_threshold = thr;
                    prop_coproc_csi_push("segmentation_threshold", thr);  /* no NVS write */
                    ESP_LOGI(TAG, "adapt: p50=%d p90=%d -> thr=%d", p50, p90, thr);
                }
                msg = "AUTO-ADAPTING";
            }
            publish(msg, live, fill);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}

esp_err_t prop_calib_init(void)
{
    if (xTaskCreatePinnedToCore(calib_task, "prop_calib", 4096, NULL, 4, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void prop_calib_reset(void)
{
    s_reset_req = true;
    prop_coproc_csi_action(PROP_CSI_ACTION_RECAL);   /* also re-run NBVI on the C6 */
}

void prop_calib_set_auto(bool on)
{
    s_auto = on;
    prop_settings_set_u32("csi_auto", on ? 1 : 0);
}

bool prop_calib_auto(void) { return s_auto; }

int prop_calib_threshold(void) { return s_threshold; }

void prop_calib_get(prop_calib_status_t *out)
{
    if (!out) { return; }
    portENTER_CRITICAL(&s_mux);
    *out = s_st;
    portEXIT_CRITICAL(&s_mux);
}
