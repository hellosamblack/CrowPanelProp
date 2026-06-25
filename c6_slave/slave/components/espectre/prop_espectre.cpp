/*
 * prop_espectre — C wrapper around ESPectre's CSI motion detector. See header.
 * GPL-3.0 (links ESPectre).
 */
#include "prop_espectre.h"

#include <new>
#include <cstring>
#include "esp_log.h"
#include "csi_manager.h"
#include "mvs_detector.h"
#include "base_detector.h"
#include "gain_controller.h"
#include "nbvi_calibrator.h"
#include "utils.h"

using namespace esphome::espectre;

static const char *TAG = "prop_espectre";

static MVSDetector     *s_det = nullptr;
static CSIManager       s_mgr;
static NBVICalibrator   s_nbvi;
static uint8_t          s_subcarriers[12];   /* current band (NBVI-selected or default) */
static bool             s_started = false;
static volatile int     s_motion  = 0;
static volatile uint32_t s_packets = 0;
static volatile bool    s_calibrating = false;
static int              s_cfg_window = 100;  /* remembered for re-calibration */

/* Map the (mode, ×1000) threshold setting to ESPectre's float. Without NBVI
 * auto-calibration in v1, "auto" falls back to the MVS default. */
static float threshold_from(int mode, int milli)
{
    if (mode == 2) { return (float)milli / 1000.0f;   }  /* manual */
    if (mode == 1) { return MVS_MIN_THRESHOLD;         }  /* min */
    return MVS_DEFAULT_THRESHOLD;                          /* auto */
}

static GainLockMode gain_mode_from(int m)
{
    return (m == 1) ? GainLockMode::ENABLED
         : (m == 2) ? GainLockMode::DISABLED
                    : GainLockMode::AUTO;
}

/* Kick off NBVI subcarrier selection. On completion, apply the selected band.
 * The threshold ESPectre would derive here is ignored — the P4 host runs its own
 * continuous adaptive threshold (prop_calib), which doesn't assume a quiet boot. */
static void start_nbvi()
{
    if (s_calibrating || s_nbvi.is_calibrating()) { return; }
    s_calibrating = true;

    s_nbvi.set_collection_complete_callback([]() {
        ESP_LOGI(TAG, "NBVI collection complete — analyzing");
    });

    esp_err_t err = s_nbvi.start_calibration(
        s_subcarriers, 12,
        [](const uint8_t *band, uint8_t size,
           const std::vector<float> &cal_values, bool success) {
            (void)cal_values;
            if (success && band && size == 12) {
                memcpy(s_subcarriers, band, 12);
                s_mgr.update_subcarrier_selection(band);
                ESP_LOGI(TAG, "NBVI band: [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
                         band[0], band[1], band[2], band[3], band[4], band[5],
                         band[6], band[7], band[8], band[9], band[10], band[11]);
            } else {
                ESP_LOGW(TAG, "NBVI failed — keeping current band");
            }
            s_mgr.clear_detector_buffer();
            s_calibrating = false;
        });
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NBVI start failed: %s", esp_err_to_name(err));
        s_calibrating = false;
    }
}

esp_err_t prop_espectre_start(const prop_espectre_cfg_t *c)
{
    if (s_started) { return ESP_OK; }
    if (!c)        { return ESP_ERR_INVALID_ARG; }

    float thr = threshold_from(c->threshold_mode, c->threshold_milli);
    s_det = new (std::nothrow) MVSDetector((uint16_t)c->window_size, thr);
    if (!s_det) { return ESP_ERR_NO_MEM; }

    s_det->configure_lowpass(c->lowpass_en != 0, (float)c->lowpass_cutoff_milli / 1000.0f);
    s_det->configure_hampel(c->hampel_en != 0, (uint8_t)c->hampel_window,
                            (float)c->hampel_thresh_milli / 1000.0f);
    /* CV normalization is decided by ESPectre at gain-lock (raw std when locked —
     * MORE sensitive; CV only when the gain can't lock). The host does the motion
     * decision against its own uncapped adaptive threshold, so the raw metric's
     * magnitude (which can exceed the detector's 0-10 threshold ceiling) is fine. */

    memcpy(s_subcarriers, DEFAULT_SUBCARRIERS, 12);
    s_cfg_window = c->window_size;

    s_mgr.init(s_det, s_subcarriers,
               (uint32_t)(c->publish_interval > 0 ? c->publish_interval : 100),
               gain_mode_from(c->gain_lock_mode));
    s_mgr.set_evaluation_interval((uint32_t)c->eval_interval);
    s_mgr.set_motion_on_hits((uint8_t)c->on_hits);
    s_mgr.set_motion_off_hits((uint8_t)c->off_hits);
    s_mgr.set_motion_state_callback([](MotionState st) {
        s_motion = (st == MotionState::MOTION) ? 1 : 0;
    });

    /* NBVI subcarrier auto-selection: the calibrator collects CSI (RAM-buffered)
     * and picks the 12 most informative subcarriers for this link. */
    s_nbvi.init(&s_mgr);
    s_nbvi.set_mvs_window_size((uint16_t)c->window_size);
    s_nbvi.configure_lowpass(c->lowpass_en != 0, (float)c->lowpass_cutoff_milli / 1000.0f);
    s_nbvi.configure_hampel(c->hampel_en != 0, (uint8_t)c->hampel_window,
                            (float)c->hampel_thresh_milli / 1000.0f);
    /* 4 windows (not the default 10) → ~26 KB RAM buffer; the full 64 KB can fail
     * to allocate on the C6 alongside WiFi+BT, which silently aborts NBVI. */
    s_nbvi.set_buffer_size((uint16_t)(c->window_size * 4));
    s_mgr.set_calibration_mode(&s_nbvi);   /* route CSI to the calibrator while it runs */

    /* When the AGC/FFT gain lock finishes: enable CV normalization if the gain
     * could NOT be locked (keeps the metric scale-invariant), then kick off NBVI
     * subcarrier selection. Mirrors ESPectre's own gain-lock callback. */
    s_mgr.set_gain_lock_callback([]() {
        /* ESPectre's own rule: CV normalization only when the gain couldn't lock. */
        const GainController &gc = s_mgr.get_gain_controller();
        bool need_cv = gc.needs_cv_normalization();
        if (s_det) { s_det->set_cv_normalization(need_cv); }
        s_nbvi.set_cv_normalization(need_cv);
        ESP_LOGI(TAG, "gain lock done: locked=%d cv=%d — starting NBVI",
                 (int)s_mgr.is_gain_locked(), (int)need_cv);
        start_nbvi();
    });

    esp_err_t err = s_mgr.enable([](MotionState st, uint32_t pkts) {
        s_motion  = (st == MotionState::MOTION) ? 1 : 0;
        s_packets = pkts;
    });
    s_started = (err == ESP_OK);
    ESP_LOGI(TAG, "start: win=%d thr=%.2f eval=%d hits=%d/%d gain=%d -> %s",
             c->window_size, thr, c->eval_interval, c->on_hits, c->off_hits,
             c->gain_lock_mode, esp_err_to_name(err));
    return err;
}

void prop_espectre_recalibrate(void)
{
    if (s_started && s_mgr.is_gain_locked()) {
        ESP_LOGI(TAG, "re-calibration requested");
        start_nbvi();
    } else {
        ESP_LOGW(TAG, "recal: not ready (started=%d locked=%d)",
                 (int)s_started, (int)s_mgr.is_gain_locked());
    }
}

bool     prop_espectre_calibrating(void)     { return s_calibrating; }
bool     prop_espectre_started(void)         { return s_started; }
int      prop_espectre_motion(void)          { return s_motion; }
uint32_t prop_espectre_packets(void)         { return s_packets; }
int      prop_espectre_movement_milli(void)  { return s_det ? (int)(s_det->get_motion_metric() * 1000.0f) : 0; }
int      prop_espectre_threshold_milli(void) { return s_det ? (int)(s_det->get_threshold() * 1000.0f) : 0; }
int      prop_espectre_turbulence_milli(void){ return s_det ? (int)(s_det->get_last_turbulence() * 1000.0f) : 0; }
int      prop_espectre_agc_gain(void)        { return s_started ? (int)s_mgr.get_gain_controller().get_agc_gain() : 0; }
int      prop_espectre_fft_gain(void)        { return s_started ? (int)s_mgr.get_gain_controller().get_fft_gain() : 0; }
int      prop_espectre_gain_locked(void)     { return s_started && s_mgr.is_gain_locked() ? 1 : 0; }
void     prop_espectre_get_subcarriers(uint8_t out[12]) { if (out) { memcpy(out, s_subcarriers, 12); } }

void prop_espectre_set_threshold(int mode, int milli)
{
    if (s_started) { s_mgr.set_threshold(threshold_from(mode, milli)); }
}
void prop_espectre_set_eval(int interval)
{
    if (s_started) { s_mgr.set_evaluation_interval((uint32_t)interval); }
}
void prop_espectre_set_hits(int on, int off)
{
    if (s_started) { s_mgr.set_motion_on_hits((uint8_t)on); s_mgr.set_motion_off_hits((uint8_t)off); }
}
void prop_espectre_set_lowpass(int en, int cutoff_milli)
{
    if (s_det) { s_det->configure_lowpass(en != 0, (float)cutoff_milli / 1000.0f); }
}
void prop_espectre_set_hampel(int en, int window, int thresh_milli)
{
    if (s_det) { s_det->configure_hampel(en != 0, (uint8_t)window, (float)thresh_milli / 1000.0f); }
}
