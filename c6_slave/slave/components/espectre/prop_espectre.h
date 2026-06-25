/*
 * prop_espectre — thin C wrapper around the (GPL-3.0) ESPectre CSI motion
 * detector, so the C slave glue (prop_csi_slave.c) can drive it. Lives inside
 * the espectre component because it #includes ESPectre C++ headers (and is thus
 * itself GPL-derived) — the P4 host links none of this.
 *
 * Owns a CSIManager + MVSDetector: prop_espectre_start() builds them, registers
 * the CSI rx callback, and begins detection on the fixed DEFAULT_SUBCARRIERS
 * band (no NBVI auto-calibration in v1). Verdict is polled via the getters.
 */
#ifndef PROP_ESPECTRE_H
#define PROP_ESPECTRE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initial config (values from prop_csi_slave's cfg store; floats ×1000). */
typedef struct {
    int window_size;        /* segmentation_window_size */
    int threshold_mode;     /* 0=auto 1=min 2=manual */
    int threshold_milli;    /* segmentation_threshold ×1000 (manual mode) */
    int eval_interval;      /* evaluation_interval (packets) */
    int on_hits, off_hits;  /* motion_on_hits / motion_off_hits */
    int lowpass_en, lowpass_cutoff_milli;
    int hampel_en, hampel_window, hampel_thresh_milli;
    int gain_lock_mode;     /* 0=auto 1=enabled 2=disabled */
    int publish_interval;   /* packets between packet_callback invocations */
} prop_espectre_cfg_t;

/* Build the detector + CSI manager and enable capture/detection. Requires
 * esp_wifi to be started (call with retry). Idempotent after success. */
esp_err_t prop_espectre_start(const prop_espectre_cfg_t *cfg);
bool      prop_espectre_started(void);

/* Latest verdict / metrics (polled by the stats heartbeat). */
int       prop_espectre_motion(void);          /* 0=idle 1=motion */
int       prop_espectre_movement_milli(void);  /* motion metric ×1000 */
int       prop_espectre_threshold_milli(void); /* active threshold ×1000 */
uint32_t  prop_espectre_packets(void);         /* packets processed */

/* Extra RF datapoints from the detector / gain controller. */
int       prop_espectre_turbulence_milli(void);/* raw per-packet spatial disturbance ×1000 */
int       prop_espectre_agc_gain(void);        /* locked AGC gain (0..63ish) */
int       prop_espectre_fft_gain(void);        /* locked FFT gain (signed) */
int       prop_espectre_gain_locked(void);     /* 1 if gain lock succeeded */
void      prop_espectre_get_subcarriers(uint8_t out[12]); /* NBVI-selected band */

/* Re-run NBVI subcarrier auto-selection (e.g. after the unit is moved). No-op if
 * not started / gain not yet locked. */
void prop_espectre_recalibrate(void);
bool prop_espectre_calibrating(void);

/* Live setters (from the runtime config handler — no reflash). No-ops before start. */
void prop_espectre_set_threshold(int mode, int threshold_milli);
void prop_espectre_set_eval(int interval);
void prop_espectre_set_hits(int on, int off);
void prop_espectre_set_lowpass(int en, int cutoff_milli);
void prop_espectre_set_hampel(int en, int window, int thresh_milli);

#ifdef __cplusplus
}
#endif

#endif /* PROP_ESPECTRE_H */
