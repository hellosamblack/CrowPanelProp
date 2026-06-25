/*
 * prop_calib — continuous adaptive auto-calibration for the on-C6 CSI motion
 * detector. Instead of a one-shot quiet/motion ritual, it runs always-on:
 *
 *   - watches the live movement-metric stream into a sliding window,
 *   - learns the quiet baseline distribution (percentiles) over time,
 *   - keeps segmentation_threshold a margin above the baseline so quiet reads
 *     IDLE and real motion (movement spiking above the learned floor) trips it.
 *
 * The threshold is PUSHED to the C6 (not persisted — re-learns each boot, ~40 s)
 * to avoid flash wear. RESET clears the window for a fast re-converge after the
 * unit is moved. Toggle off to hold a manual threshold instead. The UI (PK_CSICFG)
 * polls prop_calib_get().
 */
#ifndef PROP_CALIB_H
#define PROP_CALIB_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool auto_on;          /* adaptive control enabled */
    bool live;             /* a live CSI feed is present */
    int  fill_pct;         /* learning-window fill 0..100 (confidence) */
    int  baseline_milli;   /* learned quiet baseline (movement ×1000) */
    int  threshold_milli;  /* current adaptive threshold ×1000 */
    int  countdown;        /* >0 during a re-baseline countdown (seconds left) */
    char message[40];      /* status line for the panel */
} prop_calib_status_t;

/* Start the always-on adaptive task. Call after prop_coproc/prop_csi init. */
esp_err_t prop_calib_init(void);

/* Clear the learning window so the threshold re-converges quickly (after the
 * unit is moved / the environment changes). Sounds a short confirmation tone. */
void prop_calib_reset(void);

/* Enable/disable adaptive control (persisted). When off, the threshold is left
 * to whatever was last set manually. */
void prop_calib_set_auto(bool on);
bool prop_calib_auto(void);

/* The current adaptive threshold (×1000, raw-metric units). The HOST does the
 * motion decision against this (uncapped), since the C6 detector's own threshold
 * is capped at 10.0. */
int  prop_calib_threshold(void);

void prop_calib_get(prop_calib_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* PROP_CALIB_H */
