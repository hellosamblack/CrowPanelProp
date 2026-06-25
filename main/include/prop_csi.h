#ifndef _PROP_CSI_H_
#define _PROP_CSI_H_

/* prop_csi — WiFi CSI "signal environment" instrument.
 *
 * Real CSI is captured and processed into a motion verdict ON the C6 (ESPectre,
 * see c6_slave/.../prop_csi_slave.c); the verdict arrives over esp-hosted custom
 * RPC (prop_coproc) and this module renders it: a scrolling movement-history
 * waterfall (column) plus a live MOTION/IDLE state + movement-vs-threshold.
 *
 * Self-adapting: when the C6 is delivering motion data the column/state are real;
 * otherwise (no STA link, detector not up) it falls back to a synthetic RSSI-driven
 * column. prop_csi_is_live() reports which source is active so the panel can label it.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_CSI_BINS 32

/* Register the CSI callback, request CSI from the slave, and start the fold task.
 * Always succeeds enough to drive the synthetic fallback; returns the result of the
 * real-CSI enable attempt for logging. Call after prop_net_init (shares the C6). */
esp_err_t prop_csi_init(void);

bool prop_csi_available(void);   /* true once init ran (synthetic guarantees data) */
bool prop_csi_is_live(void);     /* true when real CSI frames are currently arriving */

/* Copy the latest PROP_CSI_BINS movement-history bars (0..100) into out. */
void prop_csi_get_column(uint8_t *out);

/* Latest motion verdict from the C6. Out-params may be NULL; movement/threshold
 * are ×1000 fixed-point. Returns true when the data is live (real C6), false when
 * synthetic. */
bool prop_csi_get_motion(bool *motion, int *movement_milli, int *threshold_milli);

/* Extra RF datapoints from the C6 detector: turbulence (raw spatial disturbance,
 * ×1000), receiver AGC + FFT gain, and whether the gain lock succeeded. Returns
 * true when live. */
bool prop_csi_get_rf(int *turbulence_milli, int *agc_gain, int *fft_gain, bool *gain_locked);

/* NBVI-selected subcarrier band (the channel "fingerprint"): 12 indices 0..63. */
void prop_csi_get_subcarriers(uint8_t out[12]);

/* SPECTRE GEIGER: audible clicks at a rate proportional to detected movement. */
void prop_csi_set_geiger(bool on);
bool prop_csi_geiger(void);

#endif /* _PROP_CSI_H_ */
