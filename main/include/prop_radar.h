/*
 * prop_radar — DFRobot 24 GHz mmWave presence radar (JYSJ "$JYBSS" firmware) on
 * UART2: RX=GPIO54 (radar TX), TX=GPIO53 (radar RX). The sensor streams binary
 * presence frames "$JYBSS, X, , , *" at 115200 8N1 (byte 7 = '1'/'0') roughly
 * once a second. Its leapMMW CLI also answers config queries (detection range,
 * sensitivity, latency, version), read once at boot and cached here.
 *
 * The firmware exposes NO live target distance — only presence. Distance can be
 * inferred by "range gating" (narrowing setRange and seeing where presence drops
 * out), but each setRange needs a saveCfg flash write, so that path is gated
 * behind an explicit, rate-limited trigger rather than run continuously.
 */
#ifndef PROP_RADAR_H
#define PROP_RADAR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Install the UART + start the reader task. NON-fatal. */
esp_err_t prop_radar_init(void);

/* True once at least one valid $JYBSS frame has been parsed (sensor is talking). */
bool prop_radar_available(void);

/* Latest presence verdict (true = something in the detection zone). */
bool prop_radar_present(void);

/* Total valid frames parsed, and sensor self-resets observed (diagnostics). */
uint32_t prop_radar_frames(void);
uint32_t prop_radar_resets(void);

/* Seconds since the presence state last changed. */
uint32_t prop_radar_dwell_s(void);

/* Saved sensor config, read once at boot. Any out-param may be NULL. Ranges are
 * metres; sensitivity 0..9; latency is trigger-delay / hold seconds. Unknown
 * values come back negative. */
void prop_radar_get_config(float *range_min, float *range_max, int *sensitivity,
                           float *lat_delay, float *lat_hold);

/* Firmware / hardware version strings ("?" until read). */
const char *prop_radar_swv(void);
const char *prop_radar_hwv(void);

/* Presence history (oldest..newest, 1=present/0=clear). Fills up to `max`
 * samples into out[], returns the count written. */
int prop_radar_history(uint8_t *out, int max);

#ifdef __cplusplus
}
#endif

#endif /* PROP_RADAR_H */
