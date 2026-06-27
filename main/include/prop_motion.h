#ifndef _PROP_MOTION_H_
#define _PROP_MOTION_H_

/* prop_motion — HiLink LD2450 24 GHz multi-target mmWave radar on UART2.
 *
 * The LD2450 streams binary 30-byte frames at 256 000 baud, reporting up to
 * three simultaneous target positions (X/Y in mm), speed (mm/s), and a
 * distance resolution value. The background reader task parks here; the UI
 * reads the cached last-parsed frame cheaply via prop_motion_get_targets().
 *
 * Non-fatal: if UART init fails, prop_motion_available() stays false and
 * callers receive zero targets — the prop never hangs.
 *
 * Pins: TX_GPIO=53 (P4→LD2450), RX_GPIO=54 (LD2450→P4), UART2, 256000 8N1.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_MOTION_MAX_TARGETS 3

typedef struct {
    int16_t  x_mm;       /* signed: + = right, - = left */
    int16_t  y_mm;       /* signed: + = forward (in front of sensor) */
    int16_t  speed_mm_s; /* signed: + = approaching */
    uint16_t dist_res_mm;
} prop_motion_target_t;

/* Init UART2, install driver, start background task.
 * Non-fatal: sets available=false and returns an error code if UART init fails.
 * Call from app_main after FreeRTOS is running. */
esp_err_t prop_motion_init(void);

/* True once init has succeeded. */
bool prop_motion_available(void);

/* Copy up to `max` active targets into `out`. Returns count (0..3).
 * Cheap cached read under short spinlock — safe to call from UI task. */
int prop_motion_get_targets(prop_motion_target_t *out, int max);

/* How many ms since the last valid frame was received (or UINT32_MAX if never). */
uint32_t prop_motion_ms_since_frame(void);

#endif /* _PROP_MOTION_H_ */
