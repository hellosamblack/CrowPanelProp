#ifndef _PROP_TRACK_H_
#define _PROP_TRACK_H_

/* prop_track — pedestrian dead-reckoning (PDR) + world-frame spatial memory.
 *
 * Fuses the IMU and the LD2450 radar into a *position* the rest of the firmware
 * can map. The MPU-6500 exposes no displacement (double-integrating accel drifts
 * hopelessly), so distance comes from the hardware DMP pedometer: each step
 * advances the operator by a fixed stride along the current heading. Radar
 * targets are rotated through the operator pose into world coordinates and kept
 * as last-known marks. Follows the house pattern (background task → cached state
 * under a mutex → UI reads), like prop_motion / prop_ble.
 *
 * Frame: metres, +y = North / forward-at-origin, +x = East. Heading 0 = +y.
 *
 * HONEST LIMITS (Phase 1, gyro-only): the MPU-6500 has no magnetometer, so DMP
 * yaw is gyro-integrated → slow heading drift; "North" is boot-relative, not
 * magnetic. Stride is fixed. The path is an *impression*, not survey-grade. A
 * magnetometer (Phase 1.5) fuses in behind track_heading_rad() alone.
 *
 * Non-fatal: if the IMU never comes online, prop_track_available() stays false,
 * the pose is invalid and no crumbs accumulate — the prop never hangs.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_TRACK_MAX_MARKS  16    /* last-known radar marks retained (LRU) */

/* Operator pose. heading is live (updates while standing); x/y advance per step. */
typedef struct {
    float   x, y;        /* world position, metres */
    float   heading;     /* world heading, radians (0 = +y/North, CW positive) */
    uint8_t floor;       /* reserved for Phase 3 multi-floor; 0 in Phase 1 */
    bool    valid;       /* true once the IMU has produced a usable pose */
} prop_track_pose_t;

/* One breadcrumb of the walked path. */
typedef struct {
    float   x, y;        /* world position, metres */
    uint8_t floor;
} prop_track_crumb_t;

/* Last-known world position of a radar target. */
typedef struct {
    float    x, y;       /* world position, metres */
    uint32_t age_ms;     /* ms since this mark was last refreshed */
    uint8_t  floor;
} prop_track_mark_t;

/* Start the background PDR task. Non-fatal: returns ESP_OK even if the IMU is
 * absent (the task idles, pose stays invalid). Call after prop_imu_init /
 * prop_motion_init and after prop_settings_init (reads NVS "dir_phi"). */
esp_err_t prop_track_init(void);

/* True once the task is running and the IMU is online. */
bool prop_track_available(void);

/* Copy the current operator pose. Safe from any task. */
void prop_track_get_pose(prop_track_pose_t *out);

/* Copy up to `max` breadcrumbs, oldest → newest. Returns the count copied. */
int prop_track_get_crumbs(prop_track_crumb_t *out, int max);

/* Copy up to `max` last-known radar marks. Returns the count copied. */
int prop_track_get_marks(prop_track_mark_t *out, int max);

/* Push a new board→world yaw offset (radians). The PK_DIRCAL calibration screen
 * calls this so the path heading tracks a recalibration live. */
void prop_track_set_dir_phi(float phi_rad);

/* Zero the origin at the current position and clear all crumbs + marks. */
void prop_track_reset(void);

#endif /* _PROP_TRACK_H_ */
