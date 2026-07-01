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
#include <stddef.h>
#include "esp_err.h"

#define PROP_MOTION_MAX_TARGETS 3

typedef struct {
    int16_t  x_mm;       /* signed: + = right, - = left */
    int16_t  y_mm;       /* signed: + = forward (in front of sensor) */
    int16_t  speed_mm_s; /* signed: + = approaching */
    uint16_t dist_res_mm;
} prop_motion_target_t;

/* ---- Config-command protocol (read/write access to on-module settings) --
 * Every prop_motion_cfg_* function performs a live UART command/ACK exchange
 * (see docs/datasheets/externalDevices/LD2450.../"LD2450 serial port
 * communication protocol V1.03.pdf" and
 * .claude/skills/sensor-datasheets/references/ld2450.md) and BLOCKS the
 * calling task for up to ~300ms (several seconds for prop_motion_cfg_set_baud
 * / prop_motion_cfg_factory_reset, which restart the module). Never call
 * these from the LVGL/UI task or from inside lvgl_port_lock()/unlock(). Only
 * one exchange runs at a time; concurrent callers serialize on an internal
 * mutex. */

typedef enum {
    PROP_MOTION_TRACK_SINGLE = 1,
    PROP_MOTION_TRACK_MULTI  = 2,
} prop_motion_track_mode_t;

typedef enum {
    PROP_MOTION_ZONE_OFF     = 0,   /* zone filtering disabled (factory default) */
    PROP_MOTION_ZONE_INCLUDE = 1,   /* only detect targets inside the listed zones */
    PROP_MOTION_ZONE_EXCLUDE = 2,   /* ignore targets inside the listed zones */
} prop_motion_zone_mode_t;

typedef struct {
    int16_t x1_mm, y1_mm, x2_mm, y2_mm;  /* diagonal rectangle corners, mm.
                                          * Plain signed int16 (two's complement)
                                          * -- NOT the sign-magnitude encoding
                                          * the real-time data frames use.
                                          * All-zero = this zone slot unused. */
} prop_motion_zone_t;

/* Tracking mode: read the module's current single/multi setting, or change it
 * (factory default: multi). */
bool prop_motion_cfg_get_mode(prop_motion_track_mode_t *out);
bool prop_motion_cfg_set_mode(prop_motion_track_mode_t mode);

/* Firmware version as a display string (e.g. "V1.02.22062416"); out_len >= 24. */
bool prop_motion_cfg_get_fw_version(char *out, size_t out_len);

/* The module's 6-byte MAC address (its onboard Bluetooth radio's MAC).
 * While Bluetooth is off the module reports the fixed placeholder
 * 08:05:04:03:02:01 instead of its real MAC. */
bool prop_motion_cfg_get_mac(uint8_t mac_out[6]);

/* Bluetooth on/off. Write-only -- the protocol has no BT-status query
 * command, so there is no prop_motion_cfg_get_bt(). On by default from the
 * factory; this project turns it off at boot (see prop_motion_init). */
bool prop_motion_cfg_set_bt(bool on);

/* Rectangular zone filtering: read or replace the current configuration (up
 * to 3 zones; a zone with all-zero coordinates is unused). */
bool prop_motion_cfg_get_zone(prop_motion_zone_mode_t *mode, prop_motion_zone_t zones[3]);
bool prop_motion_cfg_set_zone(prop_motion_zone_mode_t mode, const prop_motion_zone_t zones[3]);

/* Restart the module (it reboots itself right after ACKing this command).
 * Blocks until data streaming resumes or ~5s elapses. */
bool prop_motion_cfg_restart(void);

/* Restore all module settings to factory defaults (baud 256000, BT on, multi-
 * target tracking, zone filtering off) and restart to apply them. Reuses the
 * same restart+resync path as prop_motion_cfg_set_baud since baud reverts too. */
bool prop_motion_cfg_factory_reset(void);

/* Change the module's UART baud rate. HIGH RISK: this restarts the module and
 * reconfigures the LOCAL ESP32 UART to match, then waits for data streaming
 * to resume; only 9600/19200/38400/57600/115200/230400/256000/460800 are
 * valid. If the module doesn't resume streaming at the new rate within ~5s,
 * this function automatically falls back to the PREVIOUS baud rate and
 * returns false — if the module is silent at BOTH rates afterward, it needs
 * physical attention (power-cycle or a bench USB-serial check). */
bool prop_motion_cfg_set_baud(uint32_t new_baud_bps);

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
