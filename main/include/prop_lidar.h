#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROP_LIDAR_FRAME_W      480
#define PROP_LIDAR_FRAME_H      480
#define PROP_LIDAR_FRAME_PIXELS (PROP_LIDAR_FRAME_W * PROP_LIDAR_FRAME_H)

typedef enum {
    PROP_LIDAR_MODE_POINT_CLOUD = 0,
    PROP_LIDAR_MODE_SLAM,
    PROP_LIDAR_MODE_IR,
} prop_lidar_mode_t;

typedef enum {
    PROP_LIDAR_LINK_SEARCHING = 0,  /* resolving mDNS / connecting */
    PROP_LIDAR_LINK_OK,             /* connected, frames arriving within budget */
    PROP_LIDAR_LINK_STALE,          /* connected, but no frame in >2s */
} prop_lidar_link_t;

#define PROP_LIDAR_IR_CELLS 64   /* 8x8 thermal preview grid */

typedef struct {
    prop_lidar_link_t link;
    float             fps;
    int               point_count;
    bool              recording;
    prop_lidar_mode_t mode;
    /* Rig orientation (server IMU). Only meaningful while orientation_valid — the
     * server clears it when the boresight is vertical / the IMU is uncalibrated. */
    float             heading_deg;
    float             pitch_deg;
    float             roll_deg;
    float             yaw_rate_dps;
    bool              orientation_valid;
    /* 8x8 row-major IR preview, 0..255 per cell; has_ir_grid when the last
     * thin_telemetry message carried one. */
    uint8_t           ir_grid[PROP_LIDAR_IR_CELLS];
    bool              has_ir_grid;
} prop_lidar_telemetry_t;

/* Starts the background connection task. Call once, after prop_net_init() has brought
 * up WiFi + mdns_init(). Non-fatal to call even if WiFi is down: the task just keeps
 * retrying discovery in the background. */
esp_err_t prop_lidar_init(void);

/* Zero-copy access to the newest complete RGB565 frame (PROP_LIDAR_FRAME_W x
 * PROP_LIDAR_FRAME_H uint16_t's, little-endian, tightly packed). Returns the front
 * buffer of the internal PSRAM TRIPLE buffer and sets *out_seq to its sequence
 * number, or NULL before the first frame ever arrives. The writer always fills
 * (front+1)%3, so both the current and the previous front stay untouched for at
 * least the next two frames — a reader that retargets within one flip can never
 * be scribbled on mid-render. Point lv_canvas at the pointer directly instead of
 * memcpy'ing 460 KB per frame. */
const uint16_t *prop_lidar_peek_frame(uint32_t *out_seq);

/* Current frame sequence number without copying the frame (0 before the first frame
 * ever arrives). For observers that only want to know whether/how fast frames are
 * landing — e.g. /telemetry — without paying the 460 KB memcpy. */
uint32_t prop_lidar_get_seq(void);

/* Copies the current cached telemetry. Always succeeds; before the first thin_telemetry
 * message arrives it reports link=PROP_LIDAR_LINK_SEARCHING and zeroed fields. */
void prop_lidar_get_telemetry(prop_lidar_telemetry_t *out);

/* Best-effort outbound commands; no-ops quickly if not currently connected. */
void prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom);
void prop_lidar_send_mode(prop_lidar_mode_t mode);
void prop_lidar_send_record(bool on);

#ifdef __cplusplus
}
#endif
