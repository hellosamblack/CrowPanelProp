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

typedef struct {
    prop_lidar_link_t link;
    float             fps;
    char              power_mode[16];
    float             i3c_airtime_pct;
    int               point_count;
    bool              recording;
    prop_lidar_mode_t mode;
} prop_lidar_telemetry_t;

/* Starts the background connection task. Call once, after prop_net_init() has brought
 * up WiFi + mdns_init(). Non-fatal to call even if WiFi is down: the task just keeps
 * retrying discovery in the background. */
esp_err_t prop_lidar_init(void);

/* Copies the newest complete RGB565 frame (PROP_LIDAR_FRAME_PIXELS uint16_t's,
 * little-endian) into dst, which must be at least PROP_LIDAR_FRAME_PIXELS * 2 bytes.
 * Returns true and sets *out_seq to the frame's sequence number if a frame was
 * available; returns false (leaving dst untouched) before the first frame ever arrives.
 * Cheap to call every UI tick — skip the canvas blit when *out_seq hasn't changed. */
bool prop_lidar_get_frame(uint16_t *dst, uint32_t *out_seq);

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
