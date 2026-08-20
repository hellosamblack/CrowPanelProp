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
    PROP_LIDAR_LINK_STALE,          /* connected, but no frame in 3x the measured cadence */
} prop_lidar_link_t;

/* IR thermal preview. The rig's sensor has a 54x42 = 2268-zone native reflectance
 * grid; v1 `thin_telemetry` only ever carried an 8x8 block-mean of it, which threw
 * away ~97%. This client advertises PROP_LIDAR_IR_MAX_CELLS in thin_hello and gets
 * the native grid back (base64, in `ir_grid_b64`) when it fits, so the numbers below
 * are a CEILING, not the shape — the real width/height arrive with every message and
 * are not even constant (the rig rotates the pane with gravity, transposing them).
 * A server that predates this, or one whose sensor is too big for the budget, still
 * sends the 8x8 and everything below just reports 8x8. */
#define PROP_LIDAR_IR_MAX_W     64
#define PROP_LIDAR_IR_MAX_H     64
#define PROP_LIDAR_IR_MAX_CELLS (PROP_LIDAR_IR_MAX_W * PROP_LIDAR_IR_MAX_H)

typedef struct {
    prop_lidar_link_t link;
    /* The rig's own render rate, straight out of thin_telemetry. This is NOT what the
     * panel receives: at 460 KB/frame uncompressed the CrowPanel's link delivers a small
     * fraction of it, so never present this as the panel's frame rate — use link_fps. */
    float             fps;
    /* Frames per second actually delivered to THIS client, measured locally from frame
     * arrivals. Decays toward 0 while nothing is landing. */
    float             link_fps;
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
    /* Dimensions of the IR grid the last thin_telemetry carried, 0 if none yet. The
     * cells themselves are NOT in this struct: at full resolution they are 2-4 KB, and
     * this struct is a by-value copy that callers put on their stack. Fetch them with
     * prop_lidar_get_ir_grid() instead. */
    uint16_t          ir_w;
    uint16_t          ir_h;
    /* Negotiated wire protocol (thin_hello_ack). proto is 1 until a v2 server acks,
     * jpeg says whether frames arrive as tag-2 JPEG rather than tag-1 raw RGB565.
     * A v1 server never acks, so these stay 1/false and the link runs the old way. */
    uint8_t           proto;
    bool              jpeg;
    /* Per-client send stats straight out of thin_telemetry (v2 servers only; zero
     * otherwise). tx_fps is what the RIG says it sent us and should track link_fps;
     * dropped counts frames flow control discarded for us, which is expected to be
     * large and growing on this link -- that is credit control working, not failing. */
    float             tx_fps;
    int               tx_bytes_per_s;
    int               dropped;
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

/* Copies the newest IR zone grid (row-major, one uint8 0..255 per zone) into `out`,
 * and its shape into `out_w` and `out_h`. Returns the number of cells written, or 0 if
 * no grid has arrived yet or `max_cells` is too small to hold the current one — in which
 * case all three outputs are left alone, so a caller can simply keep displaying the last
 * good grid. Separate from prop_lidar_get_telemetry so the (potentially multi-KB) cells
 * never ride on a caller's stack. */
int prop_lidar_get_ir_grid(uint8_t *out, int max_cells, int *out_w, int *out_h);

/* Copies the current cached telemetry. Always succeeds; before the first thin_telemetry
 * message arrives it reports link=PROP_LIDAR_LINK_SEARCHING and zeroed fields. */
void prop_lidar_get_telemetry(prop_lidar_telemetry_t *out);

/* Tells the link whether the LIDAR panel is currently on screen. The stream costs
 * ~3.7 Mbit per frame on a link that carries ~1-2 Mbit/s, so it is only held open while
 * something is actually rendering it; running it behind another screen starves the rest
 * of the board's networking. Safe to call from the UI thread under the LVGL lock — it
 * only sets a flag and an event bit, never touches the socket. The connection lingers
 * for a short grace period after false() so quick navigation away and back is free. */
void prop_lidar_set_active(bool active);

/* Tells the link that the UI has finished with the frame carrying `seq` (the value
 * prop_lidar_peek_frame handed back). That is what grants the server a v2 send credit
 * (`thin_ready`), so on a v2 link frames STOP arriving if this is never called --
 * deliberately: credit is meant to reflect real consumption, not mere receipt. Call it
 * after the canvas has been retargeted, not on arrival. Cheap and non-blocking; safe
 * from the UI thread under the LVGL lock. Harmless on a v1 link (nothing is sent). */
void prop_lidar_frame_consumed(uint32_t seq);

/* Overrides the roomscanner endpoint, persisted in NVS (`lidar_host`). Accepts
 * "host" or "host:port"; an empty string or NULL clears the override and returns the
 * link to mDNS discovery (`_roomscan._tcp`), which is the default. Drops the current
 * session so the change takes effect immediately rather than at the next failure.
 * This is the escape hatch for networks where mDNS does not work. */
esp_err_t prop_lidar_set_host(const char *host);

/* Sets the JPEG quality this client asks the rig for, persisted in NVS (`lidar_q`) so it
 * survives a reflash. 40..95, or 0 to clear the override and let the rig choose — which
 * is the default and usually the right answer, since the rig is the side that can trade
 * quality against frame rate. Returns ESP_ERR_INVALID_ARG for anything else. Applies to
 * the live session immediately by re-running the handshake, not only on reconnect. */
esp_err_t prop_lidar_set_quality(int quality);

/* Best-effort outbound commands; no-ops quickly if not currently connected. */
void prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom);
void prop_lidar_send_mode(prop_lidar_mode_t mode);
void prop_lidar_send_record(bool on);

#ifdef __cplusplus
}
#endif
