/* prop_lidar — thin-client render link to lidar-roomscanner's /ws-thin endpoint.
 * See prop_lidar.h for the public API, docs/superpowers/specs/
 * 2026-08-17-lidar-thin-client-crowpanel-design.md for the v1 protocol contract and
 * 2026-08-18-lidar-thin-frame-bandwidth-protocol.md for the v2 one (credit flow
 * control + JPEG frames) this speaks when the server answers the handshake. */
#include "prop_lidar.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mdns.h"
#include "esp_websocket_client.h"
#include "driver/jpeg_decode.h"
#include "mbedtls/base64.h"
#include "prop_settings.h"

#define TAG "PROP_LIDAR"

#define FRAME_BYTES        (PROP_LIDAR_FRAME_PIXELS * 2)

/* "Stale" has to be measured against what this link can actually deliver, not against
 * an aspirational cadence. One THIN_FRAME is 460 KB = 3.7 Mbit; the CrowPanel's C6 link
 * measures ~1-2 Mbit/s, so the best case interframe gap is 2-4 s. The old fixed 2000 ms
 * threshold was therefore BELOW the healthy interframe time and the panel read STALE
 * permanently. Instead: stale = no frame for 3x this session's own measured cadence,
 * clamped, so a slow-but-working link reads OK and a genuinely wedged one still trips. */
#define STALE_MIN_MS       3000
#define STALE_MAX_MS       15000
#define STALE_DT_FACTOR    3.0f

/* How long the socket is held open after the LIDAR panel closes. Long enough that
 * flicking away and straight back doesn't pay a full mDNS + TCP + first-frame round
 * trip; short enough that background streaming can't hog the radio. */
#define IDLE_GRACE_MS      20000

#define RECONNECT_BACKOFF_MIN_MS 1000
#define RECONNECT_BACKOFF_MAX_MS 30000

/* Wire framing. Both tags are one unfragmented WS binary message; the client
 * reassembles from payload_len/payload_offset across WEBSOCKET_EVENT_DATA chunks.
 *   tag 1 THIN_FRAME      u32 tag, u16 w, u16 h, then w*h*2 bytes of raw RGB565
 *   tag 2 THIN_FRAME_JPEG u32 tag, u16 w, u16 h, u32 seq, u32 len, then len JPEG bytes */
#define THIN_TAG_RAW       1u
#define THIN_TAG_JPEG      2u
#define RX_HEADER_BYTES    8    /* u32 tag + u16 w + u16 h */
#define RX_HEADER_JPEG     16   /* ... + u32 seq + u32 payload_len */
#define RAW_MSG_BYTES      (RX_HEADER_BYTES + FRAME_BYTES)

/* Ceiling advertised to the server as max_frame_bytes: a whole tag-2 message it may
 * send us. A 480x480 quality-75 4:2:0 frame of mostly-black point cloud measures
 * ~15-30 KB, so this is ~10x headroom; the server drops (and counts) anything bigger
 * rather than sending a message this client could not reassemble. */
#define MAX_JPEG_MSG_BYTES 262144

/* One RX buffer serves both tags, so it has to fit the larger — which is still the
 * v1 raw frame. The spec's "the RX buffer shrinks to max_frame_bytes" only holds for
 * a client that has dropped v1 support, and the compatibility bar here is explicitly
 * "the v2 client still renders against a v1 server". It is PSRAM either way. */
#define RX_BUF_BYTES       ((RAW_MSG_BYTES > (RX_HEADER_JPEG + MAX_JPEG_MSG_BYTES)) \
                            ? RAW_MSG_BYTES : (RX_HEADER_JPEG + MAX_JPEG_MSG_BYTES))

/* Send credits requested in thin_hello: frames the server may have outstanding to us
 * at once. History, all measured 2026-08-19 on the live rig:
 *   2 -> 4: bytes/s sat at ~30% of the ~1.5 Mbit/s link budget at 2 credits (~57 KB/s),
 *   so the ceiling was round-trip latency (see LIDAR_EVT_CREDIT below), not the wire;
 *   4 credits measured 9.8-11.7 fps at ~88.5 KB/s -- still only ~47% of budget (~9 KB
 *   frames put the bandwidth-only ceiling at ~20 fps), so raised again to 6.
 * Deeper pipelining trades frame staleness for throughput while there is bandwidth to
 * spend; going much beyond this starts recreating the queueing problem the credit
 * scheme exists to remove. Re-measure (bytes/s vs budget, `dropped`, and whether the
 * picture visibly lags orbit input) before going higher than this. */
#define LIDAR_CREDITS      6

/* Requested frame rate in thin_hello. Measured 2026-08-19, three steps on the live rig:
 *   default (unset)   -> flat 10.00 fps, ~90 KB/s (~48% of budget), zero drops:
 *                        the rig's own DEFAULT_THIN_FPS, not a real ceiling.
 *   15 requested       -> ~11.8 fps avg, ~107-120 KB/s.
 *   60 requested (probe, the protocol's own max for jpeg -- THIN_HELLO_FPS_MAX_JPEG)
 *                       -> ~16.7 fps avg (14.2-18.2), ~151 KB/s avg (128-166 KB/s peak),
 *                          i.e. ~81% of the ~187 KB/s (~1.5 Mbit/s) link budget, and
 *                          `dropped` climbing fast (~170-190 every 5s): the rig was
 *                          rendering far more than the link could carry and discarding
 *                          the rest, exactly as credit flow control is supposed to.
 *                          That plateau (not the 60 asked for) is the real ceiling.
 * 20 lands just above the observed ~16.7-18.2 plateau: asking for materially more than
 * this buys no further delivered fps (proven by the 60 probe above), it only wastes the
 * rig's render cycles on frames that get thrown away, so there is no reason to leave the
 * request at the probe value. Re-measure before changing this again — link conditions,
 * frame content (mode/quality), or a rig-side change could all move the real ceiling. */
#define LIDAR_REQUEST_FPS  20

/* Reassembly buffer for JSON text messages. Sized from the biggest one that can arrive:
 * a thin_telemetry carrying a full-resolution IR grid, which is PROP_LIDAR_IR_MAX_CELLS
 * bytes base64'd (4 chars per 3 bytes) plus the rest of the message. That slack is not
 * decorative — with an 8x8 grid the message measures ~660 B, but at the 54x42 the rig
 * actually has it is ~3.5 KB, and an undersized buffer here drops the WHOLE message,
 * losing every telemetry field rather than just the IR. Anything over is dropped and
 * logged rather than silently truncated. Separate from the frame buffer on purpose: the
 * two message kinds are never interleaved on the wire, but one buffer each means a
 * future change to that assumption cannot corrupt a frame with half a telemetry message. */
#define TEXT_BUF_BYTES     ((((PROP_LIDAR_IR_MAX_CELLS + 2) / 3) * 4) + 2048)

/* WS RX/TX buffer. A whole raw THIN_FRAME is ~460 KB, so it always arrives split across
 * several WEBSOCKET_EVENT_DATA callbacks; a bigger buffer just means fewer of them
 * (~15 instead of ~113 at 4 KB). The component malloc()s two of these, and with
 * CONFIG_SPIRAM_USE_MALLOC + CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096 an allocation
 * this large lands in PSRAM, so it does not eat the tight internal-RAM budget. */
#define WS_BUFFER_SIZE     32768

/* NVS key holding an optional "host:port" (or "host") override for the roomscanner.
 * When set, mDNS discovery is skipped entirely — the escape hatch for networks where
 * mDNS doesn't work. Nothing in firmware writes it today (see the design notes). */
#define LIDAR_HOST_KEY     "lidar_host"

/* NVS key holding an optional JPEG quality to request in thin_hello, so the picture can
 * be tuned on a board without a reflash. Unset (or out of range) means send no `quality`
 * at all and let the rig pick — which is what the spec asks for by default, since the rig
 * is the side that can actually trade quality against rate. The server clamps to 40..95;
 * this checks the same range so an obvious typo is ignored rather than silently clamped.
 * Measured reference: q75 renders this scene at ~9 KB/frame on a ~187 KB/s budget, so
 * there is real headroom to spend here if a sharper picture is worth the frame rate. */
#define LIDAR_QUALITY_KEY  "lidar_q"
#define LIDAR_QUALITY_MIN  40
#define LIDAR_QUALITY_MAX  95

/* ---- Cached state (written by the task, read by getters under s_lock) ---------- */
static uint8_t *s_rx_buf;      /* PSRAM, RX_BUF_BYTES — one whole WS message of either tag */
static char    *s_text_buf;    /* PSRAM, TEXT_BUF_BYTES — one whole JSON text message */
static SemaphoreHandle_t s_lock;

/* lidar_task's wait/wake event group. Declared here (ahead of lidar_task itself) because
 * prop_lidar_frame_consumed(), called from the UI thread, needs to post to it. */
static EventGroupHandle_t s_evt;
#define LIDAR_EVT_DISCONNECTED (1 << 0)
#define LIDAR_EVT_ACTIVE       (1 << 1)
/* Wakes lidar_task the instant the upgrade completes so thin_hello goes out immediately
 * rather than up to LIDAR_POLL_MS later — every tick of delay is a free-running v1 frame
 * (460 KB on a 1.5 Mbit/s link) the server sends before it knows better. */
#define LIDAR_EVT_CONNECTED    (1 << 2)
/* Wakes lidar_task the instant the UI grants a credit (prop_lidar_frame_consumed),
 * instead of waiting up to LIDAR_POLL_MS for the next poll to call pump_ready(). That
 * poll-cadence lag was, on measurement 2026-08-19, the actual fps ceiling on this link
 * — bytes/s to the panel sat at ~30% of the ~1.5 Mbit/s budget, so the wire had slack;
 * the credit round trip (UI-observer lag + this poll lag + network RTT) was pacing it
 * instead. Harmless to set from a session with no socket yet / a stale prior session —
 * pump_ready() no-ops fast if there is nothing to grant. */
#define LIDAR_EVT_CREDIT       (1 << 3)

/* Triple buffer: the writer always fills (front+1)%3, so the two most recently
 * published buffers are never written. A zero-copy reader (the LIDAR canvas) that
 * keeps up within one flip can therefore never be scribbled on mid-render; even a
 * reader a full flip behind is safe. */
static uint16_t *s_frame_buf[3];   /* PSRAM triple buffer, FRAME_BYTES each */
static int       s_frame_front;    /* index of the buffer readers should use */
static uint32_t  s_frame_seq;      /* increments each time a new frame lands */
static uint32_t  s_last_frame_ms;  /* esp_timer ms at the last complete frame */

/* Arrival times of the last FPS_WIN frames, for the delivered-rate estimate. A short
 * EMA was tried first and proved useless here: delivery is bursty (a few frames close
 * together, then a multi-second gap while the link drains), so a 4-sample EMA sampled at
 * a random instant swung between 0.15 and 1.5 fps on a stream averaging 0.45. A flat
 * count over a window spanning several seconds is both stabler and easier to defend. */
#define FPS_WIN 8
static uint32_t s_frame_ts[FPS_WIN];
static uint8_t  s_frame_ts_n;      /* valid entries, saturates at FPS_WIN */
static uint8_t  s_frame_ts_i;      /* next write slot */

static prop_lidar_telemetry_t s_telemetry;

/* ---- v2 protocol state -------------------------------------------------------------
 * Written by the websocket event task (handshake ack, frame arrivals) and read by
 * lidar_task and the public getters. Everything a reader could see torn is behind
 * s_lock; the two scalars below are single-word flags whose worst case is that
 * lidar_task acts on them one 100 ms poll late. */
static volatile uint8_t s_proto;         /* 1 = v1/free-running, 2 = credit flow control */
static volatile bool    s_encoding_jpeg; /* server acked encoding "jpeg" -> expect tag 2 */

/* thin_ready accounting, in the same units as s_frame_seq (see prop_lidar_frame_consumed).
 * s_ready_target is the newest seq the UI has finished with; s_ready_acked is how far the
 * grants have got. The gap is how many credits we still owe the server -- one per frame
 * it sent us and we consumed. Both are re-based to s_frame_seq at every CONNECTED, since
 * credit is per-connection and the server starts a fresh session at credits_max. */
static uint32_t s_ready_target;   /* under s_lock (UI thread writes it) */
static uint32_t s_ready_acked;    /* lidar_task only */

/* Hardware JPEG decoder (esp_driver_jpeg). Lazily created on the first tag-2 frame so a
 * v1-only deployment never claims the peripheral, then kept for the life of the boot --
 * it is a shared engine with a mutex inside, and re-creating it per session is churn for
 * nothing. Touched only from the websocket event task, which is the only decoder. */
static jpeg_decoder_handle_t s_jpegd;
static size_t s_frame_buf_bytes;  /* allocated size of ONE frame buffer, >= FRAME_BYTES */

/* Newest IR zone grid, row-major, one byte per zone. PSRAM rather than .bss because at
 * full resolution this is a few KB and it is only ever touched at telemetry rate. Read
 * and written under s_lock; s_ir_w/s_ir_h are 0 until the first grid arrives. */
static uint8_t *s_ir_cells;
static uint16_t s_ir_w, s_ir_h;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* All three helpers below are called with s_lock held. */

/* Milliseconds spanned by the arrival window, and the completed intervals in it. */
static uint32_t frame_window_span_ms(uint8_t *out_intervals)
{
    *out_intervals = (s_frame_ts_n >= 2) ? (uint8_t)(s_frame_ts_n - 1) : 0;
    if (s_frame_ts_n < 2) return 0;
    uint32_t newest = s_frame_ts[(s_frame_ts_i + FPS_WIN - 1) % FPS_WIN];
    uint32_t oldest = s_frame_ts[(s_frame_ts_i + FPS_WIN - s_frame_ts_n) % FPS_WIN];
    return newest - oldest;
}

/* Mean interframe gap over the window, or 0 if not enough samples yet. */
static uint32_t mean_frame_dt_ms(void)
{
    uint8_t n;
    uint32_t span = frame_window_span_ms(&n);
    return n ? (span / n) : 0;
}

/* How long without a frame counts as stale, derived from this session's own cadence. */
static uint32_t stale_threshold_ms(void)
{
    uint32_t dt = mean_frame_dt_ms();
    if (dt == 0) return STALE_MIN_MS;
    float t = (float)dt * STALE_DT_FACTOR;
    if (t < (float)STALE_MIN_MS) return STALE_MIN_MS;
    if (t > (float)STALE_MAX_MS) return STALE_MAX_MS;
    return (uint32_t)t;
}

/* Frames per second actually DELIVERED to this client — not the rig's internal render
 * rate, which is what thin_telemetry's "fps" reports and which can be 20x higher.
 * Counts completed intervals over [oldest sample, now], so the in-progress gap is part
 * of the denominator: the number sags while a frame is overdue and decays toward zero if
 * one never lands, instead of freezing at the last good cadence. */
static float measured_link_fps(void)
{
    uint8_t n;
    uint32_t span = frame_window_span_ms(&n);
    if (!n) return 0.0f;
    float total = (float)span + (float)(now_ms() - s_last_frame_ms);
    return (total >= 1.0f) ? ((float)n * 1000.0f / total) : 0.0f;
}

/* ---- Public getters -------------------------------------------------------------- */

const uint16_t *prop_lidar_peek_frame(uint32_t *out_seq)
{
    if (!s_lock) return NULL;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const uint16_t *front = (s_frame_seq > 0) ? s_frame_buf[s_frame_front] : NULL;
    if (out_seq) *out_seq = s_frame_seq;
    xSemaphoreGive(s_lock);
    return front;
}

uint32_t prop_lidar_get_seq(void)
{
    if (!s_lock) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t seq = s_frame_seq;
    xSemaphoreGive(s_lock);
    return seq;
}

int prop_lidar_get_ir_grid(uint8_t *out, int max_cells, int *out_w, int *out_h)
{
    if (!out || !s_lock || !s_ir_cells) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = (int)s_ir_w * (int)s_ir_h;
    if (n <= 0 || n > max_cells) {
        xSemaphoreGive(s_lock);
        return 0;
    }
    memcpy(out, s_ir_cells, (size_t)n);
    if (out_w) *out_w = s_ir_w;
    if (out_h) *out_h = s_ir_h;
    xSemaphoreGive(s_lock);
    return n;
}

void prop_lidar_get_telemetry(prop_lidar_telemetry_t *out)
{
    if (!out) return;
    if (!s_lock) {
        memset(out, 0, sizeof(*out));
        out->link = PROP_LIDAR_LINK_SEARCHING;
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_telemetry;
    /* Both computed live rather than stored, so they stay honest between the ~2 Hz
     * thin_telemetry messages that overwrite s_telemetry wholesale. */
    out->link_fps = measured_link_fps();
    if (out->link == PROP_LIDAR_LINK_OK &&
        (now_ms() - s_last_frame_ms) > stale_threshold_ms()) {
        out->link = PROP_LIDAR_LINK_STALE;
    }
    xSemaphoreGive(s_lock);
    /* Negotiation results live outside s_telemetry precisely because that struct is
     * replaced wholesale by every thin_telemetry message — folding them in here keeps
     * them from being clobbered by the next one. */
    out->proto = s_proto ? s_proto : 1;
    out->jpeg  = s_encoding_jpeg;
}

void prop_lidar_frame_consumed(uint32_t seq)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Monotonic, and only ever forward: a seq from a previous session (or a repeat of
     * one already granted) must not hand the server extra credit. Taking the max also
     * means a UI that skipped a frame — consumed N then N+2 — still returns BOTH
     * credits, which is what stops skips from draining the window to a permanent stall. */
    if (seq > s_ready_target) s_ready_target = seq;
    xSemaphoreGive(s_lock);
    if (s_evt) xEventGroupSetBits(s_evt, LIDAR_EVT_CREDIT);
}

/* ---- Outbound commands ------------------------------------------------------------
 * These are called from the UI thread, frequently while it holds the LVGL port lock
 * (touch/dial callbacks in prop_ui.c). They therefore must NOT touch the network stack
 * or the websocket handle: a blocking send under the LVGL lock stalls the panel, and
 * `s_ws` is created/destroyed by lidar_task, so reading it from another thread races
 * with its own teardown (use-after-free).
 *
 * Instead they only post a small tagged struct onto a queue, never blocking (a full
 * queue drops the command — these are best-effort UI nudges). lidar_task drains the
 * queue and is the ONLY thread that ever touches `s_ws`. */

typedef enum { LC_ORBIT = 0, LC_MODE, LC_RECORD, LC_REHELLO } lidar_cmd_kind_t;

typedef struct {
    lidar_cmd_kind_t kind;
    union {
        struct { float dyaw, dpitch, dzoom; } orbit;
        prop_lidar_mode_t mode;
        bool on;
    } u;
} lidar_cmd_t;

#define LIDAR_CMD_QUEUE_DEPTH 12
static QueueHandle_t s_cmd_q;

static bool send_hello(void);   /* defined with the rest of the task-owned senders */

static void cmd_post(const lidar_cmd_t *c)
{
    if (!s_cmd_q) return;
    (void)xQueueSend(s_cmd_q, c, 0);   /* never block; drop if full */
}

void prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom)
{
    lidar_cmd_t c = { .kind = LC_ORBIT };
    c.u.orbit.dyaw = dyaw;
    c.u.orbit.dpitch = dpitch;
    c.u.orbit.dzoom = dzoom;
    cmd_post(&c);
}

void prop_lidar_send_mode(prop_lidar_mode_t mode)
{
    lidar_cmd_t c = { .kind = LC_MODE };
    c.u.mode = mode;
    cmd_post(&c);
}

void prop_lidar_send_record(bool on)
{
    lidar_cmd_t c = { .kind = LC_RECORD };
    c.u.on = on;
    cmd_post(&c);
}

esp_err_t prop_lidar_set_quality(int quality)
{
    if (quality != 0 && (quality < LIDAR_QUALITY_MIN || quality > LIDAR_QUALITY_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = prop_settings_set_u32(LIDAR_QUALITY_KEY, (uint32_t)quality);
    if (err != ESP_OK) return err;
    /* Re-send thin_hello on the live session rather than waiting for a reconnect: the
     * server applies a pending hello at the top of its next render tick, so the change
     * shows up within one frame instead of after the next 20 s idle-grace teardown.
     * Re-negotiation grants no free credits server-side, so this cannot disturb flow
     * control. Queued like every other command — the UI thread never touches the socket. */
    lidar_cmd_t c = { .kind = LC_REHELLO };
    cmd_post(&c);
    return ESP_OK;
}

/* ---- Host resolution and WebSocket handler ----------------------------------------- */

/* Optional non-mDNS escape hatch: if NVS holds a non-empty "lidar_host" (e.g.
 * "192.168.1.50:8000" or "roomscanner.lan:8000"), build the URI straight from it and
 * skip discovery. Returns true if an override was present and used. */
static bool uri_from_nvs(char *uri_out, size_t uri_out_sz)
{
    char host[64] = {0};
    if (prop_settings_get_str(LIDAR_HOST_KEY, host, sizeof(host), "") != ESP_OK) return false;
    if (host[0] == '\0') return false;
    snprintf(uri_out, uri_out_sz, "ws://%s/ws-thin", host);
    return true;
}

/* Resolve the roomscanner's advertised _roomscan._tcp service to "host:port". Returns
 * true and fills uri_out (e.g. "ws://192.168.4.55:8000/ws-thin") on success. */
static bool resolve_uri(char *uri_out, size_t uri_out_sz)
{
    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_ptr("_roomscan", "_tcp", 3000, 5, &results);
    if (err != ESP_OK || !results) {
        if (results) mdns_query_results_free(results);
        return false;
    }
    bool found = false;
    for (mdns_result_t *r = results; r && !found; r = r->next) {
        for (mdns_ip_addr_t *a = r->addr; a; a = a->next) {
            if (a->addr.type == ESP_IPADDR_TYPE_V4) {
                char ip[16];
                esp_ip4addr_ntoa(&a->addr.u_addr.ip4, ip, sizeof(ip));
                snprintf(uri_out, uri_out_sz, "ws://%s:%u/ws-thin", ip, (unsigned)r->port);
                found = true;
                break;
            }
        }
    }
    mdns_query_results_free(results);
    return found;
}

/* Owned exclusively by lidar_task — never read or written from any other thread. */
static esp_websocket_client_handle_t s_ws;
static volatile bool s_ws_connected;   /* set by the WS event handler, read by lidar_task */

/* Set by the UI when the LIDAR panel opens/closes (prop_lidar_set_active). The stream
 * is ~3.7 Mbit per frame on a link that carries ~1-2 Mbit/s, so leaving it running
 * behind an unrelated screen starves every other network user on the board — /screenshot
 * fetches, the /ws telemetry push, OTA. Written only by the UI thread, read by
 * lidar_task; a one-tick skew either way is harmless. */
static volatile bool     s_active;
static volatile uint32_t s_inactive_since_ms;

/* Set when the endpoint changed under us (prop_lidar_set_host). lidar_task caches the
 * resolved URI across reconnects — mdns_query_ptr blocks for its full 3 s timeout and
 * paying that on every transient drop is a painful chunk of time-to-first-frame — so a
 * new host has to explicitly invalidate that cache or it would never be picked up. */
static volatile bool     s_uri_dirty;

void prop_lidar_set_active(bool active)
{
    if (!s_evt) return;   /* init failed / not started: nothing to gate */
    if (active) {
        s_active = true;
        xEventGroupSetBits(s_evt, LIDAR_EVT_ACTIVE);   /* wake an idling lidar_task */
    } else {
        /* Timestamp first: lidar_task only reads it once it has seen s_active false. */
        s_inactive_since_ms = now_ms();
        s_active = false;
    }
}

esp_err_t prop_lidar_set_host(const char *host)
{
    esp_err_t err = prop_settings_set_str(LIDAR_HOST_KEY, host ? host : "");
    if (err != ESP_OK) return err;
    s_uri_dirty = true;
    /* Drop the current session so the new endpoint is used now rather than whenever the
     * link next happens to fail. Same path a real transport drop takes — lidar_task is
     * still the only thread that touches the socket. */
    if (s_evt) xEventGroupSetBits(s_evt, LIDAR_EVT_DISCONNECTED);
    return ESP_OK;
}

/* Rate-limited warning: a malformed/oversized WS message would otherwise log once per
 * data callback (many per frame). At most one line per second. */
static uint32_t s_last_warn_ms;
static bool warn_ok(void)
{
    uint32_t t = now_ms();
    if (s_last_warn_ms != 0 && (t - s_last_warn_ms) < 1000) return false;
    s_last_warn_ms = t ? t : 1;
    return true;
}

static const char *mode_to_str(prop_lidar_mode_t m)
{
    switch (m) {
        case PROP_LIDAR_MODE_SLAM: return "slam";
        case PROP_LIDAR_MODE_IR:   return "ir";
        default:                   return "point_cloud";
    }
}

static void set_link_state(prop_lidar_link_t link)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_telemetry.link = link;
    xSemaphoreGive(s_lock);
}

/* Flip the freshly-written back buffer to the front and stamp the arrival. The frame
 * itself was written OUTSIDE the lock: only this task ever writes frame buffers, and
 * (front+1)%3 is by construction not a front any reader can be looking at. */
static void publish_frame(int back)
{
    uint32_t t = now_ms();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_frame_ts[s_frame_ts_i] = t;
    s_frame_ts_i = (uint8_t)((s_frame_ts_i + 1) % FPS_WIN);
    if (s_frame_ts_n < FPS_WIN) s_frame_ts_n++;
    s_frame_front = back;
    s_frame_seq++;
    s_last_frame_ms = t;
    xSemaphoreGive(s_lock);
}

/* Hardware-decode one baseline 4:2:0 JPEG straight into a frame buffer. The P4's JPEG
 * block writes RGB565 by DMA, so the decoded frame lands in the same PSRAM the canvas
 * already points at — no intermediate bitmap and no CPU colour conversion. */
static bool decode_jpeg_into(uint16_t *dst, const uint8_t *jpg, uint32_t len)
{
    if (!s_jpegd) {
        /* 250 ms is ~20x the measured cost of a 480x480 decode; it exists only so a
         * wedged engine surfaces as a failed frame instead of hanging the RX task. */
        jpeg_decode_engine_cfg_t eng = { .timeout_ms = 250 };
        esp_err_t e = jpeg_new_decoder_engine(&eng, &s_jpegd);
        if (e != ESP_OK) {
            s_jpegd = NULL;
            if (warn_ok()) ESP_LOGE(TAG, "jpeg_new_decoder_engine failed: %s", esp_err_to_name(e));
            return false;
        }
        ESP_LOGI(TAG, "hardware JPEG decoder up");
    }

    /* Header-parse first: cheap, needs no hardware, and it is the only thing that can
     * tell us the JPEG's real dimensions rather than the ones the frame header claims.
     * A mismatch would otherwise decode into the wrong stride and render as garbage. */
    jpeg_decode_picture_info_t info;
    esp_err_t e = jpeg_decoder_get_info(jpg, len, &info);
    if (e != ESP_OK) {
        if (warn_ok()) ESP_LOGW(TAG, "jpeg header parse failed: %s", esp_err_to_name(e));
        return false;
    }
    if (info.width != PROP_LIDAR_FRAME_W || info.height != PROP_LIDAR_FRAME_H) {
        if (warn_ok()) {
            ESP_LOGW(TAG, "jpeg is %ux%u, expected %dx%d — dropping (renegotiate size)",
                     (unsigned)info.width, (unsigned)info.height,
                     PROP_LIDAR_FRAME_W, PROP_LIDAR_FRAME_H);
        }
        return false;
    }

    /* rgb_order BGR is esp_driver_jpeg's spelling of "small endian" for RGB565 (it
     * selects the byte-swapped DMA2D scramble, not a channel swap) — i.e. the native
     * little-endian RGB565 the canvas and the panel both take with swap_bytes=false.
     * If a decoded frame ever comes out blue-for-red, this enum is the one to flip. */
    const jpeg_decode_cfg_t cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order     = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std      = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t out = 0;
    e = jpeg_decoder_process(s_jpegd, &cfg, jpg, len,
                             (uint8_t *)dst, s_frame_buf_bytes, &out);
    if (e != ESP_OK) {
        if (warn_ok()) ESP_LOGW(TAG, "jpeg decode failed: %s", esp_err_to_name(e));
        return false;
    }
    if (out != FRAME_BYTES) {
        if (warn_ok()) ESP_LOGW(TAG, "jpeg decoded %u B, expected %u", (unsigned)out, (unsigned)FRAME_BYTES);
        return false;
    }
    return true;
}

static void on_frame_complete(const uint8_t *buf, int len)
{
    if (len < (int)RX_HEADER_BYTES) return;
    uint32_t tag; uint16_t w, h;
    memcpy(&tag, buf + 0, 4);
    memcpy(&w,   buf + 4, 2);
    memcpy(&h,   buf + 6, 2);
    if (w != PROP_LIDAR_FRAME_W || h != PROP_LIDAR_FRAME_H) {
        if (warn_ok()) ESP_LOGW(TAG, "bad frame header tag=%u w=%u h=%u", (unsigned)tag, w, h);
        return;
    }

    int back = (s_frame_front + 1) % 3;

    if (tag == THIN_TAG_RAW) {
        if (len != (int)RAW_MSG_BYTES) {
            if (warn_ok()) ESP_LOGW(TAG, "THIN_FRAME is %d B, expected %d", len, (int)RAW_MSG_BYTES);
            return;
        }
        memcpy(s_frame_buf[back], buf + RX_HEADER_BYTES, FRAME_BYTES);
    } else if (tag == THIN_TAG_JPEG) {
        if (len < (int)RX_HEADER_JPEG) {
            if (warn_ok()) ESP_LOGW(TAG, "THIN_FRAME_JPEG truncated (%d B)", len);
            return;
        }
        uint32_t payload_len;
        memcpy(&payload_len, buf + 12, 4);   /* buf+8 is the server's seq, informational */
        if (payload_len == 0 || payload_len != (uint32_t)(len - RX_HEADER_JPEG)) {
            if (warn_ok()) {
                ESP_LOGW(TAG, "THIN_FRAME_JPEG payload_len=%u but %d B followed",
                         (unsigned)payload_len, len - (int)RX_HEADER_JPEG);
            }
            return;
        }
        if (!decode_jpeg_into(s_frame_buf[back], buf + RX_HEADER_JPEG, payload_len)) return;
    } else {
        /* A tag this build does not know, or one that was never negotiated. Dropping it
         * (rather than guessing) is what lets the server add frame types safely. */
        if (warn_ok()) ESP_LOGW(TAG, "unknown frame tag=%u (%d B)", (unsigned)tag, len);
        return;
    }

    publish_frame(back);
}

static prop_lidar_mode_t mode_from_str(const char *s)
{
    if (!s) return PROP_LIDAR_MODE_POINT_CLOUD;
    if (strcmp(s, "slam") == 0) return PROP_LIDAR_MODE_SLAM;
    if (strcmp(s, "ir") == 0)   return PROP_LIDAR_MODE_IR;
    return PROP_LIDAR_MODE_POINT_CLOUD;
}

/* thin_hello_ack — the server's authoritative answer to our thin_hello. Everything in
 * it is the CLAMPED effective value, not what we asked for, so it is read rather than
 * assumed. A v1 server never sends one, which is exactly why the absence of this
 * message (not a timeout) is what selects the v1 path. */
static void on_hello_ack(const cJSON *root)
{
    const cJSON *proto = cJSON_GetObjectItem(root, "proto");
    const cJSON *enc   = cJSON_GetObjectItem(root, "encoding");
    const cJSON *w     = cJSON_GetObjectItem(root, "width");
    const cJSON *h     = cJSON_GetObjectItem(root, "height");
    const cJSON *cred  = cJSON_GetObjectItem(root, "credits");

    s_proto = (cJSON_IsNumber(proto) && proto->valueint >= 2) ? 2 : 1;
    s_encoding_jpeg = cJSON_IsString(enc) && strcmp(enc->valuestring, "jpeg") == 0;

    /* Size is negotiable in the protocol but not in this client: the canvas and the
     * triple buffer are both fixed at PROP_LIDAR_FRAME_W/H. Say so loudly rather than
     * silently dropping every frame, which is what a mismatch would otherwise look like. */
    if ((cJSON_IsNumber(w) && w->valueint != PROP_LIDAR_FRAME_W) ||
        (cJSON_IsNumber(h) && h->valueint != PROP_LIDAR_FRAME_H)) {
        ESP_LOGE(TAG, "server forced %dx%d frames; this client only renders %dx%d",
                 cJSON_IsNumber(w) ? w->valueint : -1, cJSON_IsNumber(h) ? h->valueint : -1,
                 PROP_LIDAR_FRAME_W, PROP_LIDAR_FRAME_H);
    }
    ESP_LOGI(TAG, "thin_hello_ack: proto=%u encoding=%s credits=%d",
             (unsigned)s_proto, s_encoding_jpeg ? "jpeg" : "rgb565",
             cJSON_IsNumber(cred) ? cred->valueint : -1);
}

static void on_text_message(const char *text, int len)
{
    cJSON *root = cJSON_ParseWithLength(text, len);
    if (!root) return;
    const cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(type)) {
        cJSON_Delete(root);
        return;
    }
    if (strcmp(type->valuestring, "thin_hello_ack") == 0) {
        on_hello_ack(root);
        cJSON_Delete(root);
        return;
    }
    if (strcmp(type->valuestring, "thin_telemetry") != 0) {
        cJSON_Delete(root);
        return;
    }
    prop_lidar_telemetry_t t = {0};
    t.link = PROP_LIDAR_LINK_OK;
    const cJSON *fps   = cJSON_GetObjectItem(root, "fps");
    const cJSON *pts   = cJSON_GetObjectItem(root, "point_count");
    const cJSON *rec   = cJSON_GetObjectItem(root, "recording");
    const cJSON *mode  = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsNumber(fps)) t.fps = (float)fps->valuedouble;
    if (cJSON_IsNumber(pts)) t.point_count = pts->valueint;
    if (cJSON_IsBool(rec))   t.recording = cJSON_IsTrue(rec);
    if (cJSON_IsString(mode)) t.mode = mode_from_str(mode->valuestring);

    /* Per-client send stats (v2 servers). These are the counterpart to link_fps: the
     * rig's view of what it sent US, next to our view of what actually landed. Absent
     * on a v1 server, which leaves them zeroed. */
    const cJSON *txf = cJSON_GetObjectItem(root, "tx_fps");
    const cJSON *txb = cJSON_GetObjectItem(root, "tx_bytes_per_s");
    const cJSON *drp = cJSON_GetObjectItem(root, "dropped");
    if (cJSON_IsNumber(txf)) t.tx_fps = (float)txf->valuedouble;
    if (cJSON_IsNumber(txb)) t.tx_bytes_per_s = txb->valueint;
    if (cJSON_IsNumber(drp)) t.dropped = drp->valueint;

    /* Rig orientation. heading/pitch/roll/yaw-rate are only meaningful while
     * orientation_valid is true (vertical boresight / uncalibrated clears it). */
    const cJSON *hdg  = cJSON_GetObjectItem(root, "heading_deg");
    const cJSON *pit  = cJSON_GetObjectItem(root, "pitch_deg");
    const cJSON *rol  = cJSON_GetObjectItem(root, "roll_deg");
    const cJSON *yr   = cJSON_GetObjectItem(root, "yaw_rate_dps");
    const cJSON *ovld = cJSON_GetObjectItem(root, "orientation_valid");
    if (cJSON_IsNumber(hdg)) t.heading_deg  = (float)hdg->valuedouble;
    if (cJSON_IsNumber(pit)) t.pitch_deg    = (float)pit->valuedouble;
    if (cJSON_IsNumber(rol)) t.roll_deg     = (float)rol->valuedouble;
    if (cJSON_IsNumber(yr))  t.yaw_rate_dps = (float)yr->valuedouble;
    if (cJSON_IsBool(ovld))  t.orientation_valid = cJSON_IsTrue(ovld);

    /* IR preview grid. Two shapes on the wire and exactly one is ever populated:
     *   ir_grid_b64 + ir_grid_w/h — the sensor's native zone grid, base64 uint8s,
     *                               which is what this client asks for via ir_cells;
     *   ir_grid                   — the v1 8x8 list of ints, from a server that
     *                               predates the negotiation or could not fit ours.
     * Both land in the same s_ir_cells buffer, so nothing downstream cares which
     * arrived. A message carrying NEITHER (no IR data on the rig right now) leaves the
     * previous grid in place — the panel keeps showing the last real image rather than
     * blanking, which is also what the v1 code did. */
    const cJSON *irb  = cJSON_GetObjectItem(root, "ir_grid_b64");
    const cJSON *irw  = cJSON_GetObjectItem(root, "ir_grid_w");
    const cJSON *irh  = cJSON_GetObjectItem(root, "ir_grid_h");
    const cJSON *ir   = cJSON_GetObjectItem(root, "ir_grid");
    if (s_ir_cells && cJSON_IsString(irb) && cJSON_IsNumber(irw) && cJSON_IsNumber(irh) &&
        irw->valueint > 0 && irh->valueint > 0 &&
        (long)irw->valueint * irh->valueint <= PROP_LIDAR_IR_MAX_CELLS) {
        size_t want = (size_t)irw->valueint * (size_t)irh->valueint;
        size_t got = 0;
        const char *b64 = irb->valuestring;
        int rc = mbedtls_base64_decode(NULL, 0, &got, (const unsigned char *)b64, strlen(b64));
        /* The NULL/0 probe returns BUFFER_TOO_SMALL and the true length in `got`;
         * checking it against w*h first means a lying header cannot overrun. */
        if ((rc == MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL || rc == 0) && got == want) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (mbedtls_base64_decode(s_ir_cells, PROP_LIDAR_IR_MAX_CELLS, &got,
                                      (const unsigned char *)b64, strlen(b64)) == 0) {
                s_ir_w = (uint16_t)irw->valueint;
                s_ir_h = (uint16_t)irh->valueint;
            }
            xSemaphoreGive(s_lock);
        } else if (warn_ok()) {
            ESP_LOGW(TAG, "ir_grid_b64 decodes to %u B, header says %ux%u",
                     (unsigned)got, (unsigned)irw->valueint, (unsigned)irh->valueint);
        }
    } else if (s_ir_cells && cJSON_IsArray(ir) && cJSON_GetArraySize(ir) == 64) {
        uint8_t tmp[64];
        int i = 0;
        const cJSON *cell;
        cJSON_ArrayForEach(cell, ir) {
            int v = cJSON_IsNumber(cell) ? cell->valueint : 0;
            tmp[i++] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        memcpy(s_ir_cells, tmp, sizeof(tmp));
        s_ir_w = s_ir_h = 8;
        xSemaphoreGive(s_lock);
    }
    cJSON_Delete(root);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Carry the IR shape across the wholesale replace: it is owned by the grid buffer
     * above, not by the message, and a message with no IR data must not zero it. */
    t.ir_w = s_ir_w;
    t.ir_h = s_ir_h;
    s_telemetry = t;
    xSemaphoreGive(s_lock);
}

static void ws_event_handler(void *handler_args, esp_event_base_t base,
                              int32_t event_id, void *event_data)
{
    (void)handler_args; (void)base;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "ws connected");
            s_ws_connected = true;
            /* Restart the cadence measurement with the session: the previous session's
             * interframe EMA says nothing about this one, and seeding s_last_frame_ms
             * here means the stale timer counts from the handshake rather than instantly
             * tripping on a s_last_frame_ms that is minutes old. */
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_telemetry.link = PROP_LIDAR_LINK_OK;
            s_frame_ts_n = 0;
            s_frame_ts_i = 0;
            s_last_frame_ms = now_ms();
            /* Credit is per-connection: the server grants credits_max afresh on the
             * handshake, so any grants this client still owed the OLD session must not
             * carry over. Re-basing both ends of the accounting to the current frame
             * counter makes the new session start owing exactly nothing. */
            s_ready_target = s_frame_seq;
            s_ready_acked  = s_frame_seq;
            xSemaphoreGive(s_lock);
            /* Until this session's own thin_hello_ack lands, assume v1 — that is what a
             * server that never answers the handshake is, and it is also the safe read
             * for the window before the ack arrives on one that does. */
            s_proto = 1;
            s_encoding_jpeg = false;
            xEventGroupSetBits(s_evt, LIDAR_EVT_CONNECTED);
            break;
        /* Any way the session can end has to wake lidar_task, or it waits forever and
         * the link never comes back: DISCONNECTED (transport drop), ERROR, CLOSED (clean
         * server-initiated close handshake) and FINISH (client task about to exit). */
        case WEBSOCKET_EVENT_DISCONNECTED:
        case WEBSOCKET_EVENT_ERROR:
        case WEBSOCKET_EVENT_CLOSED:
        case WEBSOCKET_EVENT_FINISH:
            ESP_LOGW(TAG, "ws disconnected/error/closed (event %d)", (int)event_id);
            s_ws_connected = false;
            xEventGroupSetBits(s_evt, LIDAR_EVT_DISCONNECTED);
            break;
        case WEBSOCKET_EVENT_DATA: {
            /* esp_websocket_client semantics (see esp_websocket_client.h):
             *   data_len       = length of THIS chunk (capped at cfg.buffer_size)
             *   payload_len    = TOTAL message length across all chunks
             *   payload_offset = this chunk's offset into that total
             * A 460 KB THIN_FRAME therefore always arrives as several chunks. */
            esp_websocket_event_data_t *d = (esp_websocket_event_data_t *)event_data;
            if (d->op_code == 0x2 /* binary */) {
                /* Message length is no longer a single constant: a raw frame is exactly
                 * RAW_MSG_BYTES, a JPEG one is 16 + however many bytes the encoder
                 * produced. So the bound here is only "fits the buffer"; on_frame_complete
                 * is what checks the length against the tag it actually got. */
                if (d->payload_len >= (int)RX_HEADER_BYTES &&
                    d->payload_len <= (int)RX_BUF_BYTES &&
                    d->payload_offset >= 0 && d->data_len >= 0 &&
                    d->payload_offset + d->data_len <= d->payload_len) {
                    memcpy(s_rx_buf + d->payload_offset, d->data_ptr, d->data_len);
                    if (d->payload_offset + d->data_len == d->payload_len) {
                        on_frame_complete(s_rx_buf, d->payload_len);
                    }
                } else if (warn_ok()) {
                    ESP_LOGW(TAG, "dropping binary msg: payload_len=%d off=%d len=%d (max %d)",
                             d->payload_len, d->payload_offset, d->data_len, (int)RX_BUF_BYTES);
                }
            } else if (d->op_code == 0x1 /* text */) {
                /* Text messages need the same reassembly as frames. A JSON message that
                 * straddles a TCP segment boundary arrives in two callbacks, and a
                 * thin_telemetry with an ir_grid (~660 B) does that routinely — measured
                 * on hardware as "payload_len=655 off=0 len=307". The old code required
                 * offset==0 && data_len==payload_len and dropped everything else, which
                 * silently ate telemetry and, worse, could eat the single thin_hello_ack
                 * that decides whether this session runs v2 or falls back to v1. */
                if (d->payload_len > 0 && d->payload_len <= (int)TEXT_BUF_BYTES &&
                    d->payload_offset >= 0 && d->data_len >= 0 &&
                    d->payload_offset + d->data_len <= d->payload_len) {
                    memcpy(s_text_buf + d->payload_offset, d->data_ptr, d->data_len);
                    if (d->payload_offset + d->data_len == d->payload_len) {
                        on_text_message(s_text_buf, d->payload_len);
                    }
                } else if (warn_ok()) {
                    ESP_LOGW(TAG, "dropping text msg: payload_len=%d off=%d len=%d (max %d)",
                             d->payload_len, d->payload_offset, d->data_len, (int)TEXT_BUF_BYTES);
                }
            }
            break;
        }
        default:
            break;
    }
}

/* ---- Background task ---------------------------------------------------------------
 * Owns the websocket handle end to end: resolve -> connect -> pump the outbound command
 * queue while connected -> tear down -> back off -> repeat. It never blocks
 * indefinitely, so a queued UI command is picked up within LIDAR_POLL_MS. */

#define LIDAR_POLL_MS 100

static uint32_t backoff_grow(uint32_t ms)
{
    uint32_t next = ms * 2;
    return next < RECONNECT_BACKOFF_MAX_MS ? next : RECONNECT_BACKOFF_MAX_MS;
}

/* Drain the outbound queue and send whatever is pending. Only ever called from
 * lidar_task, which is the sole owner of s_ws. */
static void pump_commands(void)
{
    lidar_cmd_t c;
    while (s_cmd_q && xQueueReceive(s_cmd_q, &c, 0) == pdTRUE) {
        char buf[128];
        switch (c.kind) {
            case LC_ORBIT:
                snprintf(buf, sizeof(buf),
                         "{\"type\":\"thin_orbit\",\"dyaw\":%.3f,\"dpitch\":%.3f,\"dzoom\":%.3f}",
                         c.u.orbit.dyaw, c.u.orbit.dpitch, c.u.orbit.dzoom);
                break;
            case LC_MODE:
                snprintf(buf, sizeof(buf), "{\"type\":\"thin_mode\",\"mode\":\"%s\"}",
                         mode_to_str(c.u.mode));
                break;
            case LC_RECORD:
                snprintf(buf, sizeof(buf), "{\"type\":\"thin_record\",\"on\":%s}",
                         c.u.on ? "true" : "false");
                break;
            case LC_REHELLO:
                /* Not a thin_* command with a body of its own — it just re-runs the
                 * handshake, which is how a settings change reaches a live session. */
                if (s_ws && esp_websocket_client_is_connected(s_ws)) (void)send_hello();
                continue;
            default:
                continue;
        }
        if (s_ws && esp_websocket_client_is_connected(s_ws)) {
            /* Return is bytes sent, or <0 on failure (including a TX-lock timeout —
             * see the CONFIG_ESP_WS_CLIENT_SEPARATE_TX_LOCK note in sdkconfig.defaults).
             * A silently-dropped command otherwise looks identical to "nothing happened"
             * from the UI, which is exactly what hid this bug on hardware. */
            int sent = esp_websocket_client_send_text(s_ws, buf, (int)strlen(buf), pdMS_TO_TICKS(200));
            if (sent < 0 && warn_ok()) {
                ESP_LOGW(TAG, "command send failed (kind=%d, ret=%d) -- dropped", (int)c.kind, sent);
            }
        }
    }
}

/* Discard anything queued while there is no link (best-effort commands go stale fast). */
static void flush_commands(void)
{
    if (s_cmd_q) xQueueReset(s_cmd_q);
}

/* Open the v2 handshake. The client speaks first, immediately after the upgrade: until
 * the server has this it is entitled to free-run v1 frames at us. Returns false if the
 * send failed, in which case the caller retries on the next poll — an unsent hello would
 * otherwise silently leave the session on the uncompressed, unthrottled v1 path. */
static bool send_hello(void)
{
    /* Optional quality override (NVS `lidar_q`). Omitted entirely when unset, so the
     * default really is "the rig decides" rather than "the client asked for the rig's
     * default" — those differ the moment the rig's own default changes. */
    uint32_t q = 0;
    char qfield[24] = "";
    if (prop_settings_get_u32(LIDAR_QUALITY_KEY, &q, 0) == ESP_OK &&
        q >= LIDAR_QUALITY_MIN && q <= LIDAR_QUALITY_MAX) {
        snprintf(qfield, sizeof(qfield), ",\"quality\":%u", (unsigned)q);
    }

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
                     "{\"type\":\"thin_hello\",\"proto\":2,\"client\":\"crowpanel-p4\","
                     "\"accept\":[\"jpeg\",\"rgb565\"],\"width\":%d,\"height\":%d,"
                     "\"credits\":%d,\"max_frame_bytes\":%d,\"ir_cells\":%d,\"fps\":%d%s}",
                     PROP_LIDAR_FRAME_W, PROP_LIDAR_FRAME_H,
                     LIDAR_CREDITS, MAX_JPEG_MSG_BYTES, PROP_LIDAR_IR_MAX_CELLS,
                     LIDAR_REQUEST_FPS, qfield);
    int sent = esp_websocket_client_send_text(s_ws, buf, n, pdMS_TO_TICKS(500));
    if (sent < 0) {
        if (warn_ok()) ESP_LOGW(TAG, "thin_hello send failed (ret=%d) — retrying", sent);
        return false;
    }
    ESP_LOGI(TAG, "thin_hello sent (proto 2, jpeg preferred, %d credits, %d fps%s)",
             LIDAR_CREDITS, LIDAR_REQUEST_FPS, qfield[0] ? qfield : ", rig-chosen quality");
    return true;
}

/* Hand back one send credit per frame the UI has finished with. This is the client half
 * of the flow control: the server sends only while it holds credit and DROPS rather than
 * queues at zero, so these grants are what keeps frames coming — and what stops them
 * piling into a multi-second TCP backlog when the panel cannot keep up. */
static void pump_ready(void)
{
    if (s_proto < 2) return;   /* v1 server: free-running, nothing to grant */
    if (!s_ws || !esp_websocket_client_is_connected(s_ws)) return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t target = s_ready_target;
    uint32_t acked  = s_ready_acked;
    xSemaphoreGive(s_lock);
    if (acked >= target) return;

    /* Credit saturates at the negotiated depth server-side, so grants beyond it are
     * no-ops. Jumping the backlog keeps a long unattended stretch — the panel closed
     * inside the idle grace window, say — from firing a burst of pointless sends. */
    if (target - acked > LIDAR_CREDITS) acked = target - LIDAR_CREDITS;

    while (acked < target) {
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "{\"type\":\"thin_ready\",\"seq\":%u}",
                         (unsigned)target);
        int sent = esp_websocket_client_send_text(s_ws, buf, n, pdMS_TO_TICKS(200));
        if (sent < 0) {
            /* Do NOT advance past a failed send. A lost thin_ready is a credit the
             * server never gets back, and on a 2-deep window two of them wedge the
             * stream permanently; leaving it unacked is what makes the next poll retry. */
            if (warn_ok()) ESP_LOGW(TAG, "thin_ready send failed (ret=%d) — retrying", sent);
            break;
        }
        acked++;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (acked > s_ready_acked) s_ready_acked = acked;
    xSemaphoreGive(s_lock);
}

static void lidar_task(void *arg)
{
    (void)arg;
    uint32_t backoff_ms = RECONNECT_BACKOFF_MIN_MS;
    char uri[96];
    bool have_uri = false;   /* uri[] holds a usable endpoint from a previous pass */

    for (;;) {
        /* Idle while the LIDAR panel is closed — no socket, no traffic. Waking on the
         * event bit rather than polling means opening the panel starts the connect
         * immediately instead of up to LIDAR_POLL_MS later. */
        while (!s_active) {
            set_link_state(PROP_LIDAR_LINK_SEARCHING);
            flush_commands();
            backoff_ms = RECONNECT_BACKOFF_MIN_MS;   /* a fresh open shouldn't inherit it */
            xEventGroupWaitBits(s_evt, LIDAR_EVT_ACTIVE, pdTRUE, pdFALSE, portMAX_DELAY);
        }

        set_link_state(PROP_LIDAR_LINK_SEARCHING);
        flush_commands();

        /* Discovery is cached across reconnects: mdns_query_ptr blocks for its full 3 s
         * timeout, which is a painful chunk of the panel's time-to-first-frame to pay on
         * every transient drop. A session that never completes a handshake clears the
         * cache below, so a rig that moved is still picked up. */
        if (s_uri_dirty) {
            s_uri_dirty = false;
            have_uri = false;   /* re-read NVS / re-resolve against the new setting */
        }
        if (!have_uri) {
            if (!uri_from_nvs(uri, sizeof(uri))) {
                if (!resolve_uri(uri, sizeof(uri))) {
                    ESP_LOGW(TAG, "mDNS: _roomscan._tcp not found, retrying in %u ms", (unsigned)backoff_ms);
                    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
                    backoff_ms = backoff_grow(backoff_ms);
                    continue;
                }
            }
            have_uri = true;
            ESP_LOGI(TAG, "resolved %s", uri);
        }

        esp_websocket_client_config_t cfg = {
            .uri = uri,
            .buffer_size = WS_BUFFER_SIZE,
            .network_timeout_ms = 8000,
        };
        s_ws = esp_websocket_client_init(&cfg);
        if (!s_ws) {
            ESP_LOGE(TAG, "esp_websocket_client_init failed");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = backoff_grow(backoff_ms);
            continue;
        }
        esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
        s_ws_connected = false;
        bool handshook = false;    /* this session ever reached CONNECTED */
        bool hello_sent = false;   /* thin_hello has left the building for this session */
        xEventGroupClearBits(s_evt, LIDAR_EVT_DISCONNECTED | LIDAR_EVT_CONNECTED | LIDAR_EVT_CREDIT);
        esp_websocket_client_start(s_ws);

        /* Bounded waits so queued UI commands get serviced promptly and the task can
         * never park forever on a session that ends without a DISCONNECTED event.
         * LIDAR_EVT_CREDIT wakes this the instant the UI grants a credit, instead of
         * leaving pump_ready() to find out up to LIDAR_POLL_MS late — see that bit's
         * comment for why that lag mattered on this link. */
        for (;;) {
            EventBits_t bits = xEventGroupWaitBits(s_evt,
                                                   LIDAR_EVT_DISCONNECTED | LIDAR_EVT_CONNECTED | LIDAR_EVT_CREDIT,
                                                   pdTRUE, pdFALSE,
                                                   pdMS_TO_TICKS(LIDAR_POLL_MS));
            if (s_ws_connected) {
                /* Only a real, completed handshake earns a backoff reset — resetting on
                 * every connect *attempt* turns the backoff into a flat retry. */
                backoff_ms = RECONNECT_BACKOFF_MIN_MS;
                handshook = true;
                if (!hello_sent) hello_sent = send_hello();
            }
            pump_commands();
            pump_ready();
            if (bits & LIDAR_EVT_DISCONNECTED) break;
            /* Panel closed: hold the session open for the grace window (cheap way to
             * survive a quick nav away-and-back) and then drop it so the radio is free. */
            if (!s_active && (now_ms() - s_inactive_since_ms) > IDLE_GRACE_MS) {
                ESP_LOGI(TAG, "LIDAR panel closed — dropping the stream");
                break;
            }
        }

        esp_websocket_client_stop(s_ws);
        esp_websocket_client_destroy(s_ws);
        s_ws = NULL;
        s_ws_connected = false;
        /* Negotiation is per-connection — the next server might be a v1 one. */
        s_proto = 1;
        s_encoding_jpeg = false;
        set_link_state(PROP_LIDAR_LINK_SEARCHING);
        /* Never handshook: the cached endpoint is the prime suspect — re-resolve. */
        if (!handshook) have_uri = false;

        if (!s_active) continue;   /* straight to the idle wait, no backoff sleep */

        ESP_LOGI(TAG, "reconnecting in %u ms", (unsigned)backoff_ms);
        vTaskDelay(pdMS_TO_TICKS(backoff_ms));
        backoff_ms = backoff_grow(backoff_ms);
    }
}

esp_err_t prop_lidar_init(void)
{
    esp_err_t err = ESP_ERR_NO_MEM;

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    s_evt = xEventGroupCreate();
    if (!s_evt) goto fail;

    s_cmd_q = xQueueCreate(LIDAR_CMD_QUEUE_DEPTH, sizeof(lidar_cmd_t));
    if (!s_cmd_q) goto fail;

    /* The frame buffers double as the JPEG engine's DMA output, which requires the
     * ADDRESS and the size passed to jpeg_decoder_process to be cache-line aligned —
     * a plain heap_caps_malloc trips the driver's alignment check. jpeg_alloc_decoder_mem
     * rounds both to whatever esp_cache reports (already-zeroed PSRAM), and it is a plain
     * heap allocation, so the raw path and free() below are unaffected. */
    const jpeg_decode_memory_alloc_cfg_t out_mem = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    for (int i = 0; i < 3; i++) {
        size_t got = 0;
        s_frame_buf[i] = jpeg_alloc_decoder_mem(FRAME_BYTES, &out_mem, &got);
        if (!s_frame_buf[i]) {
            ESP_LOGE(TAG, "PSRAM alloc failed for frame buffer %d (%u bytes)", i, (unsigned)FRAME_BYTES);
            goto fail;
        }
        s_frame_buf_bytes = got;   /* identical for all three — same size, same alignment */
    }

    /* The JPEG engine reads its bitstream straight out of the reassembly buffer; input
     * has no alignment requirement (the driver syncs it unaligned), so a plain PSRAM
     * allocation is all this needs. */
    s_rx_buf = heap_caps_malloc(RX_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_rx_buf) {
        ESP_LOGE(TAG, "PSRAM alloc failed for rx buffer");
        goto fail;
    }

    s_text_buf = heap_caps_malloc(TEXT_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_text_buf) {
        ESP_LOGE(TAG, "PSRAM alloc failed for text buffer");
        goto fail;
    }

    s_ir_cells = heap_caps_calloc(1, PROP_LIDAR_IR_MAX_CELLS, MALLOC_CAP_SPIRAM);
    if (!s_ir_cells) {
        ESP_LOGE(TAG, "PSRAM alloc failed for IR grid");
        goto fail;
    }
    s_ir_w = s_ir_h = 0;

    s_frame_front = 0;
    s_frame_seq = 0;
    s_last_frame_ms = 0;
    s_proto = 1;
    s_encoding_jpeg = false;
    s_ready_target = 0;
    s_ready_acked = 0;

    memset(&s_telemetry, 0, sizeof(s_telemetry));
    s_telemetry.link = PROP_LIDAR_LINK_SEARCHING;

    BaseType_t ok = xTaskCreatePinnedToCore(lidar_task, "prop_lidar", 6144, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        err = ESP_FAIL;
        goto fail;
    }
    return ESP_OK;

fail:
    /* Partial bring-up: release everything acquired so far rather than leaking it for
     * the lifetime of the boot (these are ~1 MB of PSRAM plus kernel objects). */
    if (s_ir_cells) { free(s_ir_cells); s_ir_cells = NULL; }
    if (s_text_buf) { free(s_text_buf); s_text_buf = NULL; }
    if (s_rx_buf) { free(s_rx_buf); s_rx_buf = NULL; }
    for (int i = 0; i < 3; i++) {
        if (s_frame_buf[i]) { free(s_frame_buf[i]); s_frame_buf[i] = NULL; }
    }
    if (s_cmd_q) { vQueueDelete(s_cmd_q); s_cmd_q = NULL; }
    if (s_evt) { vEventGroupDelete(s_evt); s_evt = NULL; }
    vSemaphoreDelete(s_lock);
    s_lock = NULL;
    return err;
}
