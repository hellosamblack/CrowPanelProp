/* prop_lidar — thin-client render link to lidar-roomscanner's /ws-thin endpoint.
 * See prop_lidar.h for the public API and docs/superpowers/specs/
 * 2026-08-17-lidar-thin-client-crowpanel-design.md for the protocol contract. */
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
#include "prop_settings.h"

#define TAG "PROP_LIDAR"

#define FRAME_BYTES        (PROP_LIDAR_FRAME_PIXELS * 2)
#define STALE_MS           2000
#define RECONNECT_BACKOFF_MIN_MS 1000
#define RECONNECT_BACKOFF_MAX_MS 30000
#define RX_HEADER_BYTES    8   /* u32 tag + u16 w + u16 h */
#define RX_TOTAL_BYTES     (RX_HEADER_BYTES + FRAME_BYTES)

/* WS RX/TX buffer. A whole THIN_FRAME is ~460 KB, so it always arrives split across
 * several WEBSOCKET_EVENT_DATA callbacks; a bigger buffer just means fewer of them
 * (~15 instead of ~113 at 4 KB). The component malloc()s two of these, and with
 * CONFIG_SPIRAM_USE_MALLOC + CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096 an allocation
 * this large lands in PSRAM, so it does not eat the tight internal-RAM budget. */
#define WS_BUFFER_SIZE     32768

/* NVS key holding an optional "host:port" (or "host") override for the roomscanner.
 * When set, mDNS discovery is skipped entirely — the escape hatch for networks where
 * mDNS doesn't work. Nothing in firmware writes it today (see the design notes). */
#define LIDAR_HOST_KEY     "lidar_host"

/* ---- Cached state (written by the task, read by getters under s_lock) ---------- */
static uint8_t *s_rx_buf;      /* PSRAM, RX_HEADER_BYTES + FRAME_BYTES */
static SemaphoreHandle_t s_lock;

static uint16_t *s_frame_buf[2];   /* PSRAM double buffer, FRAME_BYTES each */
static int       s_frame_front;    /* index of the buffer readers should copy from */
static uint32_t  s_frame_seq;      /* increments each time a new frame lands */
static uint32_t  s_last_frame_ms;  /* esp_timer ms at the last complete frame */

static prop_lidar_telemetry_t s_telemetry;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ---- Public getters -------------------------------------------------------------- */

bool prop_lidar_get_frame(uint16_t *dst, uint32_t *out_seq)
{
    if (!s_lock || !dst) return false;
    bool ok = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_frame_seq > 0) {
        memcpy(dst, s_frame_buf[s_frame_front], FRAME_BYTES);
        if (out_seq) *out_seq = s_frame_seq;
        ok = true;
    }
    xSemaphoreGive(s_lock);
    return ok;
}

uint32_t prop_lidar_get_seq(void)
{
    if (!s_lock) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t seq = s_frame_seq;
    xSemaphoreGive(s_lock);
    return seq;
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
    if (out->link == PROP_LIDAR_LINK_OK && (now_ms() - s_last_frame_ms) > STALE_MS) {
        out->link = PROP_LIDAR_LINK_STALE;   /* computed live, not stored */
    }
    xSemaphoreGive(s_lock);
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

typedef enum { LC_ORBIT = 0, LC_MODE, LC_RECORD } lidar_cmd_kind_t;

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
static EventGroupHandle_t s_evt;
static volatile bool s_ws_connected;   /* set by the WS event handler, read by lidar_task */
#define LIDAR_EVT_DISCONNECTED (1 << 0)

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

static void on_frame_complete(const uint8_t *buf, int len)
{
    if (len != (int)(RX_HEADER_BYTES + FRAME_BYTES)) return;
    uint32_t tag; uint16_t w, h;
    memcpy(&tag, buf + 0, 4);
    memcpy(&w,   buf + 4, 2);
    memcpy(&h,   buf + 6, 2);
    if (tag != 1 || w != PROP_LIDAR_FRAME_W || h != PROP_LIDAR_FRAME_H) {
        ESP_LOGW(TAG, "bad THIN_FRAME header tag=%u w=%u h=%u", (unsigned)tag, w, h);
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int back = 1 - s_frame_front;
    memcpy(s_frame_buf[back], buf + RX_HEADER_BYTES, FRAME_BYTES);
    s_frame_front = back;
    s_frame_seq++;
    s_last_frame_ms = now_ms();
    xSemaphoreGive(s_lock);
}

static prop_lidar_mode_t mode_from_str(const char *s)
{
    if (!s) return PROP_LIDAR_MODE_POINT_CLOUD;
    if (strcmp(s, "slam") == 0) return PROP_LIDAR_MODE_SLAM;
    if (strcmp(s, "ir") == 0)   return PROP_LIDAR_MODE_IR;
    return PROP_LIDAR_MODE_POINT_CLOUD;
}

static void on_telemetry_json(const char *text, int len)
{
    cJSON *root = cJSON_ParseWithLength(text, len);
    if (!root) return;
    const cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(type) || strcmp(type->valuestring, "thin_telemetry") != 0) {
        cJSON_Delete(root);
        return;
    }
    prop_lidar_telemetry_t t = {0};
    t.link = PROP_LIDAR_LINK_OK;
    const cJSON *fps   = cJSON_GetObjectItem(root, "fps");
    const cJSON *pw    = cJSON_GetObjectItem(root, "power_mode");
    const cJSON *i3c   = cJSON_GetObjectItem(root, "i3c_airtime_pct");
    const cJSON *pts   = cJSON_GetObjectItem(root, "point_count");
    const cJSON *rec   = cJSON_GetObjectItem(root, "recording");
    const cJSON *mode  = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsNumber(fps)) t.fps = (float)fps->valuedouble;
    if (cJSON_IsString(pw))  snprintf(t.power_mode, sizeof(t.power_mode), "%s", pw->valuestring);
    if (cJSON_IsNumber(i3c)) t.i3c_airtime_pct = (float)i3c->valuedouble;
    if (cJSON_IsNumber(pts)) t.point_count = pts->valueint;
    if (cJSON_IsBool(rec))   t.recording = cJSON_IsTrue(rec);
    if (cJSON_IsString(mode)) t.mode = mode_from_str(mode->valuestring);
    cJSON_Delete(root);

    xSemaphoreTake(s_lock, portMAX_DELAY);
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
            set_link_state(PROP_LIDAR_LINK_OK);
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
                if (d->payload_len == (int)RX_TOTAL_BYTES &&
                    d->payload_offset >= 0 && d->data_len >= 0 &&
                    d->payload_offset + d->data_len <= (int)RX_TOTAL_BYTES) {
                    memcpy(s_rx_buf + d->payload_offset, d->data_ptr, d->data_len);
                    if (d->payload_offset + d->data_len == d->payload_len) {
                        on_frame_complete(s_rx_buf, d->payload_len);
                    }
                } else if (warn_ok()) {
                    ESP_LOGW(TAG, "dropping binary msg: payload_len=%d off=%d len=%d (expected %d)",
                             d->payload_len, d->payload_offset, d->data_len, (int)RX_TOTAL_BYTES);
                }
            } else if (d->op_code == 0x1 /* text */) {
                if (d->payload_offset == 0 && d->data_len == d->payload_len) {
                    on_telemetry_json(d->data_ptr, d->data_len);
                } else if (warn_ok()) {
                    ESP_LOGW(TAG, "dropping fragmented text msg: payload_len=%d off=%d len=%d",
                             d->payload_len, d->payload_offset, d->data_len);
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
            default:
                continue;
        }
        if (s_ws && esp_websocket_client_is_connected(s_ws)) {
            esp_websocket_client_send_text(s_ws, buf, (int)strlen(buf), pdMS_TO_TICKS(200));
        }
    }
}

/* Discard anything queued while there is no link (best-effort commands go stale fast). */
static void flush_commands(void)
{
    if (s_cmd_q) xQueueReset(s_cmd_q);
}

static void lidar_task(void *arg)
{
    (void)arg;
    uint32_t backoff_ms = RECONNECT_BACKOFF_MIN_MS;

    for (;;) {
        char uri[96];
        set_link_state(PROP_LIDAR_LINK_SEARCHING);
        flush_commands();

        if (!uri_from_nvs(uri, sizeof(uri))) {
            if (!resolve_uri(uri, sizeof(uri))) {
                ESP_LOGW(TAG, "mDNS: _roomscan._tcp not found, retrying in %u ms", (unsigned)backoff_ms);
                vTaskDelay(pdMS_TO_TICKS(backoff_ms));
                backoff_ms = backoff_grow(backoff_ms);
                continue;
            }
        }
        ESP_LOGI(TAG, "resolved %s", uri);

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
        xEventGroupClearBits(s_evt, LIDAR_EVT_DISCONNECTED);
        esp_websocket_client_start(s_ws);

        /* Bounded waits so queued UI commands get serviced promptly and the task can
         * never park forever on a session that ends without a DISCONNECTED event. */
        for (;;) {
            EventBits_t bits = xEventGroupWaitBits(s_evt, LIDAR_EVT_DISCONNECTED, pdTRUE, pdFALSE,
                                                   pdMS_TO_TICKS(LIDAR_POLL_MS));
            if (s_ws_connected) {
                /* Only a real, completed handshake earns a backoff reset — resetting on
                 * every connect *attempt* turns the backoff into a flat retry. */
                backoff_ms = RECONNECT_BACKOFF_MIN_MS;
            }
            pump_commands();
            if (bits & LIDAR_EVT_DISCONNECTED) break;
        }

        esp_websocket_client_stop(s_ws);
        esp_websocket_client_destroy(s_ws);
        s_ws = NULL;
        s_ws_connected = false;
        set_link_state(PROP_LIDAR_LINK_SEARCHING);

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

    for (int i = 0; i < 2; i++) {
        s_frame_buf[i] = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
        if (!s_frame_buf[i]) {
            ESP_LOGE(TAG, "PSRAM alloc failed for frame buffer %d (%u bytes)", i, (unsigned)FRAME_BYTES);
            goto fail;
        }
        memset(s_frame_buf[i], 0, FRAME_BYTES);
    }

    s_rx_buf = heap_caps_malloc(RX_TOTAL_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_rx_buf) {
        ESP_LOGE(TAG, "PSRAM alloc failed for rx buffer");
        goto fail;
    }

    s_frame_front = 0;
    s_frame_seq = 0;
    s_last_frame_ms = 0;

    memset(&s_telemetry, 0, sizeof(s_telemetry));
    s_telemetry.link = PROP_LIDAR_LINK_SEARCHING;

    BaseType_t ok = xTaskCreate(lidar_task, "prop_lidar", 6144, NULL, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        err = ESP_FAIL;
        goto fail;
    }
    return ESP_OK;

fail:
    /* Partial bring-up: release everything acquired so far rather than leaking it for
     * the lifetime of the boot (these are ~1 MB of PSRAM plus kernel objects). */
    if (s_rx_buf) { free(s_rx_buf); s_rx_buf = NULL; }
    for (int i = 0; i < 2; i++) {
        if (s_frame_buf[i]) { free(s_frame_buf[i]); s_frame_buf[i] = NULL; }
    }
    if (s_cmd_q) { vQueueDelete(s_cmd_q); s_cmd_q = NULL; }
    if (s_evt) { vEventGroupDelete(s_evt); s_evt = NULL; }
    vSemaphoreDelete(s_lock);
    s_lock = NULL;
    return err;
}
