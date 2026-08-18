/* prop_lidar — thin-client render link to lidar-roomscanner's /ws-thin endpoint.
 * See prop_lidar.h for the public API and docs/superpowers/specs/
 * 2026-08-17-lidar-thin-client-crowpanel-design.md for the protocol contract. */
#include "prop_lidar.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"

#define TAG "PROP_LIDAR"

#define FRAME_BYTES        (PROP_LIDAR_FRAME_PIXELS * 2)
#define STALE_MS           2000
#define RECONNECT_BACKOFF_MIN_MS 1000
#define RECONNECT_BACKOFF_MAX_MS 30000

/* ---- Cached state (written by the task, read by getters under s_lock) ---------- */
static SemaphoreHandle_t s_lock;

static uint16_t *s_frame_buf[2];   /* PSRAM double buffer, FRAME_BYTES each */
static int       s_frame_front;    /* index of the buffer readers should copy from */
static uint32_t  s_frame_seq;      /* increments each time a new frame lands */
static uint32_t  s_last_frame_ms;  /* esp_timer ms at the last complete frame */

static prop_lidar_telemetry_t s_telemetry;
static bool s_have_telemetry;

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

/* ---- Outbound commands (filled in by Task 6) -------------------------------------- */

void prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom)
{
    (void)dyaw; (void)dpitch; (void)dzoom;
    /* implemented in Task 6 */
}

void prop_lidar_send_mode(prop_lidar_mode_t mode)
{
    (void)mode;
    /* implemented in Task 6 */
}

void prop_lidar_send_record(bool on)
{
    (void)on;
    /* implemented in Task 6 */
}

/* ---- Background task (connect/reconnect logic filled in by Task 4) ---------------- */

static void lidar_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "task started (mDNS/WS connect logic pending — Task 4)");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_BACKOFF_MAX_MS));
    }
}

esp_err_t prop_lidar_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    for (int i = 0; i < 2; i++) {
        s_frame_buf[i] = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
        if (!s_frame_buf[i]) {
            ESP_LOGE(TAG, "PSRAM alloc failed for frame buffer %d (%u bytes)", i, (unsigned)FRAME_BYTES);
            return ESP_ERR_NO_MEM;
        }
        memset(s_frame_buf[i], 0, FRAME_BYTES);
    }
    s_frame_front = 0;
    s_frame_seq = 0;
    s_last_frame_ms = 0;

    memset(&s_telemetry, 0, sizeof(s_telemetry));
    s_telemetry.link = PROP_LIDAR_LINK_SEARCHING;
    s_have_telemetry = false;

    BaseType_t ok = xTaskCreate(lidar_task, "prop_lidar", 6144, NULL, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}
