/* prop_track — pedestrian dead-reckoning + world-frame spatial memory.
 * See prop_track.h for the public API, coordinate frame, and honest limits. */
#include "prop_track.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "prop_imu.h"
#include "prop_motion.h"
#include "prop_settings.h"

#define TAG "PROP_TRACK"

/* ---- Tuning ------------------------------------------------------------- */
#define POLL_MS        40       /* matches the IMU poll cadence (~25 Hz) */
#define STRIDE_M       0.75f    /* metres per step (fixed; see header limits) */
#define CRUMB_MIN_M    0.40f    /* drop a breadcrumb every this much travel */
#define CRUMB_CAP      512      /* breadcrumb ring-buffer length (PSRAM) */
#define MARK_FRESH_MS  600      /* radar frame must be newer than this to ingest */
#define MARK_MERGE_M   0.70f    /* targets within this of a mark refresh it */
#define MARK_TTL_MS    30000    /* drop marks not refreshed within this */

/* ---- Cached state (written by task, read by getters under s_lock) -------- */
static SemaphoreHandle_t   s_lock;
static bool                s_available;
static float               s_dir_phi;     /* board→world yaw offset, radians */

static prop_track_pose_t   s_pose;

/* Breadcrumb ring buffer (PSRAM). Oldest at (head - count), newest at head-1. */
static prop_track_crumb_t *s_crumbs;
static int                 s_crumb_head;  /* index where the next crumb is written */
static int                 s_crumb_count; /* 0..CRUMB_CAP */
static float               s_last_crumb_x, s_last_crumb_y;

/* Last-known radar marks. last_ms = esp_timer ms at last refresh. */
typedef struct { float x, y; uint32_t last_ms; uint8_t floor; bool used; } mark_slot_t;
static mark_slot_t         s_marks[PROP_TRACK_MAX_MARKS];

/* Step tracking. */
static uint32_t            s_last_steps;
static bool                s_have_steps;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ---- Heading source seam ------------------------------------------------ *
 * Phase 1: gyro-only yaw plus the calibrated board→world offset. Phase 1.5
 * makes ONLY this function fuse a magnetometer (gyro fast-path + mag absolute,
 * tilt-compensated by accel) — PDR/crumb/mark/render code never changes. */
static inline float track_heading_rad(const prop_imu_data_t *imu)
{
    return imu->yaw + s_dir_phi;
}

/* Push a breadcrumb at the current position (caller holds s_lock). */
static void crumb_push_locked(float x, float y)
{
    s_crumbs[s_crumb_head].x = x;
    s_crumbs[s_crumb_head].y = y;
    s_crumbs[s_crumb_head].floor = s_pose.floor;
    s_crumb_head = (s_crumb_head + 1) % CRUMB_CAP;
    if (s_crumb_count < CRUMB_CAP) {
        s_crumb_count++;
    }
    s_last_crumb_x = x;
    s_last_crumb_y = y;
}

/* Fold one radar target's world position into the mark set (caller holds s_lock). */
static void mark_ingest_locked(float wx, float wy, uint32_t t)
{
    /* Refresh the nearest existing mark if close enough. */
    int best = -1;
    float best_d2 = MARK_MERGE_M * MARK_MERGE_M;
    int free_slot = -1, oldest = -1;
    uint32_t oldest_ms = 0xFFFFFFFFu;
    for (int i = 0; i < PROP_TRACK_MAX_MARKS; i++) {
        if (!s_marks[i].used) { if (free_slot < 0) free_slot = i; continue; }
        float dx = s_marks[i].x - wx, dy = s_marks[i].y - wy;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
        if (s_marks[i].last_ms < oldest_ms) { oldest_ms = s_marks[i].last_ms; oldest = i; }
    }
    int slot = (best >= 0) ? best : (free_slot >= 0) ? free_slot : oldest;
    if (slot < 0) return;
    if (best >= 0) {
        /* nudge toward the new fix (light smoothing) */
        s_marks[slot].x += (wx - s_marks[slot].x) * 0.35f;
        s_marks[slot].y += (wy - s_marks[slot].y) * 0.35f;
    } else {
        s_marks[slot].x = wx;
        s_marks[slot].y = wy;
    }
    s_marks[slot].last_ms = t;
    s_marks[slot].floor = s_pose.floor;
    s_marks[slot].used = true;
}

static void track_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        prop_imu_data_t imu;
        prop_imu_get_data(&imu);
        if (!imu.online || !imu.valid) {
            continue;   /* IMU not producing yet — pose stays invalid */
        }

        float heading = track_heading_rad(&imu);
        float sth = sinf(heading), cth = cosf(heading);

        xSemaphoreTake(s_lock, portMAX_DELAY);

        /* First valid packet: seed the pose + origin crumb. */
        if (!s_pose.valid) {
            s_pose.x = s_pose.y = 0.0f;
            s_pose.valid = true;
            s_last_crumb_x = s_last_crumb_y = 0.0f;
            crumb_push_locked(0.0f, 0.0f);
        }
        s_pose.heading = heading;

        /* Step-based dead reckoning off the hardware pedometer. */
        if (!s_have_steps) {
            s_last_steps = imu.step_count;
            s_have_steps = true;
        }
        uint32_t steps = imu.step_count;
        int32_t dsteps = (int32_t)(steps - s_last_steps);
        if (dsteps < 0) { dsteps = 0; s_last_steps = steps; }  /* counter reset — resync, drop the delta */
        if (dsteps > 0) {
            s_last_steps = steps;
            float d = STRIDE_M * (float)dsteps;
            s_pose.x += d * sth;                      /* +x = East */
            s_pose.y += d * cth;                      /* +y = North */
            /* Drop crumbs along the travelled segment. */
            float ddx = s_pose.x - s_last_crumb_x, ddy = s_pose.y - s_last_crumb_y;
            if (ddx * ddx + ddy * ddy >= CRUMB_MIN_M * CRUMB_MIN_M) {
                crumb_push_locked(s_pose.x, s_pose.y);
            }
        }

        /* Ingest fresh radar targets as world marks; age out stale marks. */
        uint32_t t = now_ms();
        if (prop_motion_ms_since_frame() < MARK_FRESH_MS) {
            prop_motion_target_t tg[PROP_MOTION_MAX_TARGETS];
            int n = prop_motion_get_targets(tg, PROP_MOTION_MAX_TARGETS);
            for (int i = 0; i < n; i++) {
                float fwd = (float)tg[i].y_mm / 1000.0f;   /* +Y forward (m) */
                float rgt = (float)tg[i].x_mm / 1000.0f;   /* +X right   (m) */
                /* forward axis F=(sinθ,cosθ); right axis R=(cosθ,-sinθ) */
                float wx = s_pose.x + fwd * sth + rgt * cth;
                float wy = s_pose.y + fwd * cth - rgt * sth;
                mark_ingest_locked(wx, wy, t);
            }
        }
        for (int i = 0; i < PROP_TRACK_MAX_MARKS; i++) {
            if (s_marks[i].used && (uint32_t)(t - s_marks[i].last_ms) > MARK_TTL_MS) {
                s_marks[i].used = false;
            }
        }

        xSemaphoreGive(s_lock);
    }
}

esp_err_t prop_track_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }
    s_crumbs = heap_caps_malloc(sizeof(prop_track_crumb_t) * CRUMB_CAP, MALLOC_CAP_SPIRAM);
    if (!s_crumbs) {
        ESP_LOGE(TAG, "crumb buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }
    memset(s_crumbs, 0, sizeof(prop_track_crumb_t) * CRUMB_CAP);

    /* Travel-direction offset, signed milli-radians (matches prop_ui PK_DIRCAL). */
    uint32_t phi_mrad = 0;
    prop_settings_get_u32("dir_phi", &phi_mrad, 0);
    s_dir_phi = (float)(int32_t)phi_mrad / 1000.0f;

    s_available = prop_imu_available();

    BaseType_t ok = xTaskCreatePinnedToCore(track_task, "prop_track", 4096, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "dead-reckoning up (IMU %s, dir_phi %.3f rad)",
             s_available ? "online" : "absent", s_dir_phi);
    return ESP_OK;
}

bool prop_track_available(void)
{
    return s_available && prop_imu_available();
}

void prop_track_get_pose(prop_track_pose_t *out)
{
    if (!out) return;
    if (!s_lock) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_pose;
    xSemaphoreGive(s_lock);
}

int prop_track_get_crumbs(prop_track_crumb_t *out, int max)
{
    if (!out || max <= 0 || !s_lock) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = s_crumb_count < max ? s_crumb_count : max;
    /* Copy newest n, oldest → newest. Oldest of the full set is at head-count. */
    int start = (s_crumb_head - n + CRUMB_CAP) % CRUMB_CAP;
    for (int i = 0; i < n; i++) {
        out[i] = s_crumbs[(start + i) % CRUMB_CAP];
    }
    xSemaphoreGive(s_lock);
    return n;
}

int prop_track_get_marks(prop_track_mark_t *out, int max)
{
    if (!out || max <= 0 || !s_lock) return 0;
    uint32_t t = now_ms();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = 0;
    for (int i = 0; i < PROP_TRACK_MAX_MARKS && n < max; i++) {
        if (!s_marks[i].used) continue;
        out[n].x = s_marks[i].x;
        out[n].y = s_marks[i].y;
        out[n].age_ms = (uint32_t)(t - s_marks[i].last_ms);
        out[n].floor = s_marks[i].floor;
        n++;
    }
    xSemaphoreGive(s_lock);
    return n;
}

void prop_track_set_dir_phi(float phi_rad)
{
    if (!s_lock) { s_dir_phi = phi_rad; return; }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_dir_phi = phi_rad;
    xSemaphoreGive(s_lock);
}

void prop_track_reset(void)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_pose.x = s_pose.y = 0.0f;
    s_pose.valid = false;        /* re-seeds origin + crumb on next valid packet */
    s_crumb_head = s_crumb_count = 0;
    s_last_crumb_x = s_last_crumb_y = 0.0f;
    s_have_steps = false;
    memset(s_marks, 0, sizeof(s_marks));
    xSemaphoreGive(s_lock);
}
