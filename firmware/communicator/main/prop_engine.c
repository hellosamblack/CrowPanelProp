/* prop_engine — scene state machine + animation task + observer fan-out. */
#include "prop_engine.h"
#include "bsp_io.h"
#include <string.h>
#include <strings.h>   /* strcasecmp */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define ENGINE_TAG "ENGINE"
#define MAX_OBSERVERS 4
#define ANIM_PERIOD_MS 100   /* animation tick = 10 fps; smooth enough for LED blinks */

/* Scene definitions: default status/channel text. LED behavior is computed per
 * tick in animate_scene() so it can blink/sweep. Keep order matched to enum. */
static const struct {
    const char *name;
    const char *status;
} scene_table[SCENE_COUNT] = {
    [SCENE_IDLE]            = { "IDLE",            "STANDBY"          },
    [SCENE_SCANNING]        = { "SCANNING",        "SCANNING..."      },
    [SCENE_SIGNAL_ACQUIRED] = { "SIGNAL_ACQUIRED", "SIGNAL ACQUIRED"  },
    [SCENE_COMMS]           = { "COMMS",           "CHANNEL OPEN"     },
    [SCENE_ALERT]           = { "ALERT",           "** ALERT **"      },
};

static prop_state_t s_state;
static SemaphoreHandle_t s_mutex;
static struct { prop_observer_cb_t cb; void *ctx; } s_observers[MAX_OBSERVERS];
static int s_observer_count;
static uint32_t s_led_override_mask;   /* bits the operator has forced on */
static uint32_t s_led_override_valid;  /* bits that are currently overridden */

const char *prop_scene_name(prop_scene_t scene)
{
    return (scene < SCENE_COUNT) ? scene_table[scene].name : "?";
}

prop_scene_t prop_scene_from_name(const char *name)
{
    if (name) {
        for (int i = 0; i < SCENE_COUNT; i++) {
            if (strcasecmp(name, scene_table[i].name) == 0) {
                return (prop_scene_t)i;
            }
        }
    }
    return SCENE_COUNT;
}

/* Compute the animated LED mask for the active scene at the current tick.
 * Discrete on/off LEDs, so "animation" = blink phases. Override bits win. */
static uint32_t scene_led_mask(prop_scene_t scene, uint32_t tick)
{
    uint32_t mask = 0;
    bool slow = (tick / 5) & 0x1;   /* ~0.5 s phase */
    bool fast = (tick / 2) & 0x1;   /* ~0.2 s phase */
    switch (scene) {
        case SCENE_IDLE:
            mask = slow ? (1u << LED_POWER) : 0;            /* slow heartbeat */
            break;
        case SCENE_SCANNING:
            mask = (1u << LED_POWER) | (fast ? (1u << LED_SIGNAL) : 0); /* searching blink */
            break;
        case SCENE_SIGNAL_ACQUIRED:
            mask = (1u << LED_POWER) | (1u << LED_SIGNAL);  /* steady lock */
            break;
        case SCENE_COMMS:
            mask = (1u << LED_POWER) | (slow ? (1u << LED_SIGNAL) : 0); /* traffic */
            break;
        case SCENE_ALERT:
            mask = fast ? ((1u << LED_POWER) | (1u << LED_ALERT)) : 0;  /* fast flash all */
            break;
        default:
            break;
    }
    /* Apply manual overrides. */
    mask = (mask & ~s_led_override_valid) | (s_led_override_mask & s_led_override_valid);
    return mask;
}

/* Push current state to LEDs + notify observers. Call with mutex held. */
static void publish_locked(void)
{
    bsp_io_led_set_mask(s_state.led_mask);
    /* Snapshot under lock, notify outside lock to avoid re-entrancy deadlock. */
    prop_state_t snapshot = s_state;
    for (int i = 0; i < s_observer_count; i++) {
        prop_observer_cb_t cb = s_observers[i].cb;
        void *ctx = s_observers[i].ctx;
        xSemaphoreGive(s_mutex);
        cb(&snapshot, ctx);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
}

static void animate_task(void *arg)
{
    (void)arg;
    while (1) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_state.tick++;
        s_state.led_mask = scene_led_mask(s_state.scene, s_state.tick);
        publish_locked();
        xSemaphoreGive(s_mutex);
        vTaskDelay(pdMS_TO_TICKS(ANIM_PERIOD_MS));
    }
}

esp_err_t prop_engine_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_state, 0, sizeof(s_state));
    s_state.scene = SCENE_IDLE;
    s_state.link = LINK_DOWN;
    strlcpy(s_state.status, scene_table[SCENE_IDLE].status, PROP_TEXT_MAX);
    strlcpy(s_state.channel, "CH -- / --- MHz", PROP_TEXT_MAX);

    BaseType_t ok = xTaskCreate(animate_task, "prop_anim", 4096, NULL, 5, NULL);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(ENGINE_TAG, "engine started, scene=%s", prop_scene_name(s_state.scene));
    return ESP_OK;
}

esp_err_t prop_engine_add_observer(prop_observer_cb_t cb, void *ctx)
{
    if (!cb) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ESP_ERR_NO_MEM;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_observer_count < MAX_OBSERVERS) {
        s_observers[s_observer_count].cb = cb;
        s_observers[s_observer_count].ctx = ctx;
        s_observer_count++;
        err = ESP_OK;
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t prop_engine_set_scene(prop_scene_t scene)
{
    if (scene >= SCENE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state.scene = scene;
    s_led_override_valid = 0;   /* a new scene clears manual LED overrides */
    strlcpy(s_state.status, scene_table[scene].status, PROP_TEXT_MAX);
    s_state.led_mask = scene_led_mask(scene, s_state.tick);
    publish_locked();
    xSemaphoreGive(s_mutex);
    ESP_LOGI(ENGINE_TAG, "scene -> %s", prop_scene_name(scene));
    return ESP_OK;
}

esp_err_t prop_engine_next_scene(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    prop_scene_t next = (prop_scene_t)((s_state.scene + 1) % SCENE_COUNT);
    xSemaphoreGive(s_mutex);
    return prop_engine_set_scene(next);
}

esp_err_t prop_engine_set_link(prop_link_t link)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state.link = link;
    publish_locked();
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t prop_engine_set_status(const char *text)
{
    if (!text) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    strlcpy(s_state.status, text, PROP_TEXT_MAX);
    publish_locked();
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t prop_engine_set_channel(const char *text)
{
    if (!text) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    strlcpy(s_state.channel, text, PROP_TEXT_MAX);
    publish_locked();
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t prop_engine_set_led(uint32_t led_index, bool on)
{
    if (led_index >= LED_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_led_override_valid |= (1u << led_index);
    if (on) {
        s_led_override_mask |= (1u << led_index);
    } else {
        s_led_override_mask &= ~(1u << led_index);
    }
    s_state.led_mask = scene_led_mask(s_state.scene, s_state.tick);
    publish_locked();
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void prop_engine_get_state(prop_state_t *out)
{
    if (!out) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_mutex);
}
