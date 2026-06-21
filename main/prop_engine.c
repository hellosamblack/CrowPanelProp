/* prop_engine — scene state machine + animation task + observer fan-out. */
#include "prop_engine.h"
#include "prop_audio.h"
#include "bsp_io.h"
#include <string.h>
#include <strings.h>   /* strcasecmp */
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_random.h"

#define ENGINE_TAG "ENGINE"
#define MAX_OBSERVERS 4
#define ANIM_PERIOD_MS 50    /* animation tick = 20 fps; smooth sweep with LVGL headroom */
#define TICKS_PER_SEC (1000 / ANIM_PERIOD_MS)   /* 20 */

/* Waveform sweep: one fresh column per tick. At 20 fps that's a smooth recorder
 * trace (~8 s to fill PROP_WAVE_SAMPLES) — slow, dense, still noisy. */
#define WAVE_STEP  1
#define SCRAMBLE_EVERY 2     /* re-roll the channel readout ~10x/s (legible hunt) */
#define BOOT_TICKS (TICKS_PER_SEC * 5 / 2)   /* ~2.5 s power-on self-test */

/* Scene definitions: default status/channel text. LED behavior is computed per
 * tick in animate_scene() so it can blink/sweep. Keep order matched to enum. */
static const struct {
    const char *name;
    const char *status;
} scene_table[SCENE_COUNT] = {
    [SCENE_IDLE]            = { "IDLE",            "STANDBY"          },
    [SCENE_SCANNING]        = { "SCANNING",        "SCANNING..."      },
    [SCENE_SIGNAL_ACQUIRED] = { "SIGNAL_ACQUIRED", "SIGNAL DETECTED"  },
    [SCENE_COMMS]           = { "COMMS",           "CHANNEL OPEN"     },
    [SCENE_ALERT]           = { "ALERT",           "** ALERT **"      },
};

static prop_state_t s_state;
static SemaphoreHandle_t s_mutex;
static struct { prop_observer_cb_t cb; void *ctx; } s_observers[MAX_OBSERVERS];
static int s_observer_count;
static uint32_t s_led_override_mask;   /* bits the operator has forced on */
static uint32_t s_led_override_valid;  /* bits that are currently overridden */
static uint32_t s_wave_col;            /* free-running oscillator phase (per column) */
static uint32_t s_boot_remaining;      /* boot self-test ticks left (0 = done) */

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

/* ---- Waveform recorder trace ------------------------------------------------
 * The engine generates the trail the UI plots in the scanner track. Each scene
 * has its own "character": flat idle ripple, scanning noise grass, a frozen
 * eruption on acquisition, a modulated COMMS carrier, full ALERT jitter. Samples
 * are signed deflections in -100..100 (% of half-track-height). */

static inline int8_t clamp8(int v)
{
    if (v >  100) return  100;
    if (v < -100) return -100;
    return (int8_t)v;
}

/* Symmetric integer noise in [-amp, amp]. */
static int noise(int amp)
{
    return (int)(esp_random() % (uint32_t)(2 * amp + 1)) - amp;
}

/* Scale a raw deflection by receiver sensitivity: 0 -> 0.2x, 100 -> 1.6x. Low
 * gain shrinks the noise floor; high gain drives spikes into the rails. */
static int apply_gain(int v)
{
    int g = 20 + (int)s_state.sensitivity * 14 / 10;   /* gain percent */
    return v * g / 100;
}

/* One new column of trail for `scene` at oscillator phase `col`. */
static int8_t gen_sample(prop_scene_t scene, uint32_t col)
{
    float p = (float)col;
    switch (scene) {
        case SCENE_IDLE: {
            /* Near-flat baseline with a faint slow ripple; the very rare spike
             * keeps the device looking alive on camera (never dead-flat). */
            int v = (int)(7.0f * sinf(p * 0.13f));
            if (esp_random() % 90 == 0) {
                v += (esp_random() & 1 ? 1 : -1) * (28 + (int)(esp_random() % 28));
            }
            return clamp8(apply_gain(v));
        }
        case SCENE_SCANNING: {
            /* Low noise grass with the occasional small spike (a near-miss). */
            int v = noise(13);
            if (esp_random() % 22 == 0) {
                v += (esp_random() & 1 ? 1 : -1) * (32 + (int)(esp_random() % 30));
            }
            return clamp8(apply_gain(v));
        }
        case SCENE_COMMS: {
            /* Modulated carrier: a fast carrier under a slow rhythmic envelope,
             * reading as "transmission in progress". */
            float env = 0.35f + 0.65f * fabsf(sinf(p * 0.10f));
            int v = (int)(78.0f * env * sinf(p * 0.95f)) + noise(5);
            return clamp8(apply_gain(v));
        }
        case SCENE_ALERT:
            return clamp8(apply_gain(noise(96)));   /* whole trace jitters hard */
        case SCENE_SIGNAL_ACQUIRED:
        default:
            return 0;   /* frozen scene: new columns aren't generated */
    }
}

/* Boot self-test sweep: a confident high-amplitude trace ramping in. */
static int8_t gen_boot_sample(uint32_t col)
{
    return clamp8((int)(92.0f * sinf((float)col * 0.28f)));
}

/* Advance the write head, laying down `count` fresh columns. */
static void wave_emit(prop_scene_t scene, int count, bool boot)
{
    for (int k = 0; k < count; k++) {
        s_wave_col++;
        s_state.wave_head = (s_state.wave_head + 1) % PROP_WAVE_SAMPLES;
        s_state.wave[s_state.wave_head] =
            boot ? gen_boot_sample(s_wave_col) : gen_sample(scene, s_wave_col);
    }
}

/* Stamp a dramatic eruption at the current head (SIGNAL_ACQUIRED entry). The
 * sweep then freezes, holding this spike as a peak the UI can mark. */
static void wave_signal_spike(void)
{
    static const int shape[] = { 100, 82, 55, 28 };   /* center out */
    int h = s_state.wave_head;
    for (int d = 0; d < (int)(sizeof(shape) / sizeof(shape[0])); d++) {
        int up = (h + d) % PROP_WAVE_SAMPLES;
        int dn = (h - d + PROP_WAVE_SAMPLES) % PROP_WAVE_SAMPLES;
        s_state.wave[up] = (int8_t)shape[d];
        s_state.wave[dn] = (int8_t)shape[d];
    }
}

/* ---- Channel readout: scramble while hunting, snap on lock ------------------ */

/* Band spans 100..400 MHz; map a frequency to the 0..100 gauge marker position. */
static uint8_t freq_to_pos(int mhz)
{
    int pos = (mhz - 100) * 100 / 300;
    if (pos < 0)   pos = 0;
    if (pos > 100) pos = 100;
    return (uint8_t)pos;
}

/* Write a locked CH/frequency into the channel readout (call on acquisition). */
static void lock_channel(void)
{
    int ch = 1 + (int)(esp_random() % 99);
    int mhz = 118 + (int)(esp_random() % 280);     /* 118..397 MHz */
    int frac = (int)(esp_random() % 100);
    snprintf(s_state.channel, PROP_TEXT_MAX, "CH %02d / %d.%02d MHz", ch, mhz, frac);
    s_state.chan_pos = freq_to_pos(mhz);
}

/* Re-roll the channel readout so the digits hunt like a tuner chasing a signal. */
static void scramble_channel(void)
{
    int ch = (int)(esp_random() % 100);
    int mhz = 100 + (int)(esp_random() % 300);
    int frac = (int)(esp_random() % 100);
    snprintf(s_state.channel, PROP_TEXT_MAX, "CH %02d / %d.%02d MHz", ch, mhz, frac);
    s_state.chan_pos = freq_to_pos(mhz);
}

/* Boot self-test status line for the given ticks-remaining. In-world: the unit
 * waking, handshaking the Armada data hub, and verifying its archive. */
static const char *boot_status(uint32_t remaining)
{
    if (remaining > 18) return "INTERFACE WAKE // SELF-TEST";
    if (remaining > 10) return "ARMADA LINK // HANDSHAKE";
    if (remaining > 3)  return "ARCHIVE INTEGRITY // OK";
    return "STANDBY";
}

/* Compute the animated LED mask for the active scene at the current tick.
 * Discrete on/off LEDs, so "animation" = blink phases. Override bits win. */
static uint32_t scene_led_mask(prop_scene_t scene, uint32_t tick)
{
    uint32_t mask = 0;
    bool slow = (tick / (TICKS_PER_SEC / 2)) & 0x1;   /* ~0.5 s phase */
    bool fast = (tick / (TICKS_PER_SEC / 5)) & 0x1;   /* ~0.2 s phase */
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
    /* LEDs sit on an I2C expander — only drive the bus when the mask actually
     * changes, so high-rate publishes (20 fps + command bursts) don't flood it. */
    static uint32_t s_last_led_mask = 0xFFFFFFFF;
    if (s_state.led_mask != s_last_led_mask) {
        bsp_io_led_set_mask(s_state.led_mask);
        s_last_led_mask = s_state.led_mask;
    }
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
        s_state.scene_tick++;

        if (s_boot_remaining > 0) {
            /* Power-on choreography: sweep a self-test trace, cycle the status,
             * then hand control back to the (IDLE) scene. */
            s_boot_remaining--;
            wave_emit(s_state.scene, WAVE_STEP, true);
            strlcpy(s_state.status, boot_status(s_boot_remaining), PROP_TEXT_MAX);
            if (s_boot_remaining == 0) {
                s_state.booting = false;
                strlcpy(s_state.status, scene_table[s_state.scene].status, PROP_TEXT_MAX);
            }
        } else {
            /* SCANNING hunts the channel; SIGNAL_ACQUIRED freezes the trail. The
             * readout re-rolls ~10x/s (not every 30 fps frame) so it stays legible. */
            if (s_state.scene == SCENE_SCANNING && s_state.tick % SCRAMBLE_EVERY == 0) {
                scramble_channel();
            }
            wave_emit(s_state.scene,
                      s_state.scene == SCENE_SIGNAL_ACQUIRED ? 0 : WAVE_STEP, false);
        }

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
    s_state.sensitivity = 65;     /* default receiver gain */
    s_state.chan_pos = 50;        /* gauge marker parked mid-band */
    s_state.booting = true;       /* run the power-on self-test choreography */
    s_boot_remaining = BOOT_TICKS;

    BaseType_t ok = xTaskCreatePinnedToCore(animate_task, "prop_anim", 4096, NULL, 5, NULL, 0);
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
    prop_scene_t prev = s_state.scene;
    s_state.scene = scene;
    s_state.scene_tick = 0;     /* drives the UI's punch-in / settle timing */
    s_led_override_valid = 0;   /* a new scene clears manual LED overrides */
    strlcpy(s_state.status, scene_table[scene].status, PROP_TEXT_MAX);

    /* Channel/trace side-effects of entering a scene. */
    switch (scene) {
        case SCENE_IDLE:
            strlcpy(s_state.channel, "CH -- / --- MHz", PROP_TEXT_MAX);
            s_state.chan_pos = 50;   /* untuned: marker rests mid-band */
            break;
        case SCENE_SIGNAL_ACQUIRED:
            lock_channel();         /* snap the hunting digits to a locked value */
            wave_signal_spike();    /* erupt + freeze (sweep holds in animate) */
            break;
        default:
            break;                  /* SCANNING scrambles per tick; COMMS/ALERT hold */
    }

    s_state.led_mask = scene_led_mask(scene, s_state.tick);
    publish_locked();
    xSemaphoreGive(s_mutex);

    /* Scene-transition sting (only on a real change), off the engine lock. */
    if (scene != prev) {
        if (scene == SCENE_SIGNAL_ACQUIRED) {
            prop_audio_play(PA_SIGNAL);
        } else if (scene == SCENE_ALERT) {
            prop_audio_play(PA_ALERT);
        }
    }
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

esp_err_t prop_engine_set_sensitivity(uint8_t pct)
{
    if (pct > 100) {
        pct = 100;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state.sensitivity = pct;
    xSemaphoreGive(s_mutex);
    /* No synchronous publish: the 20 fps animate loop repaints the SENS meter on
     * its next tick. A dragged web slider fires bursts of these — rendering each
     * one would saturate LVGL and stall the panel. */
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
