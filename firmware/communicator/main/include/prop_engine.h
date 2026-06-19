#ifndef _PROP_ENGINE_H_
#define _PROP_ENGINE_H_

/* prop_engine — the single source of truth for prop behavior.
 *
 * Holds the current scene + derived output state (LEDs, on-screen text). A
 * background task animates the active scene (LED blinks/sweeps, ticking counters)
 * and notifies observers on every change. Inputs from anywhere — physical buttons
 * (bsp_io) and remote commands (prop_api) — are funneled through the set_* calls
 * so LEDs and the screen never drift out of sync.
 *
 * Observers (prop_ui, prop_api) register a callback and receive a snapshot of the
 * state whenever it changes. Callbacks run in the engine task context; keep them
 * quick and re-entrant (the UI marshals onto the LVGL task; the API broadcasts).
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_TEXT_MAX 40

typedef enum {
    SCENE_IDLE = 0,
    SCENE_SCANNING,
    SCENE_SIGNAL_ACQUIRED,
    SCENE_COMMS,
    SCENE_ALERT,
    SCENE_COUNT,
} prop_scene_t;

typedef enum {
    LINK_DOWN = 0,   /* no network */
    LINK_AP,         /* only our own hotspot up */
    LINK_STA,        /* joined an upstream network */
} prop_link_t;

/* Immutable snapshot handed to observers. */
typedef struct {
    prop_scene_t scene;
    uint32_t led_mask;                 /* bit i => LED i should be on right now */
    char status[PROP_TEXT_MAX];        /* big status line, e.g. "SCANNING..." */
    char channel[PROP_TEXT_MAX];       /* secondary readout, e.g. "CH 04 / 147.55 MHz" */
    prop_link_t link;
    uint32_t tick;                     /* monotonic animation frame counter */
} prop_state_t;

typedef void (*prop_observer_cb_t)(const prop_state_t *state, void *ctx);

/* Start the engine + animation task. Call after bsp_io is initialized. */
esp_err_t prop_engine_init(void);

/* Register an observer. Returns ESP_ERR_NO_MEM if the (small) table is full. */
esp_err_t prop_engine_add_observer(prop_observer_cb_t cb, void *ctx);

/* Commands — all thread-safe; each triggers an observer notification. */
esp_err_t prop_engine_set_scene(prop_scene_t scene);
esp_err_t prop_engine_set_link(prop_link_t link);
esp_err_t prop_engine_set_status(const char *text);
esp_err_t prop_engine_set_channel(const char *text);
/* Manual LED override (e.g. operator forcing a lamp); persists until the scene
 * animation next drives that LED. */
esp_err_t prop_engine_set_led(uint32_t led_index, bool on);

/* Cycle to the next scene — handy default for a physical "mode" button. */
esp_err_t prop_engine_next_scene(void);

/* Copy the current state out. */
void prop_engine_get_state(prop_state_t *out);

/* Scene name <-> enum (for the JSON API). Returns SCENE_COUNT if unknown. */
const char *prop_scene_name(prop_scene_t scene);
prop_scene_t prop_scene_from_name(const char *name);

#endif /* _PROP_ENGINE_H_ */
