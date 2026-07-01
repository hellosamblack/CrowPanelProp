#ifndef _PROP_UI_H_
#define _PROP_UI_H_

/* prop_ui — cassette-futurism screen. Pure view of prop_engine state.
 *
 * Builds an amber-on-black retro readout: title bar, big status line, a sweeping
 * scanner bar, a channel/frequency readout, and a LINK indicator. Registers as a
 * prop_engine observer; updates run under the LVGL port lock. Owns no logic.
 *
 * Call AFTER display_init() (LVGL must be running). */

#include <stdbool.h>
#include "esp_err.h"

esp_err_t prop_ui_init(void);

/* Navigate the UI to a named screen (thread-safe; takes the LVGL lock). Used by
 * the API {"cmd":"ui","screen":"..."} for remote testing + screenshots. Names:
 * "home" (console), "scanner", "archive", "cassette", "insights", "menu", "wifi",
 * "display", "audio", "leds", "about", "vitals", "scan", "spectrum", "rfband",
 * "ble", "csi", "csicfg", "csiset", "instruments", "sensors", "motion", "dircal",
 * "minimap", "range", "io", "io<N>". */
void prop_ui_goto(const char *screen);

/* Physical-control input, decoupled from hardware (web portal drives it now; the
 * real knobs/switches route here later). Thread-safe; takes the LVGL lock.
 *   control = "selector": arg >0 = rotate CW, <0 = CCW, 0 = press
 *   control = "tab":      arg = archive section index
 *   control = "action":   arg 1 = primary/select, 2 = back to console */
void prop_ui_input(const char *control, int arg);

/* Show/hide the FPS meter HUD (top-right). Persisted to NVS ("fps_on").
 * Thread-safe; takes the LVGL lock. */
void prop_ui_set_fps(bool on);

/* Name of the panel currently on screen, in the same vocabulary as
 * prop_ui_goto()'s `screen` names (e.g. "scanner", "vitals", "spectrum").
 * For telemetry/diagnostics. Cheap enum read; no lock needed. */
const char *prop_ui_current_screen(void);

#endif /* _PROP_UI_H_ */
