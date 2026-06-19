#ifndef _PROP_UI_H_
#define _PROP_UI_H_

/* prop_ui — cassette-futurism screen. Pure view of prop_engine state.
 *
 * Builds an amber-on-black retro readout: title bar, big status line, a sweeping
 * scanner bar, a channel/frequency readout, and a LINK indicator. Registers as a
 * prop_engine observer; updates run under the LVGL port lock. Owns no logic.
 *
 * Call AFTER display_init() (LVGL must be running). */

#include "esp_err.h"

esp_err_t prop_ui_init(void);

/* Navigate the UI to a named screen (thread-safe; takes the LVGL lock). Used by
 * the API {"cmd":"ui","screen":"..."} for remote testing + screenshots.
 * Names: "home", "menu", "wifi", "display", "audio", "leds", "about". */
void prop_ui_goto(const char *screen);

#endif /* _PROP_UI_H_ */
