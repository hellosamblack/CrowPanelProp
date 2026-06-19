#ifndef _PROP_FX_H_
#define _PROP_FX_H_

/* prop_fx — cassette-CRT post overlay on the LVGL top layer (above every screen).
 *
 * Cheap, overlay-based effects: a phosphor amber wash, scanlines, a faint grid,
 * and a slow rolling "refresh" bar. It is a GLOBAL layer independent of which
 * screen is shown, with a runtime toggle and tunable intensity.
 *
 * OFF BY DEFAULT — the overlay can hurt legibility on a camera-representative
 * capture, so it is opt-in (DISPLAY settings panel / `{"cmd":"fx",...}`). When
 * disabled the overlay objects are hidden and cost nothing to render.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Build the (hidden) overlay on lv_layer_top(). Call after the UI is built.
 * Takes the LVGL lock internally — do NOT call while already holding it. */
esp_err_t prop_fx_init(void);

void prop_fx_set_enabled(bool on);
bool prop_fx_enabled(void);

/* Overall overlay strength, 0..100 (maps to layer opacity). */
void prop_fx_set_intensity(uint8_t pct);
uint8_t prop_fx_intensity(void);

#endif /* _PROP_FX_H_ */
