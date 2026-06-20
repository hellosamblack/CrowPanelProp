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

/* Per-effect strength, 0..100 (0 = effect off). Each re-bakes / re-opacities the
 * overlay live and persists to NVS. Safe to call from any task (locks internally). */
void prop_fx_set_scanlines(uint8_t pct);
uint8_t prop_fx_scanlines(void);
void prop_fx_set_phosphor(uint8_t pct);
uint8_t prop_fx_phosphor(void);
void prop_fx_set_vignette(uint8_t pct);
uint8_t prop_fx_vignette(void);
void prop_fx_set_refresh(uint8_t pct);
uint8_t prop_fx_refresh(void);

/* Play the configured screen-change transition ("old TV" channel change). Call
 * right after a panel swap, while holding the LVGL lock. Flavor + on/off come from
 * the "fx_trans" setting (0 off, 1 snow, 2 roll, 3 collapse, 4 snow+collapse). */
void prop_fx_transition_play(void);

#endif /* _PROP_FX_H_ */
