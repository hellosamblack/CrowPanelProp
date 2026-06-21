/* prop_kit — the cassette-futurism design system for the communicator UI.
 *
 * Single source of truth for the palette + fonts, plus reusable themed components so
 * screens are *composed* from building blocks instead of hand-positioned lv_obj calls.
 * Built on LVGL 9: flex/grid layout (no manual y-offset bookkeeping) and the complex
 * SW-draw gradient path. Keep the look consistent by reaching for these first.
 *
 * House rules (see communicator-ui skill): square corners, amber-on-near-black, Eurostile,
 * and camera legibility — never COL_DIM for text the viewer must read (use COL_MUTE + bold).
 */
#pragma once
#include "lvgl.h"

/* ---- Palette (amber phosphor on near-black) ----------------------------- */
#define COL_BG         lv_color_hex(0x0A0A06)   /* screen background          */
#define COL_AMBER      lv_color_hex(0xE0B000)   /* primary text / active      */
#define COL_MUTE       lv_color_hex(0xB58A00)   /* secondary text, on-camera  */
#define COL_DIM        lv_color_hex(0x6B5300)   /* unlit / de-emphasised only */
#define COL_ALERT      lv_color_hex(0xFF3030)   /* alarms                     */
#define COL_PANEL_ITEM lv_color_hex(0x141008)   /* input/field/card fill      */

/* ---- Fonts (Eurostile; FontAwesome symbols merged into 14/24) ----------- */
LV_FONT_DECLARE(eurostile_14);
LV_FONT_DECLARE(eurostile_24);
LV_FONT_DECLARE(eurostile_40);   /* resting status headline   */
LV_FONT_DECLARE(eurostile_56);   /* status punch-in           */
#define FONT_BODY   (&eurostile_14)
#define FONT_HEAD   (&eurostile_24)
#define FONT_STATUS (&eurostile_40)
#define FONT_PUNCH  (&eurostile_56)

/* ---- Components (LVGL 9 flex + gradients) -------------------------------- */

/* A themed card: square, dim border on COL_PANEL_ITEM, set up as a vertical flex
 * column with consistent padding + row gap. Children added afterwards stack
 * top-to-bottom automatically — no manual y math. */
lv_obj_t *kit_card(lv_obj_t *parent, lv_coord_t w, lv_coord_t h);

/* A key/value row for a flex column (e.g. a kit_card): muted key on the left, amber
 * value on the right, space-between. Returns the VALUE label so callers can update it
 * live (e.g. from the observer). Width defaults to 100% of the parent. */
lv_obj_t *kit_info_row(lv_obj_t *parent, const char *key, const char *val);

/* Paint a 2-stop phosphor gradient (c1 -> c2 along `dir`) as an object's background.
 * Uses the v9 complex SW-draw gradient path (CONFIG_LV_DRAW_SW_COMPLEX). */
void kit_phosphor_grad(lv_obj_t *obj, lv_color_t c1, lv_color_t c2, lv_grad_dir_t dir);

/* ---- Styling primitives (theme a raw widget; the kit is the single source) --- */
void kit_style_btn(lv_obj_t *o);      /* amber-bordered button on the panel fill   */
void kit_style_field(lv_obj_t *o);    /* dropdown/textarea: dim border, amber text  */
void kit_style_slider(lv_obj_t *o);   /* square amber slider                        */
void kit_style_switch(lv_obj_t *o);   /* square amber switch                        */

/* ---- Meters ------------------------------------------------------------- */
lv_obj_t *kit_meter(lv_obj_t *parent, lv_coord_t w);   /* square track + amber fill; returns the fill */
void kit_set_meter(lv_obj_t *fill, int pct, lv_color_t col);

/* ---- Flex page + rows (compose a panel body with no manual y offsets) ---- */
/* Transparent flex-column body filling the panel below the title; rows self-stack
 * with a consistent gap (vertically scrollable if they overflow). */
lv_obj_t *kit_body(lv_obj_t *panel);
/* "LABEL ........ VAL%" over a slider. Returns the value label — store it and update
 * it from `cb` (which reads the slider via lv_event_get_target). */
lv_obj_t *kit_slider_row(lv_obj_t *body, const char *label, int min, int max, int val, lv_event_cb_t cb);
/* "LABEL .......... [switch]". Returns the switch. */
lv_obj_t *kit_switch_row(lv_obj_t *body, const char *label, bool on, lv_event_cb_t cb, void *ud);
/* "LABEL ........ VALUE" over a meter (pass fill_out=NULL for a value-only row).
 * Returns the value label; *fill_out (if given) receives the fill for kit_set_meter. */
lv_obj_t *kit_meter_row(lv_obj_t *body, const char *label, lv_obj_t **fill_out);
/* Full-width menu/list row (big amber text, left-aligned). Returns the row. */
lv_obj_t *kit_list_row(lv_obj_t *body, const char *text, lv_event_cb_t cb, void *ud);
/* Bare full-width flex row (space-between) — fill it yourself with a label + a widget
 * (e.g. a dropdown). Returns the row container. */
lv_obj_t *kit_row(lv_obj_t *body);
