/* prop_kit — reusable cassette-futurism components. See prop_kit.h. */
#include "prop_kit.h"
#include "prop_audio.h"
#include "esp_timer.h"
#include <stdio.h>

/* Interaction sound feedback. Attached centrally here so every themed widget sounds the
 * same across all screens (prop_audio_play is a no-op when audio is unavailable). */
static void kit_btn_sfx_cb(lv_event_t *e)    { (void)e; prop_audio_play(PA_BUTTON); }
static void kit_slider_sfx_cb(lv_event_t *e)
{
    /* Dragging fires many VALUE_CHANGED events; throttle to a ~35 ms ratchet so it
     * ticks instead of buzzing. One slider is dragged at a time, so a single gate is fine. */
    static int64_t last_us;
    int64_t now = esp_timer_get_time();
    if (now - last_us < 35000) {
        return;
    }
    last_us = now;
    (void)e;
    prop_audio_play(PA_SLIDER);
}

lv_obj_t *kit_card(lv_obj_t *parent, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_set_size(c, w, h);
    /* Subtle vertical phosphor gradient (panel fill -> near-black) for CRT-tube depth —
     * exercises the v9 complex SW-draw gradient path; the two tones are close so it never
     * hurts legibility. */
    kit_phosphor_grad(c, COL_PANEL_ITEM, COL_BG, LV_GRAD_DIR_VER);
    lv_obj_set_style_border_color(c, COL_DIM, 0);
    lv_obj_set_style_border_width(c, 1, 0);
    lv_obj_set_style_radius(c, 0, 0);          /* square corners — house style */
    lv_obj_set_style_pad_all(c, 14, 0);
    lv_obj_set_style_pad_row(c, 10, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(c, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(c, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    return c;
}

lv_obj_t *kit_info_row(lv_obj_t *parent, const char *key, const char *val)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *k = lv_label_create(row);
    lv_label_set_text(k, key);
    lv_obj_set_style_text_color(k, COL_MUTE, 0);

    lv_obj_t *v = lv_label_create(row);
    lv_label_set_text(v, val);
    lv_obj_set_style_text_color(v, COL_AMBER, 0);
    return v;
}

void kit_phosphor_grad(lv_obj_t *obj, lv_color_t c1, lv_color_t c2, lv_grad_dir_t dir)
{
    lv_obj_set_style_bg_color(obj, c1, 0);
    lv_obj_set_style_bg_grad_color(obj, c2, 0);
    lv_obj_set_style_bg_grad_dir(obj, dir, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
}

/* ---- Styling primitives ------------------------------------------------- */
void kit_style_btn(lv_obj_t *b)
{
    lv_obj_set_style_bg_color(b, COL_PANEL_ITEM, 0);
    lv_obj_set_style_bg_color(b, COL_DIM, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_color(b, COL_AMBER, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_radius(b, 0, 0);
    lv_obj_set_style_text_color(b, COL_AMBER, 0);
    /* Every themed button clicks when pressed (one callback per styled button). */
    lv_obj_add_event_cb(b, kit_btn_sfx_cb, LV_EVENT_CLICKED, NULL);
}
void kit_style_field(lv_obj_t *f)
{
    lv_obj_set_style_bg_color(f, COL_PANEL_ITEM, 0);
    lv_obj_set_style_border_color(f, COL_DIM, 0);
    lv_obj_set_style_border_width(f, 1, 0);
    lv_obj_set_style_radius(f, 0, 0);
    lv_obj_set_style_text_color(f, COL_AMBER, 0);
    /* Dropdowns click when tapped open (text areas stay silent — tapping one opens a
     * keyboard, a different interaction). */
    if (lv_obj_check_type(f, &lv_dropdown_class)) {
        lv_obj_add_event_cb(f, kit_btn_sfx_cb, LV_EVENT_CLICKED, NULL);
    }
}
void kit_style_slider(lv_obj_t *s)
{
    lv_obj_set_style_bg_color(s, COL_PANEL_ITEM, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(s, COL_DIM, LV_PART_MAIN);
    lv_obj_set_style_border_width(s, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(s, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, COL_AMBER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, COL_AMBER, LV_PART_KNOB);
    lv_obj_set_style_radius(s, 0, LV_PART_KNOB);
}
void kit_style_switch(lv_obj_t *sw)
{
    lv_obj_set_style_bg_color(sw, COL_PANEL_ITEM, LV_PART_MAIN);
    lv_obj_set_style_border_color(sw, COL_DIM, LV_PART_MAIN);
    lv_obj_set_style_border_width(sw, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(sw, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, COL_AMBER, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_radius(sw, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sw, COL_AMBER, LV_PART_KNOB);
    lv_obj_set_style_radius(sw, 0, LV_PART_KNOB);
}

/* ---- Meters ------------------------------------------------------------- */
lv_obj_t *kit_meter(lv_obj_t *parent, lv_coord_t w)
{
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, w, 18);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(bar, COL_PANEL_ITEM, 0);
    lv_obj_set_style_border_color(bar, COL_DIM, 0);
    lv_obj_set_style_border_width(bar, 2, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *fill = lv_obj_create(bar);
    lv_obj_remove_style_all(fill);
    lv_obj_set_height(fill, 10);
    lv_obj_set_width(fill, 0);
    lv_obj_align(fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(fill, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(fill, COL_AMBER, 0);
    return fill;
}
void kit_set_meter(lv_obj_t *fill, int pct, lv_color_t col)
{
    if (!fill) return;
    lv_coord_t w = lv_obj_get_content_width(lv_obj_get_parent(fill));
    if (w <= 1) w = 100;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    lv_obj_set_width(fill, pct * w / 100);
    lv_obj_set_style_bg_color(fill, col, 0);
}

/* ---- Flex page + rows --------------------------------------------------- */
lv_obj_t *kit_body(lv_obj_t *panel)
{
    lv_obj_t *b = lv_obj_create(panel);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, LV_PCT(92), LV_PCT(82));
    lv_obj_align(b, LV_ALIGN_TOP_MID, 0, 76);
    lv_obj_set_scroll_dir(b, LV_DIR_VER);
    lv_obj_set_flex_flow(b, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(b, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(b, 18, 0);
    return b;
}

/* a transparent full-width cell; column or row flow */
static lv_obj_t *cell(lv_obj_t *parent, lv_flex_flow_t flow, lv_coord_t gap)
{
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_remove_style_all(c);
    lv_obj_set_width(c, LV_PCT(100));
    lv_obj_set_height(c, LV_SIZE_CONTENT);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(c, flow);
    if (flow == LV_FLEX_FLOW_ROW)
        lv_obj_set_flex_align(c, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(c, gap, 0);
    return c;
}

static lv_obj_t *amber_label(lv_obj_t *parent, const char *t)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, t);
    lv_obj_set_style_text_color(l, COL_AMBER, 0);
    return l;
}

lv_obj_t *kit_slider_row(lv_obj_t *body, const char *label, int min, int max, int val, lv_event_cb_t cb)
{
    lv_obj_t *col = cell(body, LV_FLEX_FLOW_COLUMN, 8);
    lv_obj_t *top = cell(col, LV_FLEX_FLOW_ROW, 0);
    amber_label(top, label);
    lv_obj_t *v = lv_label_create(top);
    char buf[8]; snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(v, buf);
    lv_obj_set_style_text_color(v, COL_MUTE, 0);

    lv_obj_t *s = lv_slider_create(col);
    lv_obj_set_width(s, LV_PCT(100));
    lv_slider_set_range(s, min, max);
    lv_slider_set_value(s, val, LV_ANIM_OFF);
    kit_style_slider(s);
    lv_obj_add_event_cb(s, kit_slider_sfx_cb, LV_EVENT_VALUE_CHANGED, NULL);
    if (cb) lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return v;
}

lv_obj_t *kit_switch_row(lv_obj_t *body, const char *label, bool on, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *row = cell(body, LV_FLEX_FLOW_ROW, 0);
    amber_label(row, label);
    lv_obj_t *sw = lv_switch_create(row);
    kit_style_switch(sw);
    if (on) lv_obj_add_state(sw, LV_STATE_CHECKED);
    lv_obj_add_event_cb(sw, kit_btn_sfx_cb, LV_EVENT_VALUE_CHANGED, NULL);
    if (cb) lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, ud);
    return sw;
}

lv_obj_t *kit_meter_row(lv_obj_t *body, const char *label, lv_obj_t **fill_out)
{
    lv_obj_t *col = cell(body, LV_FLEX_FLOW_COLUMN, 8);
    lv_obj_t *top = cell(col, LV_FLEX_FLOW_ROW, 0);
    amber_label(top, label);
    lv_obj_t *v = lv_label_create(top);
    lv_label_set_text(v, "--");
    lv_obj_set_style_text_color(v, COL_MUTE, 0);
    if (fill_out) *fill_out = kit_meter(col, LV_PCT(100));
    return v;
}

lv_obj_t *kit_list_row(lv_obj_t *body, const char *text, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_btn_create(body);
    lv_obj_set_width(b, LV_PCT(100));
    lv_obj_set_height(b, 56);
    kit_style_btn(b);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, FONT_HEAD, 0);
    lv_obj_align(l, LV_ALIGN_LEFT_MID, 20, 0);
    return b;
}

lv_obj_t *kit_row(lv_obj_t *body)
{
    return cell(body, LV_FLEX_FLOW_ROW, 0);
}
