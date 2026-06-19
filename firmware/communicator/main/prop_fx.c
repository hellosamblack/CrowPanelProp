/* prop_fx — CRT-style post overlay on the LVGL top layer. See prop_fx.h.
 *
 * Design constraints learned the hard way:
 *  - The scanline canvas is a full-screen ARGB buffer (~1.8 MB PSRAM), so it is
 *    allocated LAZILY: built only while enabled, torn down on disable. An
 *    off-by-default effect must not hold that memory (it starves the /screenshot
 *    snapshot buffer).
 *  - NO continuously-animated full-screen element. A rolling bar swept across the
 *    whole screen forces a full recomposite every frame, which pegs the LVGL task
 *    and starves the render lock (touch + /screenshot both hang). The overlay is
 *    therefore STATIC: it only recomposites over regions the UI already redrew
 *    (mainly the waveform track), which is bounded per frame.
 */
#include "prop_fx.h"
#include "prop_settings.h"
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#define FX_TAG "PROP_FX"

#define FX_W 1024
#define FX_H 600

/* Cassette-CRT palette (mirrors prop_ui.c). */
#define FX_AMBER lv_color_hex(0xE0B000)
#define FX_BLACK lv_color_hex(0x000000)

/* Static look of the overlay (the live strength is the layer opacity). */
#define FX_SCAN_STEP   3      /* px between scanlines */
#define FX_SCAN_OPA    90     /* darkness of each scanline */
#define FX_GRID_STEP   96     /* px between vertical grid lines */
#define FX_GRID_OPA    22     /* faint grid */
#define FX_WASH_OPA    14     /* phosphor amber wash */

static lv_obj_t *s_canvas;       /* baked scanlines + grid + wash */
static void *s_canvas_buf;
static bool s_built;
static bool s_enabled;
static uint8_t s_intensity = 55;

/* Map intensity 0..100 to a layer opacity. Capped below full so the overlay
 * never completely smothers the readout. */
static lv_opa_t intensity_opa(uint8_t pct)
{
    if (pct > 100) pct = 100;
    return (lv_opa_t)(pct * 230 / 100);
}

/* Bake the static overlay (scanlines + grid + phosphor wash) into the canvas. */
static void paint_canvas(void)
{
    lv_canvas_fill_bg(s_canvas, FX_AMBER, FX_WASH_OPA);   /* faint phosphor wash */

    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.radius = 0;
    d.border_width = 0;

    /* Scanlines: darkened rows. */
    d.bg_color = FX_BLACK;
    d.bg_opa = FX_SCAN_OPA;
    for (int y = 0; y < FX_H; y += FX_SCAN_STEP) {
        lv_canvas_draw_rect(s_canvas, 0, y, FX_W, 1, &d);
    }

    /* Faint vertical grid. */
    d.bg_color = FX_AMBER;
    d.bg_opa = FX_GRID_OPA;
    for (int x = 0; x < FX_W; x += FX_GRID_STEP) {
        lv_canvas_draw_rect(s_canvas, x, 0, 1, FX_H, &d);
    }
}

/* Allocate + build the overlay on the top layer. Call with LVGL locked. */
static bool fx_build(void)
{
    if (s_built) {
        return true;
    }
    size_t buf_sz = LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(FX_W, FX_H);
    s_canvas_buf = heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM);
    if (!s_canvas_buf) {
        ESP_LOGE(FX_TAG, "no PSRAM for fx canvas (%u bytes)", (unsigned)buf_sz);
        return false;
    }

    lv_obj_t *top = lv_layer_top();   /* composited above every screen on the panel */
    s_canvas = lv_canvas_create(top);
    lv_canvas_set_buffer(s_canvas, s_canvas_buf, FX_W, FX_H, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_align(s_canvas, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_clear_flag(s_canvas, LV_OBJ_FLAG_CLICKABLE);   /* touches pass through */
    lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_FLOATING);
    paint_canvas();
    lv_obj_set_style_opa(s_canvas, intensity_opa(s_intensity), 0);

    s_built = true;
    return true;
}

/* Delete the overlay and free the canvas buffer. Call with LVGL locked. */
static void fx_teardown(void)
{
    if (s_canvas) {
        lv_obj_del(s_canvas);
        s_canvas = NULL;
    }
    if (s_canvas_buf) {
        free(s_canvas_buf);
        s_canvas_buf = NULL;
    }
    s_built = false;
}

esp_err_t prop_fx_init(void)
{
    uint32_t v = 55;
    prop_settings_get_u32("fx_intensity", &v, 55);
    s_intensity = (uint8_t)(v > 100 ? 100 : v);
    uint32_t on = 0;
    prop_settings_get_u32("fx_on", &on, 0);
    s_enabled = on != 0;

    if (s_enabled) {
        if (!lvgl_port_lock(1000)) {
            return ESP_FAIL;
        }
        bool ok = fx_build();
        lvgl_port_unlock();
        if (!ok) {
            s_enabled = false;
        }
    }
    ESP_LOGI(FX_TAG, "fx ready (enabled=%d intensity=%d)", s_enabled, s_intensity);
    return ESP_OK;
}

void prop_fx_set_enabled(bool on)
{
    if (on == s_enabled && (!on || s_built)) {
        return;
    }
    if (!lvgl_port_lock(500)) {
        return;
    }
    if (on) {
        s_enabled = fx_build();
    } else {
        fx_teardown();
        s_enabled = false;
    }
    lvgl_port_unlock();
    prop_settings_set_u32("fx_on", s_enabled ? 1 : 0);
}

bool prop_fx_enabled(void)
{
    return s_enabled;
}

void prop_fx_set_intensity(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_intensity = pct;
    if (s_built && lvgl_port_lock(200)) {
        lv_obj_set_style_opa(s_canvas, intensity_opa(pct), 0);
        lvgl_port_unlock();
    }
    prop_settings_set_u32("fx_intensity", pct);
}

uint8_t prop_fx_intensity(void)
{
    return s_intensity;
}
