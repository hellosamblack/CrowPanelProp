/* prop_fx — CRT-style post overlay on the LVGL top layer. See prop_fx.h.
 *
 * Two pieces, both on lv_layer_top() (composited above every screen on the panel):
 *
 *  1. A STATIC baked canvas: horizontal scanlines + a phosphor amber wash + an
 *     edge vignette (the curved-tube glow). Allocated LAZILY (full-screen ARGB,
 *     ~1.8 MB PSRAM) — built only while enabled, torn down on disable, so an
 *     off-by-default effect never holds that memory (it would starve /screenshot).
 *
 *  2. A slow horizontal REFRESH LINE: a thin (~80 px) floating canvas with a soft
 *     amber gradient + bright core that scrolls top->bottom and wraps. This is the
 *     only animated element. The historical hazard (CLAUDE.md) is a *full-screen*
 *     animated element: it recomposites the whole panel every frame and pegs the
 *     LVGL task. This band is deliberately thin and stepped slowly, so each move
 *     invalidates only its own bounded stripe (old ∪ new ≈ 82 px tall), not the
 *     whole screen. Do NOT widen it to full height or speed it up much.
 *
 * NB: /screenshot reads the DPI framebuffer directly, so it DOES capture this
 * top-layer overlay (see prop_api.c) — captures are good enough to judge it.
 */
#include "prop_fx.h"
#include "prop_settings.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#define FX_TAG "PROP_FX"

#define FX_W 1024
#define FX_H 600

/* Cassette-CRT palette (mirrors prop_ui.c). */
#define FX_AMBER lv_color_hex(0xE0B000)
#define FX_GLOW  lv_color_hex(0xFFF0C0)   /* hot phosphor core (near-white amber) */
#define FX_BLACK lv_color_hex(0x000000)

/* Static look of the overlay (the live strength is the layer opacity). */
#define FX_SCAN_STEP   3      /* px between scanlines */
#define FX_SCAN_OPA    95     /* darkness of each scanline (crisper horizontal lines) */
#define FX_WASH_OPA    20     /* phosphor amber wash (glow) */
#define FX_VIGN        80     /* vignette depth in px from each edge */
#define FX_VIGN_MAX    70     /* darkest vignette opacity at the very edge */

/* Scrolling refresh line. */
#define FX_BAND_H      80     /* band height (keep thin — bounds the recomposite) */
#define FX_BAND_STEP   2      /* px advanced per tick */
#define FX_BAND_MS     55     /* tick period (~18 fps; slow drift) */
#define FX_BAND_PEAK   70     /* soft glow alpha at the band centre */

static lv_obj_t *s_canvas;       /* baked scanlines + wash + vignette */
static void *s_canvas_buf;
static lv_obj_t *s_band;         /* scrolling refresh line */
static void *s_band_buf;
static lv_timer_t *s_band_timer;
static int s_band_y;
static bool s_built;
static bool s_enabled;

/* Per-effect levels, 0..100 (0 = that effect contributes nothing). */
static uint8_t s_scan_pct     = 60;   /* scanline darkness        */
static uint8_t s_phosphor_pct = 30;   /* amber wash / glow        */
static uint8_t s_vignette_pct = 45;   /* edge falloff             */
static uint8_t s_refresh_pct  = 25;   /* scrolling sweep band     */

/* Scale a base alpha (0..255) by a 0..100 percent level. */
static lv_opa_t scale_opa(uint8_t base, uint8_t pct)
{
    if (pct > 100) pct = 100;
    return (lv_opa_t)((uint16_t)base * pct / 100);
}

/* Alpha-blend (src-over) a solid-colour rect straight into an ARGB8888 canvas buffer.
 * v9's canvas layer-draw path (lv_draw_rect + lv_canvas_finish_layer) replaced v8's
 * synchronous lv_canvas_draw_rect, but its dispatch loop can't be driven from a
 * non-LVGL task while the port lock is held — it spins forever (boot watchdog). These
 * overlays are just solid rects with alpha, so we composite them by hand. Buffer is
 * tightly packed (stride = w*4; LV_DRAW_BUF_STRIDE_ALIGN == 1), one uint32 0xAARRGGBB
 * per pixel. */
static void fx_fill(void *buf, int bw, int bh, int x0, int y0, int w, int h,
                    lv_color_t color, lv_opa_t opa)
{
    if (opa == 0 || w <= 0 || h <= 0) return;
    int x1 = x0 + w, y1 = y0 + h;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > bw) x1 = bw;
    if (y1 > bh) y1 = bh;
    uint32_t sr = color.red, sg = color.green, sb = color.blue, sa = opa, ia = 255u - opa;
    uint32_t *base = (uint32_t *)buf;
    for (int y = y0; y < y1; y++) {
        uint32_t *row = base + (size_t)y * bw;
        for (int x = x0; x < x1; x++) {
            uint32_t d = row[x];
            uint32_t da = d >> 24;
            uint32_t oa = sa + da * ia / 255u;
            if (oa == 0) { row[x] = 0; continue; }
            uint32_t dr = (d >> 16) & 0xFF, dg = (d >> 8) & 0xFF, db = d & 0xFF;
            uint32_t r = (sr * sa + dr * da * ia / 255u) / oa;
            uint32_t g = (sg * sa + dg * da * ia / 255u) / oa;
            uint32_t b = (sb * sa + db * da * ia / 255u) / oa;
            row[x] = (oa << 24) | (r << 16) | (g << 8) | b;
        }
    }
}

/* ---- LUT-accelerated bake ---------------------------------------------------
 * Every canvas pixel is "solid black at some alpha `a`, src-over the constant
 * phosphor wash". The final ARGB value therefore depends ONLY on `a` — so instead
 * of running the src-over math per pixel (the old fx_fill loops, ~40 ms per full
 * bake), precompute a 256-entry alpha→ARGB table once per bake and reduce the whole
 * composite to one table lookup per pixel.
 *
 * (The PPA blend engine was evaluated for this — prop_ppa_spike.c verified A8-over-
 * RGB565 at ~2.5x the src-over loop — but the ARGB8888-out variant this canvas
 * needs is unverified on hardware, and a blocking PPA call sits on the boot path
 * before mark_app_valid, where a hang is unrecoverable over OTA. The LUT removes
 * the per-pixel math entirely, which is the same win with zero hardware risk.)
 *
 * Black alphas stack like the old sequential fills did: src-over of two black
 * layers a1 then a2 combines to a1 + a2 - a1*a2/255, so output parity with the
 * previous look is exact. */
static uint8_t combine_black_alpha(uint8_t a1, uint8_t a2)
{
    return (uint8_t)((uint32_t)a1 + a2 - ((uint32_t)a1 * a2) / 255u);
}

/* Bake the static overlay (scanlines + phosphor wash + vignette) into the canvas. */
static void paint_canvas(void)
{
    lv_opa_t wash = scale_opa(FX_WASH_OPA, s_phosphor_pct);
    lv_opa_t scan = scale_opa(FX_SCAN_OPA, s_scan_pct);
    lv_color_t amber = FX_AMBER;

    /* lut[a] = ARGB of (black @ alpha a) src-over (amber wash @ alpha `wash`). */
    uint32_t lut[256];
    for (int a = 0; a < 256; a++) {
        uint32_t oa = (uint32_t)a + wash - ((uint32_t)a * wash) / 255u;
        if (oa == 0) {
            lut[a] = 0;
            continue;
        }
        /* Black contributes no colour, so each channel is wash_c * wash * (1-a). */
        uint32_t k = (uint32_t)wash * (255u - a) / 255u;   /* effective wash weight */
        uint32_t r = (uint32_t)amber.red   * k / oa;
        uint32_t g = (uint32_t)amber.green * k / oa;
        uint32_t b = (uint32_t)amber.blue  * k / oa;
        lut[a] = (oa << 24) | (r << 16) | (g << 8) | b;
    }

    /* Per-column black alpha from the left/right vignette; per-row from the
     * scanline pattern + top/bottom vignette. Pixel alpha = row ⊕ column. */
    static uint8_t col_a[FX_W];
    memset(col_a, 0, sizeof(col_a));
    for (int i = 0; i < FX_VIGN; i++) {
        lv_opa_t edge = (lv_opa_t)(FX_VIGN_MAX * (FX_VIGN - i) / FX_VIGN);
        lv_opa_t o = scale_opa(edge, s_vignette_pct);
        if (!o) continue;
        col_a[i]            = combine_black_alpha(col_a[i], o);
        col_a[FX_W - 1 - i] = combine_black_alpha(col_a[FX_W - 1 - i], o);
    }

    uint32_t *px = (uint32_t *)s_canvas_buf;
    for (int y = 0; y < FX_H; y++) {
        uint8_t row_a = (y % FX_SCAN_STEP == 0) ? scan : 0;
        int vd = (y < FX_H - 1 - y) ? y : FX_H - 1 - y;   /* distance to nearer h-edge */
        if (vd < FX_VIGN) {
            lv_opa_t edge = (lv_opa_t)(FX_VIGN_MAX * (FX_VIGN - vd) / FX_VIGN);
            row_a = combine_black_alpha(row_a, scale_opa(edge, s_vignette_pct));
        }
        uint32_t *row = px + (size_t)y * FX_W;
        uint32_t base = lut[row_a];
        for (int x = 0; x < FX_W; x++) {
            row[x] = col_a[x] ? lut[combine_black_alpha(row_a, col_a[x])] : base;
        }
    }
}

/* Bake the refresh line: a soft amber glow that peaks at the band centre, with a
 * brighter 2px core — reads as a slightly blurred CRT refresh sweep. */
static void paint_band(void)
{
    lv_canvas_fill_bg(s_band, FX_AMBER, 0);   /* fully transparent base */

    int c = FX_BAND_H / 2;
    for (int y = 0; y < FX_BAND_H; y++) {
        float dist = fabsf((float)(y - c)) / (float)c;   /* 0 centre .. 1 edge */
        float prof = 1.0f - dist;
        if (prof <= 0) {
            continue;
        }
        prof *= prof;                                    /* soft falloff */
        fx_fill(s_band_buf, FX_W, FX_BAND_H, 0, y, FX_W, 1, FX_AMBER, (lv_opa_t)(prof * FX_BAND_PEAK));
    }
    /* Bright hot core line. */
    fx_fill(s_band_buf, FX_W, FX_BAND_H, 0, c - 1, FX_W, 2, FX_GLOW, 120);
}

/* Advance the refresh line (runs in the LVGL task under the port lock, so it must
 * NOT re-lock). Only the band's own stripe is invalidated by the move. */
static void band_tick(lv_timer_t *t)
{
    (void)t;
    if (!s_band) {
        return;
    }
    s_band_y += FX_BAND_STEP;
    if (s_band_y > FX_H) {
        s_band_y = -FX_BAND_H;
    }
    lv_obj_set_y(s_band, s_band_y);
}

/* Allocate + build the overlay on the top layer. Call with LVGL locked. */
static bool fx_build(void)
{
    if (s_built) {
        return true;
    }
    lv_obj_t *top = lv_layer_top();   /* composited above every screen on the panel */

    /* Static scanline/wash/vignette canvas. */
    size_t buf_sz = LV_CANVAS_BUF_SIZE(FX_W, FX_H, 32, LV_DRAW_BUF_STRIDE_ALIGN);
    s_canvas_buf = heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM);
    if (!s_canvas_buf) {
        ESP_LOGE(FX_TAG, "no PSRAM for fx canvas (%u bytes)", (unsigned)buf_sz);
        return false;
    }
    s_canvas = lv_canvas_create(top);
    lv_canvas_set_buffer(s_canvas, s_canvas_buf, FX_W, FX_H, LV_COLOR_FORMAT_ARGB8888);
    lv_obj_align(s_canvas, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_clear_flag(s_canvas, LV_OBJ_FLAG_CLICKABLE);   /* touches pass through */
    lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_FLOATING);
    paint_canvas();
    lv_obj_set_style_opa(s_canvas, LV_OPA_COVER, 0);

    /* Scrolling refresh line (thin band canvas + a slow timer). */
    size_t band_sz = LV_CANVAS_BUF_SIZE(FX_W, FX_BAND_H, 32, LV_DRAW_BUF_STRIDE_ALIGN);
    s_band_buf = heap_caps_malloc(band_sz, MALLOC_CAP_SPIRAM);
    if (s_band_buf) {
        s_band = lv_canvas_create(top);
        lv_canvas_set_buffer(s_band, s_band_buf, FX_W, FX_BAND_H, LV_COLOR_FORMAT_ARGB8888);
        lv_obj_clear_flag(s_band, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(s_band, LV_OBJ_FLAG_FLOATING);
        paint_band();
        lv_obj_set_style_opa(s_band, scale_opa(LV_OPA_COVER, s_refresh_pct), 0);
        s_band_y = -FX_BAND_H;
        lv_obj_set_pos(s_band, 0, s_band_y);
        s_band_timer = lv_timer_create(band_tick, FX_BAND_MS, NULL);
    } else {
        ESP_LOGW(FX_TAG, "no PSRAM for refresh line — scanlines only");
    }

    s_built = true;
    return true;
}

/* Delete the overlay and free buffers. Call with LVGL locked. */
static void fx_teardown(void)
{
    if (s_band_timer) {
        lv_timer_del(s_band_timer);
        s_band_timer = NULL;
    }
    if (s_band) {
        lv_obj_del(s_band);
        s_band = NULL;
    }
    if (s_band_buf) {
        free(s_band_buf);
        s_band_buf = NULL;
    }
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
    /* Migration: first boot after upgrade has no per-effect keys but may have the
     * legacy "fx_intensity". Seed each effect's default scaled by the old global
     * level so the look is roughly preserved; thereafter the per-effect keys win. */
    uint32_t legacy = 55;
    bool had_legacy = prop_settings_has("fx_intensity");
    prop_settings_get_u32("fx_intensity", &legacy, 55);

    uint32_t v;
    v = had_legacy ? (60u * legacy / 55u) : 60; prop_settings_get_u32("fx_scan",     &v, v); s_scan_pct     = (uint8_t)(v > 100 ? 100 : v);
    v = had_legacy ? (30u * legacy / 55u) : 30; prop_settings_get_u32("fx_phosphor", &v, v); s_phosphor_pct = (uint8_t)(v > 100 ? 100 : v);
    v = had_legacy ? (45u * legacy / 55u) : 45; prop_settings_get_u32("fx_vignette", &v, v); s_vignette_pct = (uint8_t)(v > 100 ? 100 : v);
    v = had_legacy ? (25u * legacy / 55u) : 25; prop_settings_get_u32("fx_refresh",  &v, v); s_refresh_pct  = (uint8_t)(v > 100 ? 100 : v);

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
    ESP_LOGI(FX_TAG, "fx ready (enabled=%d scan=%d phos=%d vign=%d refr=%d)",
             s_enabled, s_scan_pct, s_phosphor_pct, s_vignette_pct, s_refresh_pct);
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

/* Scanlines + phosphor + vignette are baked into the static canvas, so changing any
 * of them re-bakes that canvas (cheap; PSRAM, off the hot path). */
static void rebake_canvas_locked(void)
{
    if (s_built && s_canvas) {
        paint_canvas();
        lv_obj_invalidate(s_canvas);
    }
}

void prop_fx_set_scanlines(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_scan_pct = pct;
    if (lvgl_port_lock(200)) { rebake_canvas_locked(); lvgl_port_unlock(); }
    prop_settings_set_u32("fx_scan", pct);
}
uint8_t prop_fx_scanlines(void) { return s_scan_pct; }

void prop_fx_set_phosphor(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_phosphor_pct = pct;
    if (lvgl_port_lock(200)) { rebake_canvas_locked(); lvgl_port_unlock(); }
    prop_settings_set_u32("fx_phosphor", pct);
}
uint8_t prop_fx_phosphor(void) { return s_phosphor_pct; }

void prop_fx_set_vignette(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_vignette_pct = pct;
    if (lvgl_port_lock(200)) { rebake_canvas_locked(); lvgl_port_unlock(); }
    prop_settings_set_u32("fx_vignette", pct);
}
uint8_t prop_fx_vignette(void) { return s_vignette_pct; }

/* Refresh band has its own layer opacity (it is a separate floating canvas). */
void prop_fx_set_refresh(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_refresh_pct = pct;
    if (s_built && s_band && lvgl_port_lock(200)) {
        lv_obj_set_style_opa(s_band, scale_opa(LV_OPA_COVER, pct), 0);
        lvgl_port_unlock();
    }
    prop_settings_set_u32("fx_refresh", pct);
}
uint8_t prop_fx_refresh(void) { return s_refresh_pct; }

/* ---- Screen-change transition (the "old TV channel change") ----------------
 * Played by prop_ui right after a panel swap. A clipping container holds a noise
 * "static" image; the swap has already happened underneath, so the container masks
 * it and then animates away to reveal the new screen. One object, deleted by the
 * anim ready-cb (plus cleared on the next play), so it can never get stuck.
 *
 * On the ACTIVE screen (a normal top-most child), NOT lv_layer_top, so panel
 * teardown can never leave it orphaned above an unrelated screen.
 *
 * Flavors (NVS "fx_trans"): 0 off, 1 snow, 2 roll-up, 3 collapse, 4 snow+collapse. */
#define TR_NW 160
#define TR_NH 100
static lv_obj_t  *s_tr_box;        /* clipping container on the active screen */
static lv_obj_t  *s_tr_img;        /* noise image inside it (snow flavors only) */
static lv_obj_t  *s_tr_seam;       /* bright sweep/pinch line (roll/collapse) */
static void      *s_tr_buf;        /* noise pixel buffer (PSRAM) */
static lv_img_dsc_t s_tr_noise;
static uint32_t   s_tr_mode = 1;

static void tr_fill_noise(void)
{
    uint16_t *px = (uint16_t *)s_tr_buf;
    for (int i = 0; i < TR_NW * TR_NH; i++) {
        uint8_t v = (uint8_t)(rand() & 0xFF);          /* amber-channel static */
        uint8_t g = (uint8_t)((v * 0xB0) / 0xFF);
        px[i] = (uint16_t)(((v & 0xF8) << 8) | ((g & 0xFC) << 3));  /* RGB565, B=0 */
    }
}

static void tr_ready_cb(lv_anim_t *a)
{
    (void)a;
    if (s_tr_box) { lv_obj_del(s_tr_box); s_tr_box = NULL; s_tr_img = NULL; s_tr_seam = NULL; }
}

static void tr_exec_cb(void *var, int32_t v)
{
    (void)var;
    if (!s_tr_box) return;
    switch (s_tr_mode) {
        case 1:                                     /* snow: re-randomize each frame */
            tr_fill_noise();
            if (s_tr_img) lv_obj_invalidate(s_tr_img);
            break;
        case 2:                                     /* roll up: slide off the top */
            lv_obj_set_y(s_tr_box, -v);
            break;
        case 3: {                                   /* collapse to a center line */
            int h = FX_H - v; if (h < 2) h = 2;
            lv_obj_set_height(s_tr_box, h);
            lv_obj_set_y(s_tr_box, (FX_H - h) / 2);
            break; }
        case 4: {                                   /* snow collapsing to a line */
            tr_fill_noise();
            if (s_tr_img) lv_obj_invalidate(s_tr_img);
            int h = FX_H - v; if (h < 2) h = 2;
            lv_obj_set_height(s_tr_box, h);
            lv_obj_set_y(s_tr_box, (FX_H - h) / 2);
            break; }
    }
}

void prop_fx_transition_play(void)
{
    uint32_t mode = 1;
    prop_settings_get_u32("fx_trans", &mode, 1);
    if (mode == 0 || mode > 4) return;
    s_tr_mode = mode;

    /* Lazily allocate the noise buffer + descriptor (kept for the app lifetime — a
     * single small tile, re-randomized per play). */
    if (!s_tr_buf) {
        s_tr_buf = heap_caps_malloc(TR_NW * TR_NH * 2, MALLOC_CAP_SPIRAM);
        if (!s_tr_buf) return;
        s_tr_noise.header.cf = LV_COLOR_FORMAT_RGB565;
        s_tr_noise.header.w = TR_NW;
        s_tr_noise.header.h = TR_NH;
        s_tr_noise.header.stride = TR_NW * 2;          /* v9 image headers carry an explicit stride */
        s_tr_noise.data = (const uint8_t *)s_tr_buf;
        s_tr_noise.data_size = TR_NW * TR_NH * 2;
    }

    /* Cancel any in-flight transition cleanly before starting a new one. */
    if (s_tr_box) {
        lv_anim_del(s_tr_box, tr_exec_cb);
        lv_obj_del(s_tr_box);
        s_tr_box = NULL; s_tr_img = NULL; s_tr_seam = NULL;
    }

    bool snow = (mode == 1 || mode == 4);   /* noise-textured flavors */

    /* The mask: black for roll/collapse (the "tube" going dark), noise for snow.
     * v8 clips children to the box, so shrinking it crops the contents. */
    s_tr_box = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_tr_box);
    lv_obj_set_size(s_tr_box, FX_W, FX_H);
    lv_obj_set_pos(s_tr_box, 0, 0);
    lv_obj_clear_flag(s_tr_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_tr_box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_opa(s_tr_box, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_tr_box, FX_BLACK, 0);

    if (snow) {
        tr_fill_noise();
        s_tr_img = lv_image_create(s_tr_box);
        lv_image_set_src(s_tr_img, &s_tr_noise);
        lv_image_set_antialias(s_tr_img, false);
        lv_image_set_scale(s_tr_img, (256 * FX_W / TR_NW) + 1);   /* overfill 1024x600 */
        lv_obj_align(s_tr_img, LV_ALIGN_CENTER, 0, 0);
    }

    /* Bright phosphor sweep/pinch line for roll + collapse (not snow). */
    if (mode == 2 || mode == 3) {
        s_tr_seam = lv_obj_create(s_tr_box);
        lv_obj_remove_style_all(s_tr_seam);
        lv_obj_set_size(s_tr_seam, FX_W, 4);
        lv_obj_set_style_bg_opa(s_tr_seam, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(s_tr_seam, FX_GLOW, 0);
        /* roll: seam rides the box's lower edge; collapse: it sits at the pinch centre. */
        lv_obj_align(s_tr_seam, mode == 2 ? LV_ALIGN_BOTTOM_MID : LV_ALIGN_CENTER, 0, 0);
    }

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_tr_box);
    lv_anim_set_exec_cb(&a, tr_exec_cb);
    lv_anim_set_ready_cb(&a, tr_ready_cb);
    switch (mode) {
        case 1: lv_anim_set_values(&a, 0, 6);     lv_anim_set_time(&a, 280); break;  /* snow shimmer */
        case 2: lv_anim_set_values(&a, 0, FX_H);  lv_anim_set_time(&a, 420); break;  /* roll up */
        case 3: lv_anim_set_values(&a, 0, FX_H);  lv_anim_set_time(&a, 340); break;  /* collapse */
        case 4: lv_anim_set_values(&a, 0, FX_H);  lv_anim_set_time(&a, 440); break;  /* snow+collapse */
    }
    lv_anim_start(&a);
}
