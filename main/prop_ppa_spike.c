/* prop_ppa_spike — offline PPA blend A8 spike: latency + correctness harness.
 *
 * Phase B step 1 of the framerate-optimisation plan. Measures whether the
 * ESP32-P4 PPA blend engine (A8 foreground / fixed black / RGB565 background)
 * is faster than the equivalent software src-over loop for a full 1024×600
 * frame. All work is done in PSRAM scratch buffers — the live LVGL framebuffer
 * and flush path are never touched.
 *
 * HARD CONSTRAINTS (enforced here):
 *   - No LVGL objects, no lvgl_port_lock, no flush/render callbacks.
 *   - All PPA-visible buffers in PSRAM, 64-byte aligned, 64-byte-aligned sizes.
 *   - PPA client registered and unregistered within this call (one-shot).
 *   - All scratch buffers freed before returning (no PSRAM leak).
 *   - esp_cache_msync writeback on inputs BEFORE ppa_do_blend; ppa_do_blend
 *     also does its own internal sync, but we be explicit on input writeback to
 *     ensure CPU writes are visible to the DMA.
 *
 * Trigger: POST /cmd {"cmd":"fx","ppaspike":true}  (wired in prop_api.c)
 */

#include "prop_ppa_spike.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "driver/ppa.h"

#define TAG "PPA_SPIKE"

/* --------------------------------------------------------------------------
 * Canvas geometry — mirrors prop_fx.c exactly.
 * -------------------------------------------------------------------------- */
#define FX_W            1024
#define FX_H            600
#define FX_SCAN_STEP    3       /* rows between scanlines                     */
#define FX_SCAN_OPA     95      /* scanline darkness alpha (0-255)            */
#define FX_VIGN         80      /* vignette depth in px from each edge        */
#define FX_VIGN_MAX     70      /* darkest vignette alpha at the very edge    */

/* Default effect levels (60 % scan, 45 % vignette — matches prop_fx defaults). */
#define SCAN_PCT        60
#define VIGN_PCT        45

/* --------------------------------------------------------------------------
 * Buffer layout.
 *   bg   : 1024×600 × 2 B/px  = 1 228 800 B  RGB565
 *   mask : 1024×600 × 1 B/px  =   614 400 B  A8
 *   out  : 1024×600 × 2 B/px  = 1 228 800 B  RGB565
 * Total  ≈ 3 072 000 B (~2.93 MB) PSRAM.
 * All sizes rounded UP to the next 64-byte boundary.
 * -------------------------------------------------------------------------- */
#define ALIGN64         64u
#define ALIGN_UP64(n)   (((size_t)(n) + (ALIGN64 - 1u)) & ~(ALIGN64 - 1u))

#define BG_PIXELS       ((size_t)FX_W * FX_H)
#define BG_SIZE         ALIGN_UP64(BG_PIXELS * 2u)      /* RGB565            */
#define MASK_SIZE       ALIGN_UP64(BG_PIXELS * 1u)      /* A8                */
#define OUT_SIZE        BG_SIZE                          /* RGB565, same dims */

/* Number of blend iterations for the timing loop. */
#define N_ITERS 100

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

/* Scale base alpha (0-255) by a 0..100 percent level. */
static inline uint8_t scale_opa(uint8_t base, uint8_t pct)
{
    return (uint8_t)((uint16_t)base * pct / 100u);
}

/* Fill an RGB565 background buffer with a simple diagonal gradient so the
 * output is not trivially all-black (makes correctness sampling meaningful). */
static void fill_bg_gradient(uint16_t *buf, int w, int h)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            /* Amber-ish gradient: ramp R/G channels diagonally */
            uint8_t r5 = (uint8_t)(((x + y) % 256) >> 3);   /* 0-31 */
            uint8_t g6 = (uint8_t)(((x * 2 + y) % 256) >> 2); /* 0-63 */
            uint8_t b5 = 0u;
            buf[y * w + x] = (uint16_t)((r5 << 11) | (g6 << 5) | b5);
        }
    }
}

/* Bake the scanlines + vignette CRT overlay into an A8 mask buffer.
 * Each byte is the alpha of *black* to composite over the background.
 * This mirrors the geometry in prop_fx.c::paint_canvas(). */
static void bake_a8_mask(uint8_t *mask, int w, int h)
{
    /* Start with a transparent mask. */
    memset(mask, 0, (size_t)w * h);

    /* Scanlines: darkened rows every FX_SCAN_STEP pixels. */
    uint8_t scan = scale_opa(FX_SCAN_OPA, SCAN_PCT);
    if (scan) {
        for (int y = 0; y < h; y += FX_SCAN_STEP) {
            memset(mask + y * w, scan, (size_t)w);
        }
    }

    /* Vignette: nested 1-px black frames, darkest at the edge. */
    for (int i = 0; i < FX_VIGN; i++) {
        uint8_t edge = (uint8_t)((uint32_t)FX_VIGN_MAX * (FX_VIGN - i) / FX_VIGN);
        uint8_t o = scale_opa(edge, VIGN_PCT);
        if (!o) continue;

        /* Clamp combined alpha (max of scanline alpha and vignette alpha) to
         * avoid overflow; the PPA treats the A8 byte as the per-pixel fg alpha
         * directly, so we just take the maximum at each pixel boundary row. */
#define SET_VIGN_PIXEL(idx)  do { \
    uint8_t cur = mask[idx]; \
    mask[idx] = (o > cur) ? o : cur; \
} while (0)

        /* Top row i */
        for (int x = 0; x < w; x++) { SET_VIGN_PIXEL(i * w + x); }
        /* Bottom row (h-1-i) */
        for (int x = 0; x < w; x++) { SET_VIGN_PIXEL((h - 1 - i) * w + x); }
        /* Left column i */
        for (int y = 0; y < h; y++) { SET_VIGN_PIXEL(y * w + i); }
        /* Right column (w-1-i) */
        for (int y = 0; y < h; y++) { SET_VIGN_PIXEL(y * w + (w - 1 - i)); }

#undef SET_VIGN_PIXEL
    }
}

/* --------------------------------------------------------------------------
 * Software reference blend: for each pixel, composite solid black at the
 * mask's A8 alpha over the RGB565 background, writing RGB565 output.
 * This is the equivalent of what the PPA does in the A8 / fixed-black mode.
 *
 * src-over with opaque fg colour (black, R=G=B=0):
 *   out_r = (0 * a + bg_r * (255-a)) / 255  =>  bg_r * inv_a / 255
 *   out_g = (0 * a + bg_g * (255-a)) / 255  =>  bg_g * inv_a / 255
 *   out_b = (0 * a + bg_b * (255-a)) / 255  =>  bg_b * inv_a / 255
 * -------------------------------------------------------------------------- */
static void sw_blend_a8_black(const uint16_t *bg,
                              const uint8_t  *mask,
                              uint16_t       *out,
                              int w, int h)
{
    int total = w * h;
    for (int i = 0; i < total; i++) {
        uint16_t px = bg[i];
        uint8_t  a  = mask[i];
        if (a == 0) {
            out[i] = px;
            continue;
        }
        uint32_t inv_a = 255u - a;

        /* Unpack RGB565 */
        uint32_t r5 = (px >> 11) & 0x1Fu;
        uint32_t g6 = (px >>  5) & 0x3Fu;
        uint32_t b5 =  px        & 0x1Fu;

        /* Scale each channel */
        uint32_t or5 = (r5 * inv_a + 127u) / 255u;
        uint32_t og6 = (g6 * inv_a + 127u) / 255u;
        uint32_t ob5 = (b5 * inv_a + 127u) / 255u;

        out[i] = (uint16_t)((or5 << 11) | (og6 << 5) | ob5);
    }
}

/* Simple additive checksum over an RGB565 output buffer. */
static uint32_t checksum_rgb565(const uint16_t *buf, size_t npixels)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < npixels; i++) {
        sum += buf[i];
    }
    return sum;
}

/* --------------------------------------------------------------------------
 * prop_ppa_spike_run — the one-shot measurement entry point.
 * -------------------------------------------------------------------------- */
void prop_ppa_spike_run(void)
{
    ESP_LOGI(TAG, "=== PPA blend spike START ===");
    ESP_LOGI(TAG, "Canvas %d x %d, RGB565 bg + A8 mask (scanlines+vignette), fg=black", FX_W, FX_H);

    /* ------------------------------------------------------------------ */
    /* 1. Allocate scratch buffers in PSRAM, 64-byte aligned.              */
    /* ------------------------------------------------------------------ */
    uint16_t *bg   = heap_caps_aligned_alloc(ALIGN64, BG_SIZE,   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t  *mask = heap_caps_aligned_alloc(ALIGN64, MASK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint16_t *out  = heap_caps_aligned_alloc(ALIGN64, OUT_SIZE,  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!bg || !mask || !out) {
        ESP_LOGE(TAG, "PSRAM alloc failed (bg=%p mask=%p out=%p) -- aborting", bg, mask, out);
        goto cleanup;
    }
    ESP_LOGI(TAG, "Buffers: bg=%p (%u B) mask=%p (%u B) out=%p (%u B)",
             bg, (unsigned)BG_SIZE, mask, (unsigned)MASK_SIZE, out, (unsigned)OUT_SIZE);

    /* ------------------------------------------------------------------ */
    /* 2. Populate buffers with test content.                              */
    /* ------------------------------------------------------------------ */
    fill_bg_gradient(bg, FX_W, FX_H);
    bake_a8_mask(mask, FX_W, FX_H);
    /* Clear output buffer so stale data doesn't fool the checksum. */
    memset(out, 0, OUT_SIZE);

    /* ------------------------------------------------------------------ */
    /* 3. Register PPA blend client.                                        */
    /* ------------------------------------------------------------------ */
    ppa_client_handle_t ppa_client = NULL;
    ppa_client_config_t client_cfg = {
        .oper_type           = PPA_OPERATION_BLEND,
        .max_pending_trans_num = 1,   /* blocking mode: 1 is sufficient    */
        /* data_burst_length defaults to PPA_DATA_BURST_LENGTH_128 */
    };
    esp_err_t ret = ppa_register_client(&client_cfg, &ppa_client);
    if (ret != ESP_OK || !ppa_client) {
        ESP_LOGE(TAG, "ppa_register_client failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGI(TAG, "PPA blend client registered");

    /* ------------------------------------------------------------------ */
    /* 4. Build the blend config (reused across the timing loop).          */
    /*                                                                     */
    /* Struct fields used (from driver/ppa.h):                            */
    /*   ppa_in_pic_blk_config_t: buffer, pic_w, pic_h,                  */
    /*                            block_w, block_h,                       */
    /*                            block_offset_x, block_offset_y,        */
    /*                            blend_cm                                */
    /*   ppa_out_pic_blk_config_t: buffer, buffer_size,                  */
    /*                             pic_w, pic_h,                          */
    /*                             block_offset_x, block_offset_y,       */
    /*                             blend_cm                               */
    /*   ppa_blend_oper_config_t: in_bg, in_fg, out,                     */
    /*                            bg_rgb_swap, bg_byte_swap,              */
    /*                            bg_alpha_update_mode,                   */
    /*                            fg_rgb_swap, fg_byte_swap,              */
    /*                            fg_alpha_update_mode,                   */
    /*                            fg_fix_rgb_val (color_pixel_rgb888_data_t: .r .g .b) */
    /*                            bg_ck_en, fg_ck_en,                    */
    /*                            mode                                    */
    /*                                                                     */
    /* A8 alpha mode: in_fg.blend_cm = PPA_BLEND_COLOR_MODE_A8.          */
    /* The A8 byte is the per-pixel foreground alpha (PPA_ALPHA_NO_CHANGE */
    /* on fg — the hardware uses the A8 value directly as alpha).         */
    /* fg_fix_rgb_val = {.r=0, .g=0, .b=0} for solid black.             */
    /* ------------------------------------------------------------------ */
    ppa_blend_oper_config_t blend_cfg = {
        .in_bg = {
            .buffer         = (const void *)bg,
            .pic_w          = FX_W,
            .pic_h          = FX_H,
            .block_w        = FX_W,
            .block_h        = FX_H,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .blend_cm       = PPA_BLEND_COLOR_MODE_RGB565,
        },
        .bg_rgb_swap          = false,
        .bg_byte_swap         = false,
        .bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .bg_alpha_fix_val     = 0,
        .bg_ck_en             = false,

        .in_fg = {
            .buffer         = (const void *)mask,
            .pic_w          = FX_W,
            .pic_h          = FX_H,
            .block_w        = FX_W,
            .block_h        = FX_H,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .blend_cm       = PPA_BLEND_COLOR_MODE_A8,
        },
        .fg_rgb_swap          = false,
        .fg_byte_swap         = false,
        /* PPA_ALPHA_NO_CHANGE on A8 input: the A8 byte IS the alpha.    */
        .fg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .fg_alpha_fix_val     = 0,
        .fg_ck_en             = false,

        /* Solid black foreground colour (used when blend_cm is A8/A4). */
        .fg_fix_rgb_val = { .r = 0, .g = 0, .b = 0 },

        .out = {
            .buffer         = (void *)out,
            .buffer_size    = OUT_SIZE,  /* 64-byte aligned via ALIGN_UP64 */
            .pic_w          = FX_W,
            .pic_h          = FX_H,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .blend_cm       = PPA_BLEND_COLOR_MODE_RGB565,
        },

        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    /* ------------------------------------------------------------------ */
    /* 5. Cache writeback of input buffers before the timing loop.         */
    /*    The PPA driver (ppa_blend.c) also does this internally, but we  */
    /*    do it here to cover the initial CPU writes from steps 2 above    */
    /*    before the first DMA read.  Subsequent iterations reuse the same */
    /*    buffers (bg and mask never change during timing), so the first   */
    /*    writeback is sufficient for the inputs.                          */
    /* ------------------------------------------------------------------ */
    esp_cache_msync((void *)bg,   BG_SIZE,   ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    esp_cache_msync((void *)mask, MASK_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    /* ------------------------------------------------------------------ */
    /* 6. PPA timing loop — N_ITERS blocking blend ops.                    */
    /* ------------------------------------------------------------------ */
    int64_t ppa_start = esp_timer_get_time();
    for (int i = 0; i < N_ITERS; i++) {
        ret = ppa_do_blend(ppa_client, &blend_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ppa_do_blend failed at iter %d: %s", i, esp_err_to_name(ret));
            goto cleanup_client;
        }
    }
    int64_t ppa_end = esp_timer_get_time();
    int64_t ppa_total_us = ppa_end - ppa_start;
    int64_t ppa_us_per_op = ppa_total_us / N_ITERS;

    /* After the last PPA op (blocking), the driver has already invalidated
     * the output cache window.  Invalidate the full out buffer to be safe
     * before our CPU reads it for the checksum. */
    esp_cache_msync((void *)out, OUT_SIZE,
                    ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    /* ------------------------------------------------------------------ */
    /* 7. Correctness: sample known pixels from the PPA output.            */
    /* ------------------------------------------------------------------ */
    /* Row 0, pixel 0: should be a scanline row (y=0 is a multiple of 3). */
    uint16_t px_scan_row   = out[0];                          /* y=0, x=0 — scanline hit    */
    /* Row 1, pixel 0: non-scanline row (y=1 is NOT a multiple of 3).     */
    uint16_t px_noscan_row = out[1 * FX_W + 0];              /* y=1, x=0                   */
    /* Vignette edge: pixel at (0, FX_VIGN/2) — well inside vignette zone.*/
    uint16_t px_vign_edge  = out[(FX_VIGN / 2) * FX_W + 0]; /* left edge mid-vignette      */
    /* Centre pixel: no scanline (y=300 not on step), no vignette.        */
    uint16_t px_centre     = out[300 * FX_W + 512];          /* y=300, x=512                */

    uint32_t ppa_csum = checksum_rgb565(out, BG_PIXELS);

    ESP_LOGI(TAG, "PPA pixel samples:");
    ESP_LOGI(TAG, "  (y=0,x=0)   scanline row : 0x%04X", px_scan_row);
    ESP_LOGI(TAG, "  (y=1,x=0)   non-scanline : 0x%04X", px_noscan_row);
    ESP_LOGI(TAG, "  (y=%d,x=0)  vignette edge: 0x%04X", FX_VIGN / 2, px_vign_edge);
    ESP_LOGI(TAG, "  (y=300,x=512) centre      : 0x%04X", px_centre);
    ESP_LOGI(TAG, "PPA output checksum: 0x%08" PRIX32, ppa_csum);

    /* ------------------------------------------------------------------ */
    /* 8. Software reference timing loop — N_ITERS SW blend ops.           */
    /* ------------------------------------------------------------------ */
    int64_t sw_start = esp_timer_get_time();
    for (int i = 0; i < N_ITERS; i++) {
        sw_blend_a8_black(bg, mask, out, FX_W, FX_H);
    }
    int64_t sw_end = esp_timer_get_time();
    int64_t sw_total_us = sw_end - sw_start;
    int64_t sw_us_per_op = sw_total_us / N_ITERS;

    /* SW writes go to cached PSRAM; invalidate before reading checksum. */
    esp_cache_msync((void *)out, OUT_SIZE,
                    ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    uint32_t sw_csum = checksum_rgb565(out, BG_PIXELS);

    /* ------------------------------------------------------------------ */
    /* 9. Log the timing banner + checksums.                               */
    /* ------------------------------------------------------------------ */
    float ratio = (sw_us_per_op > 0) ? (float)sw_us_per_op / (float)ppa_us_per_op : 0.0f;

    /* Use integer formatting to avoid the LVGL-internal printf %f issue:
     * this is ESP_LOGI (not lv_label_set_text_fmt), so %f works fine here. */
    ESP_LOGI(TAG, "PPA_SPIKE: blend A8 full-screen: PPA=%lld us/op  SW=%lld us/op  ratio=%.2f",
             (long long)ppa_us_per_op, (long long)sw_us_per_op, (double)ratio);
    ESP_LOGI(TAG, "PPA_SPIKE: N=%d iters  PPA_total=%lld us  SW_total=%lld us",
             N_ITERS, (long long)ppa_total_us, (long long)sw_total_us);
    if (ppa_csum == sw_csum) {
        ESP_LOGI(TAG, "PPA_SPIKE: checksum MATCH (0x%08" PRIX32 ") -- PPA and SW produce same output", ppa_csum);
    } else {
        ESP_LOGW(TAG, "PPA_SPIKE: checksum MISMATCH PPA=0x%08" PRIX32 " SW=0x%08" PRIX32
                 " (possible rounding diff or alpha interpretation difference)", ppa_csum, sw_csum);
    }
    if (ppa_us_per_op < sw_us_per_op) {
        ESP_LOGI(TAG, "PPA_SPIKE: VERDICT -> PPA is FASTER (%.2fx speedup)", (double)ratio);
    } else {
        ESP_LOGW(TAG, "PPA_SPIKE: VERDICT -> PPA is NOT faster than SW (ratio %.2f < 1.0 means SW wins)", (double)ratio);
    }

cleanup_client:
    /* ------------------------------------------------------------------ */
    /* 10. Unregister PPA client.                                           */
    /* ------------------------------------------------------------------ */
    if (ppa_client) {
        ret = ppa_unregister_client(ppa_client);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ppa_unregister_client failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "PPA client unregistered");
        }
    }

cleanup:
    /* ------------------------------------------------------------------ */
    /* 11. Free all scratch buffers.                                        */
    /* ------------------------------------------------------------------ */
    if (bg)   heap_caps_free(bg);
    if (mask) heap_caps_free(mask);
    if (out)  heap_caps_free(out);

    ESP_LOGI(TAG, "=== PPA blend spike END — all scratch buffers freed ===");
}
