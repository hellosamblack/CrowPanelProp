/**
 * @file lv_draw_ppa_fill.c
 *
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA

void lv_draw_ppa_fill(lv_draw_task_t * t, const lv_draw_fill_dsc_t * dsc,
                      const lv_area_t * coords)
{
    lv_draw_ppa_unit_t * u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t * layer = t->target_layer;
    lv_draw_buf_t * draw_buf = layer->draw_buf;
    int width  = lv_area_get_width(coords);
    int height = lv_area_get_height(coords);

    if(width <= 0 || height <= 0) {
        LV_LOG_WARN("Invalid draw area for filling!");
        return;
    }

    /* LOCAL PATCH (CrowPanelProp, Phase 0 PPA re-enablement). Upstream (experimental)
     * set out.pic_w/pic_h to the FILL size and block_offset to 0, which only works when
     * the fill covers the whole layer draw-buffer at its origin. For any sub-region fill
     * (e.g. the always-present rail/top/footer chrome) it wrote rows at the fill's stride
     * instead of the buffer's, at offset 0 — garbling those regions while full-area fills
     * looked fine. The picture is the WHOLE draw buffer; the block is the fill rect at its
     * offset within it (coords are absolute & already clipped to buf_area by the caller).
     * Matches the sibling lv_draw_ppa_img.c which already uses header.w/h + aligned size. */
    ppa_fill_oper_config_t cfg = {
        .fill_argb_color.val = lv_color_to_u32(dsc->color),
        .fill_block_w    = width,
        .fill_block_h    = height,
        .out = {
            .buffer         = draw_buf->data,
            .buffer_size    = PPA_ALIGN_UP(draw_buf->data_size, CONFIG_CACHE_L1_CACHE_LINE_SIZE),
            .pic_w          = draw_buf->header.w,
            .pic_h          = draw_buf->header.h,
            .block_offset_x = coords->x1 - layer->buf_area.x1,
            .block_offset_y = coords->y1 - layer->buf_area.y1,
            .fill_cm        = lv_color_format_to_ppa_fill(draw_buf->header.cf),
        },

        /* LOCAL PATCH (CrowPanelProp): mode MUST match the dispatch's completion logic.
         * Upstream hardcoded NON_BLOCKING, but with LV_PPA_NONBLOCKING_OPS=0 (our config,
         * the default) ppa_dispatch marks the task FINISHED immediately after this call
         * (lv_draw_ppa.c #if !LV_PPA_NONBLOCKING_OPS) WITHOUT waiting for the ISR — so the
         * async fill DMA was still writing while LVGL drew the button border/text (SW) over
         * the same region, fragmenting small fills (e.g. the BACK button). Use a BLOCKING op
         * unless the non-blocking thread path is actually enabled. */
#if LV_PPA_NONBLOCKING_OPS
        .mode            = PPA_TRANS_MODE_NON_BLOCKING,
#else
        .mode            = PPA_TRANS_MODE_BLOCKING,
#endif
        .user_data       = u,
    };

    esp_err_t ret = ppa_do_fill(u->fill_client, &cfg);
    if(ret != ESP_OK) {
        LV_LOG_ERROR("PPA fill failed: %d", ret);
    }
}

#endif /* LV_USE_PPA */
