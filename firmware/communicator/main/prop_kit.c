/* prop_kit — reusable cassette-futurism components. See prop_kit.h. */
#include "prop_kit.h"

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
