# Translation: XML idiom ↔ C idiom

How a construct on one side becomes the other. The C side is LVGL 9.4 with the
`prop_kit` helpers (`prop_kit.h`); the XML side is the LVGL Pro schema. Prefer the
kit helpers in C and named `<style>`s in XML — that's how the existing code reads.

## Palette & fonts (single source of truth)

| XML | C |
|---|---|
| `#col_bg` | `COL_BG` |
| `#col_amber` | `COL_AMBER` |
| `#col_mute` | `COL_MUTE` |
| `#col_dim` | `COL_DIM` |
| `#col_alert` | `COL_ALERT` |
| `#col_panel_item` | `COL_PANEL_ITEM` |
| `font_body` (14) | `FONT_BODY` |
| `font_head` (24) | `FONT_HEAD` |
| `font_status` (40) | `FONT_STATUS` |

Values live in `globals.xml <consts>` and `prop_kit.h`. If a sync changes a colour or
adds a font, change **both** — they must stay numerically identical.

## Geometry constants

The content area sits right of a 76 px rail: `RAIL_W 76`, `SCAN_W = 1024-76 = 948`.
Panels are `SCAN_W × 600`. In XML the rail is `<nav_rail/>` and the panel is
`<lv_obj width="948" height="600" align="top_right">`. A C panel from `make_panel()`
is already `SCAN_W × 600`. Keep x/y offsets equal on both sides (header BACK at x=20
y=12 w=130; title `top_mid` y=18; etc.).

## Panel frame & header

- **C**: `make_panel(parent, "TITLE", back_cb)` builds the amber-bordered frame +
  `< BACK` button + centred title.
- **XML**: the `panel` + `btn_back` + `title` named styles + an `<lv_button>` BACK +
  `<lv_label … align="top_mid">`. A BACK button carries
  `<screen_create_event screen="<parent>" trigger="clicked"/>`.

## Kit components (these cover most rows)

| C (`prop_kit.h`) | XML equivalent |
|---|---|
| `kit_body(panel)` | `<column>` (or an `<lv_obj flex_flow="column">`) below the header |
| `kit_list_row(b,"TEXT",cb,ud)` | `<lv_button>` with the `list_row` style + a label + a `screen_create_event` |
| `kit_slider_row(b,"L",min,max,val,cb)` → value label | `<lv_label>`(value) + `<lv_slider min_value max_value>` with `sld_*` part styles |
| `kit_switch_row(b,"L",on,cb,ud)` | `<lv_label>` + `<lv_switch>` with `sw_*` part styles + `checked="true"` |
| `kit_info_row(card,"K","V")` → value label | an `<lv_obj>` row: muted key `left_mid` + amber value `right_mid` |
| `kit_meter_row(b,"L",&fill)` | label + a track `<lv_obj>` containing a fill `<lv_obj>` |
| `kit_card(p,w,h)` | `<lv_obj>` with the `card` style (panel-item bg + ver gradient, dim border) |
| `kit_meter(p,w)` / `kit_set_meter(fill,pct,col)` | a `meter_track` box + a `meter_fill` box sized by % |

## Widget styling

- **C** sets styles imperatively: `lv_obj_set_style_bg_color(o, COL_AMBER, 0)`,
  `..._text_font(o, FONT_HEAD, 0)`, part via `LV_PART_INDICATOR`/`LV_PART_KNOB`.
- **XML** uses named `<style>`s applied with `<style name="x"/>`, or inline
  `style_<prop>="…"`. **Parts** use a selector, never a bundled key:
  `<style name="sld_ind" selector="indicator"/>` or `style_bg_color-indicator="…"`.
  (A single `<style>` is a flat `lv_style_t`; `knob_bg_color` as a key is invalid.)

## Navigation

| C | XML |
|---|---|
| `make_btn(...back_cb)` / `kit_list_row(...menu_open_cb, PK_X)` | child `<screen_create_event screen="screen_x" trigger="clicked"/>` |
| `open_panel(PK_X)` | loading `screen_x` (create-on-enter / delete-on-leave = same lazy model) |
| `back_to_menu_cb` → `PK_MENU` | `screen_create_event screen="screen_setup"` |
| `back_to_instruments_cb` / `back_to_sensors_cb` | `screen_instruments` / `screen_sensors` |
| rail cell click (`rail_click_cb`) | `nav_rail.xml` cell's `screen_create_event` |

The nav graph must match: a BACK in C that returns to `PK_INSTRUMENTS` must, in XML,
`screen_create_event` to `screen_instruments`, and vice-versa.

## Live data: observer ↔ subjects

The XML `subj_*` subjects exist so the collaborator can preview reactive state; in
firmware that state is pushed by `ui_observer`. They line up by meaning:

| XML subject | C source (in `ui_observer` / builders) |
|---|---|
| `subj_status` / `subj_channel` | `st->status` / `st->channel` on the scanner |
| `subj_link_text` / `subj_ip` | `link_text(st->link)` / `prop_net_get_ip` |
| `subj_brightness`/`subj_volume`/`subj_*` (fx) | NVS keys read in the DISPLAY/AUDIO builders |
| `subj_core_temp`/`subj_free_ram`/`subj_cell`/`subj_uptime` | VITALS observer updates |

Adding a `bind_*` in XML does **not** require a C change unless the firmware was also
meant to show that value — note it rather than auto-editing the observer.

## Adding a brand-new screen (XML → C)

If the collaborator added a screen with no C counterpart, follow the procedure in the
repo `CLAUDE.md` ("Add a screen"): add a `PK_*`, write `build_<x>_panel`, add the
`open_panel` case, a `menu_item`/`kit_list_row`, a `prop_ui_goto` name, and NULL its
widget pointers in `close_panel`. Then `idf.py reconfigure` (CMake GLOBs `main/*.c`).

## C → XML direction

Mirror a C builder into a new/edited `screen_*.xml`, then run `check_ui.py`. Follow
`xml-authoring.md` for the viewer-safe rules (they are easy to get wrong and the
online viewer fails hard). Keep the header comment pointing back at the C builder.
