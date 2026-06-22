# Viewer-safe XML authoring rules

The online viewer (viewer.lvgl.io) is strict: an unknown attribute aborts the whole
screen render, and the project only loads if `globals.xml` is valid. These rules are
the ones we learned by breaking them — `check_ui.py` enforces the mechanical ones, but
read these when writing or editing a screen by hand. Ground truth is the LVGL editor
docs and the `lvgl_editor` example screens.

## Project shape (don't move these)
- A screen is `ui/screens/screen_<name>.xml`; its name is the filename. The viewer
  auto-discovers them — no manifest. `project.xml` + `globals.xml` must stay at `ui/`.
- A `<screen>` may contain only `<consts>`, `<styles>`, `<view>`. **No `<animations>`,
  no `<api>`** — those are component-only. Timelines/animations live in a
  `<component>` (e.g. a transition component), not a screen.
- The 76 px rail is the shared `<nav_rail/>` component, placed first in `<view>`.
  Don't re-inline a placeholder rail.

## Fonts & glyphs (the "cache not allocated" trap)
- Fonts are `<tiny_ttf as_file="true">` in `globals.xml`. The online viewer **cannot**
  use `<bin>` (it would need C/codegen) — keep tiny_ttf.
- tiny_ttf renders only glyphs the TTF actually contains. Eurostile is a text face
  with **no symbol glyphs** — any `LV_SYMBOL_*`, ▶/■, arrows, or emoji in a label make
  the viewer log `ttf_get_glyph_dsc_cb: cache not allocated`. Use plain ASCII
  (`0x20–0x7F`): `< BACK`, `|<`, `>`, `[]`, `>|`, `SHOW`. The firmware C can still use
  `LV_SYMBOL_*` — those glyphs are merged into its compiled fonts; the XML can't.

## Styles & parts
- A named `<style>` is a flat `lv_style_t`. Apply it to a part with a **selector**, not
  by prefixing the property:
  `<style name="sld_ind" selector="indicator"/>`, `selector="checked|indicator"`.
  Inline equivalent: `style_bg_color-indicator="…"`, `style_text_font-items="…"`.
  Never `indicator_bg_color="…"` / `knob_radius="…"` inside a `<style>`.
- Geometry (`width`/`height`/`align`/`x`/`y`/`flex_flow`) goes as **element
  attributes**, not buried in a style (component roots and flex children don't reliably
  pick up style-sheet geometry — this is what made `nav_rail` collapse).
- There is no per-side border width: use `border_side="right"` + `border_width`, not
  `border_right_width`.

## Widget property names (the ones that bite)
- `lv_slider`/`lv_bar`: **`min_value` / `max_value` / `value`** — never `min`/`max`
  (unknown attrs abort the render; this blanked the AUDIO/DISPLAY previews).
- `lv_dropdown`: `options="A&#10;B&#10;C"` (newline-separated), `selected="N"` or
  `bind_value="subj_x"`.
- `lv_label`: `long_mode="wrap"`, `bind_text="subj_x"`, `bind_text-fmt="%d%%"`.
- `lv_switch`/`lv_checkbox`: `checked="true"`, `bind_checked="subj_x"`; the ON fill is
  the `indicator` part in the `checked` state.
- `lv_textarea`: `placeholder_text="…"`, `one_line="true"`, `password_mode="true"`.
- `lv_line`: `points="(x y) (x y) …"` — parenthesized pairs, not a flat number list.
- Flag/clickable: `clickable="true"` to make a plain `<lv_obj>` tappable (e.g. I/O boxes).

## Subjects & binding (reactive preview)
- Declare in `globals.xml <subjects>`; only `int` and `string` types exist.
- Bind with `bind_text` / `bind_value` / `bind_checked`; conditional styling/flags via
  `bind_state_if_eq` / `bind_flag_if_eq` (child elements with `subject` + `ref_value`).
- Every bound subject must be declared or the screen errors — `check_ui.py` verifies.

## After any XML edit
Run `python3 .claude/skills/communicator-sync/scripts/check_ui.py` from the repo root.
It re-checks well-formedness, nav targets, subjects, fonts, and the smells above.
Green means the project will load and every screen will preview.
