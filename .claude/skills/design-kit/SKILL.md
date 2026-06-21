---
name: design-kit
description: Build or redesign communicator prop screens (repo root) the code-first way — compose them from the reusable prop_kit components (cards, key/value rows, themed buttons/fields/meters) and lean on LVGL 9 features (flex/grid layout, gradients, observer). Use when adding a screen or reworking an existing panel's layout. For the build/flash/screenshot loop and house style, see the communicator-ui skill.
---

# Communicator prop — code-first design kit

We design screens **in code, composed from reusable building blocks** (we evaluated
SquareLine Studio and dropped it: it can't represent the procedural screens — FFT spectrum,
scanner waveform, CRT FX, icon rail — and there's no headless authoring). The kit lives in
`main/prop_kit.{h,c}` and is the single source of truth for the **theme** (palette + fonts)
and for **reusable components**. Reach for a kit component before hand-positioning `lv_obj`s.

Build/flash/see loop, house style, and hard-won gotchas: **read the `communicator-ui` skill.**
This skill is *how to compose a screen*.

## The kit (`main/prop_kit.h`)

Theme (include `prop_kit.h` — `prop_ui.c` already does):
- Palette: `COL_BG`, `COL_AMBER` (primary text), `COL_MUTE` (secondary, still on-camera),
  `COL_DIM` (unlit/borders only — never for readable text), `COL_ALERT`, `COL_PANEL_ITEM`.
- Fonts: `FONT_BODY` (14), `FONT_HEAD` (24), `FONT_STATUS` (40), `FONT_PUNCH` (56).

Containers / cards:
- `kit_card(parent, w, h)` → square, dim-bordered card with a subtle phosphor gradient,
  pre-set as a **vertical flex column** with padding + row gap. `LV_SIZE_CONTENT` for h.
- `kit_body(panel)` → transparent full-width flex-column **page body** below the title;
  add rows and they self-stack with a consistent gap (scrolls if they overflow). This is
  the backbone of a converted settings/info panel.

Rows (add these to a `kit_body`/`kit_card` — no manual y math):
- `kit_info_row(parent, key, val)` → muted key / amber value; **returns the value label**.
- `kit_slider_row(body, label, min, max, val, cb)` → "LABEL … VAL%" over a full-width
  slider; **returns the value label** (update it from `cb`, which reads the slider).
- `kit_switch_row(body, label, on, cb, ud)` → "LABEL … [switch]"; returns the switch.
- `kit_meter_row(body, label, &fill)` → "LABEL … VALUE" over a meter (NULL fill = value
  only); returns the value label, `*fill` feeds `kit_set_meter`.
- `kit_list_row(body, text, cb, ud)` → full-width menu row (big amber text); returns it.
- `kit_row(body)` → bare space-between flex row; fill it yourself (e.g. label + dropdown).

Primitives (the kit is the single source of the look; `prop_ui.c`'s `style_btn`/`style_field`/
`make_meter_bar`/`set_meter` are thin wrappers over these):
- `kit_style_btn/field/slider/switch(obj)`, `kit_meter(parent, w)` / `kit_set_meter(fill, pct, col)`,
  `kit_phosphor_grad(obj, c1, c2, dir)`.

Still in `prop_ui.c`: `make_panel` (framed panel + BACK + title), `make_btn`, `panel_label`
(used by the procedural instrument chrome). The old absolute-position `make_slider`/`make_switch`/
`menu_item`/`fx_row`/`vitals_row` are gone — use the flex rows above.

**Converted vs themed:** the row-composable panels (menu, display, audio, leds, vitals, about)
are built from these flex rows. The procedural / form / tabbed-list screens (scanner+home
console, spectrum, signal scan, archive+article, cassette, insights, wifi) keep their bespoke
structure but draw on the kit theme + primitives — don't force flex onto those.

## Recipe: compose / redesign a screen

1. `make_panel(parent, "TITLE", back_to_menu_cb)` for the framed panel + BACK + title.
2. Lay out content with `kit_card` + `kit_info_row` (or flex/grid directly). Prefer flex
   over manual `lv_obj_align` for stacked/listed content; keep absolute align for HUD
   clusters and **bottom-anchored animated meters** (re-`align` after `set_height` each frame).
3. Stash any live-updating widget pointer in a `static` and update it in `ui_observer`
   (guarded by `s_cur_kind == PK_x && ptr`). `kit_info_row` returns that pointer.
4. Wire nav per the `communicator-ui` "add a screen" steps (PK_ enum, `open_panel` case,
   `prop_ui_goto` name, NULL the pointers in `close_panel`).
5. **Verify with the batch loop** (don't eyeball one at a time):
   ```bash
   python tools/gallery.py --out baselines      # before
   #  ...build, flash...
   python tools/gallery.py --out after
   python tools/diff_png.py baselines after      # screens you didn't touch must stay ~0%
   ```
   Then `Read after/<screen>.png` and judge the redesigned one.

## LVGL 9 features to use (enabled in our build)

- **Flex** (`CONFIG_LV_USE_FLEX`) — `lv_obj_set_flex_flow`/`_align`, `LV_PCT`, `LV_SIZE_CONTENT`.
  This is the big maintainability win: stop hand-computing y offsets. (`kit_card`/`kit_info_row`.)
- **Grid** (`CONFIG_LV_USE_GRID`) — for true column alignment across rows (e.g. the SCAN list:
  index / name / bar / dBm). Use when space-between flex isn't enough.
- **Gradients** (`CONFIG_LV_DRAW_SW_COMPLEX`, 2 stops) — `kit_phosphor_grad` / the
  `bg_grad_color`+`bg_grad_dir` style props. Keep tones close so text stays legible. Bump
  `CONFIG_LV_GRADIENT_MAX_STOPS` if you need >2.
- **Observer/Subject** (`CONFIG_LV_USE_OBSERVER`) — `lv_subject_*` + `lv_label_bind_text`,
  `lv_obj_bind_flag_if_*`. Candidate to replace bits of the hand-rolled `ui_observer` plumbing
  for new screens (bind a widget to a subject instead of poking it each tick). Adopt incrementally.

**OFF (don't reach for these):** `LV_USE_VECTOR_GRAPHIC`, `LV_USE_THORVG` (no scalable vector
icons — keep using FontAwesome glyphs from the merged Eurostile fonts), `LV_USE_FLOAT`
(`lv_point_precise_t` is integer here; fine for our lines).

## Extending the kit

When a layout pattern repeats across two+ screens, promote it into `prop_kit.{h,c}` (e.g. a
`kit_section_header`, `kit_list_row`, `kit_stat_tile`). New `main/*.c`/header is GLOBbed but
needs `idf.py reconfigure` once (or you get an `undefined reference`). Re-run the gallery diff
after — a kit change touches every screen that uses it, so confirm the deltas are intended.

## Memory note

LVGL's heap is in PSRAM (`main/lv_port_mem.c`) so object count is no longer a hard ceiling,
but panels are still **lazily built one-at-a-time** (`open_panel`/`close_panel`) to keep the
per-frame render cost down. Heavy procedural screens (spectrum) run ~8 fps; don't pile static
objects onto those. See `communicator-ui` and `CLAUDE.md` for the why.
