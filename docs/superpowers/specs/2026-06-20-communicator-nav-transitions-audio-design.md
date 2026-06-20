# Communicator Prop — Nav Retool, CRT Transitions, Granular FX & Action Audio

**Date:** 2026-06-20
**Status:** Approved design — ready for implementation plan
**Scope:** `firmware/communicator/` (ESP-IDF 6.0.1, ESP32-P4, LVGL 8.4)
**See also:** `2026-06-19-communicator-prop-roadmap-design.md` (scanner instrument),
`2026-06-19-communicator-archive-addendum.md` (archive reframing / input model).

## Context

The device is Zarrah's Armada multi-data interface: an archive console where the
scanner is one instrument on a function rail. Navigation today is a **vertical text
list** on the console home (`PK_HOME`), with each panel showing a top-left BACK/CONSOLE
button; screen changes are **instant cuts** via `open_panel`/`close_panel` (one panel
alive at a time). The CRT FX layer (`prop_fx.c`) has a **single master on/off + one
global intensity** scaling a baked canvas (scanlines + phosphor wash + vignette) plus a
separate scrolling refresh band. The I2S **speaker amp is wired but unused** by the
firmware; Setup ▸ Audio is a stub.

This spec covers a polish-focused retool in four threads:

1. **Persistent icon left rail** replacing the home text-list as the primary nav.
2. **CRT channel-change transitions** (four flavors) between screens.
3. **Granular per-effect CRT FX control** in Setup ▸ Display.
4. **Synthesized action-feedback audio** over the I2S speaker amp.

## Guiding constraints (carried from existing CLAUDE.md / roadmap)

- **Camera legibility is a requirement.** Never use `COL_DIM` for text the viewer must
  read; the hero instruments (Scanner/Spectrum) must stay uncluttered on camera.
- **LV_MEM is hard-capped at 32 KB** — do not raise it. Big buffers/assets go to PSRAM
  (`MALLOC_CAP_SPIRAM`). Keep LVGL object count down; panels stay lazily built, one
  alive at a time.
- **Never call WiFi/SDIO APIs or block under `lvgl_port_lock()`.** Audio playback and
  any heavy work run off the LVGL task; the UI reads caches.
- **`/screenshot` captures the active screen only** — NOT the `lv_layer_top()` overlay.
  CRT FX and transitions are judged on the physical panel.

---

## 1. Persistent icon left rail

### Behavior

- A fixed **~64 px vertical rail** pinned to the left edge of every screen. Eight
  function cells (~66 px tall): `ARCHIVE · SCANNER · VITALS · SIGNAL SCAN · SPECTRUM ·
  CASSETTE · INSIGHTS · SETUP`. `HOME`/console is the **default content** shown when no
  function panel is open (not a rail cell).
- **Icon-only** cells; the **cursor item's name** renders as a short caption at the rail
  head so the highlighted function is always legible.
- The selector **dial rotates the rail cursor; press opens** the highlighted function —
  replacing the home vertical-list selection. Tab/action inputs behave as today within
  the opened panel.
- **Tint states** (via text color on the icon glyph): cursor = glow/bright
  (`FX_GLOW`/near-white amber), current open function = `COL_AMBER`, idle = `COL_MUTE`,
  hero-collapsed = `COL_DIM`.
- **Hero collapse:** on Scanner and Spectrum only, after ~3 s idle the rail dims to
  `COL_DIM` and narrows to a thin spine, keeping the hero uncluttered on camera. Any
  dial input restores it to full width + amber immediately.

### Architecture

- The rail is a **persistent sibling** on `lv_scr_act()`, built once (like `build_screen`),
  NOT inside the swapped panel. Panels build into a **content container to the rail's
  right**; `open_panel`/`close_panel` keep the one-alive-panel discipline within that
  container. The rail is a small fixed object — negligible LVGL heap.
- Existing `prop_ui_input(control,arg)` is the entry point: `selector` rotate/press drives
  the rail; the GPIO buttons / future encoder route through it unchanged.
- `prop_ui_goto(name)` continues to work and now also moves the rail cursor to match.

### Icons

- A **custom geometric icon set** authored as SVG/TTF, converted with `lv_font_conv`
  into an **icon font** (one glyph per private-use codepoint) at the rail glyph size.
  Follows the existing font-gen recipe in the `communicator-ui` skill (merge the symbol
  range; fix the generated include to plain `"lvgl.h"`).
- Glyph concepts: archive = stacked bars/book, scanner = sine wave, vitals = pulse,
  signal = radiating arcs, spectrum = bar graph, cassette = twin reels, insights =
  node/graph, setup = gear/sliders.
- Cassette reels are a candidate for a later live line-draw (spinning) but ship as a
  static glyph first.

### Touches

`prop_ui.c` (new persistent rail builder + content-container refactor of `open_panel`/
`close_panel`, rail cursor state, hero-collapse timer), new icon-font `.c` in `main/`,
`prop_ui.h` if the goto/input surface changes.

---

## 2. CRT channel-change transitions

### Behavior

Fired on **function/screen change** (nav). A `TRANSITION` setting selects the flavor;
**default = Snow, transitions on** (user-overridable, persisted in NVS). Four flavors + Off:

- **Snow burst (~180 ms):** an opaque noise overlay masks the swap — old screen → snow
  → panel swapped underneath → snow clears → new screen. Noise is a **modest-resolution
  tile scaled/scrolled** via LVGL image-zoom, NOT per-pixel full-res, held to ~3–5
  frames so the brief full-screen recomposite is acceptable.
- **Roll + desync (~200 ms):** content container animates Y (old slides up, new enters
  from bottom) with a thin noise seam band riding the tear, then locks.
- **CRT collapse (~150 ms):** content transforms to a center horizontal line + bright
  flash, swaps, reopens.
- **Snow + collapse (~250–300 ms):** collapse → snow at the line → reopen.

### Architecture

- New code in `prop_fx` (e.g. `prop_fx_transition_run(kind, swap_cb)`): the caller hands
  in the panel-swap as a callback executed at the masked midpoint, so the swap is hidden.
- Transient buffers (noise tile, seam band) allocated from **PSRAM and freed after each
  run** — an idle transition holds no memory.
- Respects the `prop_fx` invalidation lesson: snow is a short burst, not a sustained
  full-screen animation; roll/collapse animate bounded objects.
- **`/screenshot` will not capture transitions** (top layer) — verified on the panel.

### Touches

`prop_fx.c`/`prop_fx.h` (transition runner + noise/seam helpers), `prop_ui.c`
(`open_panel`/`close_panel` route the swap through the transition runner), settings read.

---

## 3. Granular CRT FX

### Behavior

Replace the single global intensity with **per-effect intensity (0 = off)** behind one
master CRT on/off:

- `scanlines`, `phosphor` (wash/glow), `vignette`, `refresh sweep` — each a 0–100 slider.
- Setup ▸ Display becomes a **scrolling panel**: backlight, master CRT toggle, the four
  effect sliders, the TRANSITION dropdown (Off/Snow/Roll/Collapse/Snow+Collapse), and a
  Transitions on/off toggle.

### Architecture

- `prop_fx` gains per-effect state (`scan_pct`, `phosphor_pct`, `vignette_pct`,
  `refresh_pct`). `paint_canvas()` bakes the static canvas using per-effect opacities
  (effect at 0% contributes nothing); the refresh band uses its own opacity. Changing a
  slider re-bakes the (PSRAM) canvas or adjusts the band — cheap, off the hot path.
- **NVS keys per effect** (`fx_scan`, `fx_phosphor`, `fx_vignette`, `fx_refresh`);
  the legacy `fx_intensity` seeds defaults on first run after upgrade.
- Setters: `prop_fx_set_scanlines(pct)` etc., mirroring the existing
  `prop_fx_set_intensity` pattern; old global setter retained as a no-op or convenience.

### Touches

`prop_fx.c`/`prop_fx.h` (per-effect params, bake, setters, NVS), `prop_ui.c`
`build_display_panel` (scrolling layout + new controls), `prop_settings` keys.

---

## 4. Synthesized action audio

### Behavior

Short synthesized feedback tones on interaction and key scene moments, over the I2S
speaker amp. Cassette-futurism bleeps/clicks; no asset files.

- **Event → sound table:** dial tick (1 ms noise click), open/confirm (rising 2-tone
  chirp), back (single low blip), tab switch (click), deny/error (harsh square buzz),
  SIGNAL DETECTED (bright 3-note sting), ALERT (square klaxon), boot chime.
- **Setup ▸ Audio** (today a stub) becomes real: **master volume slider + mute**, both
  persisted in NVS. Mute gates the amp-enable line.

### Architecture

- New **`bsp_audio`** (I2S-std TX, IDF6 `driver/i2s_std.h`): pins **BCLK 22 / LRCLK 21 /
  SDATA 23, amp-enable GPIO30**, 16-bit mono, ~16–22 kHz. Amp enabled on first sound,
  gated by mute.
- New **`prop_audio.c`/`.h`**: a small tone synth (square/sine + noise with a short
  attack/decay envelope) renders PCM per event into a buffer streamed to I2S on a
  **dedicated audio task** fed by a queue. `prop_audio_play(event_id)` is the API —
  non-blocking, **never called under `lvgl_port_lock()`**.
- Event calls wired at `prop_ui_input()` (UI actions) and `prop_engine` scene
  transitions (stings/boot). Volume/mute read from `prop_settings`.
- **De-risk spike first:** before the event table, a single-tone amp test confirms the
  output path (documented in root `readme.md` but unverified in firmware). If the amp is
  unreachable, audio reprioritizes/defers — it does not block threads 1–3.

### Touches

new `peripheral/bsp_audio/`, new `main/prop_audio.{c,h}`, `prop_ui.c`
(`build_audio_panel` real controls + play calls in `prop_ui_input`), `prop_engine.c`
(scene-transition play calls), `prop_settings` keys (`audio_vol`, `audio_mute`),
`main/CMakeLists.txt` / component REQUIRES, root module-map note in firmware CLAUDE.md.

---

## Build order (low-risk → high)

1. **Granular FX** — pure software on an existing module; no new hardware/chrome.
2. **Icon rail** — new persistent chrome + icon font; biggest UI change.
3. **Transitions** — build on the rail's content container.
4. **Audio** — amp-test spike → synth → event table → Audio panel.

## Risks & open items

- **Amp path unverified (thread 4)** — highest uncertainty; de-risked by a tone spike
  before building the table. Does not gate threads 1–3.
- **Snow full-screen recomposite (thread 2)** — must stay a short burst with a scaled
  noise tile, not sustained full-res, to respect the `prop_fx` invalidation budget.
- **Persistent rail vs uncluttered hero** — mitigated by the hero auto-collapse; verify
  on a camera-representative capture that Scanner/Spectrum still read clean.
- **Internal RAM pressure** — transition + audio buffers live in PSRAM; the rail adds
  few LVGL objects.

## Verification (per thread, on device)

```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" -p COM7 flash
python firmware/communicator/tools/prop.py shot home.png --screen home --wait
```

- **FX:** set each effect slider independently; screenshot won't show the overlay —
  confirm on the panel that scanlines/phosphor/vignette/refresh respond and persist.
- **Rail:** capture every screen; confirm rail renders, cursor/open/idle tints, caption,
  and hero auto-collapse on Scanner/Spectrum (camera-representative).
- **Transitions:** select each flavor; confirm on the panel (not screenshot) the masked
  swap looks right and is snappy; confirm Off = instant cut.
- **Audio:** tone-test the amp; then exercise each event (dial/open/back/tab/deny, scene
  stings, boot) and confirm volume/mute persist.
