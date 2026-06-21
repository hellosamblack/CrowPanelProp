# CLAUDE.md — communicator prop firmware

Guidance for working on this firmware (the cassette-futurism communicator/scanner prop).
Read the root `../../CLAUDE.md` for board/repo context. This file is about *developing here*,
with an emphasis on **graphics/UI work**.

## Build / flash / monitor (this machine)

ESP-IDF **6.0.1** is installed via EIM. It is NOT on PATH; activate it per command, and
force UTF-8 (the component manager prints an emoji that crashes cp1252):

```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" -p COM7 flash
```

Or use the helper: `pwsh tools/dev.ps1 bf -Port COM7` (build+flash). Board enumerates on **COM7**
(CH341 driver). Target is `esp32p4`; the board is **chip rev v1.3** and config already pins the
pre-v3 silicon line — don't touch `CONFIG_ESP32P4_*REV*` or it won't boot. See `../../` memory
`idf6-migration` for the full list of 6.0/board quirks (cJSON, driver split, esp_lcd API, C6 SDIO pins).

## Seeing the UI WITHOUT asking a human (use this constantly for graphics)

The firmware serves a live screen capture. The board advertises **mDNS**, so reach it at
**`comm-unit-7.local`** (hostname `PROP_HOSTNAME`) instead of hunting the IP — no serial grep
needed. (STA IP still shows in the log, e.g. `172.17.2.167`; AP fallback is `192.168.4.1`.)

```bash
# tools/prop.py is the fast loop: wait-ready -> navigate -> settle -> capture, in one go.
python tools/prop.py shot wifi.png --screen wifi --wait      # screens: home menu wifi
python tools/prop.py shot sig.png  --scene SIGNAL_ACQUIRED   #   display audio leds vitals
python tools/prop.py shot c.png    --crop 764,8,260,86 --zoom 3   #   scan spectrum about
```

`/screenshot` returns raw RGB565LE + `X-Width`/`X-Height` read straight from the MIPI-DPI
framebuffer, so it captures the **whole panel including the `prop_fx` CRT overlay** (LVGL 9's
`lv_snapshot` deadlocks under the port lock, so `prop_api.c` reads the FB directly with a
cache-invalidate). `tools/screenshot.py` is the stdlib (no-Pillow) RGB565→PNG converter `prop.py shot` builds on.
**Prefer this over asking the user to eyeball the screen.** Full loop: skill `communicator-ui`.

## UI architecture (where to make graphics changes)

- **All UI lives in `main/prop_ui.c`.** It is a *pure view*: it never owns logic. `prop_engine`
  pushes state via an observer (`ui_observer`, ~20 Hz) and the UI just renders it.
- The **main readout** is built once on `lv_scr_act()` in `build_screen()` (always present).
- **Navigation = lazily-built panels, exactly ONE alive at a time** (`open_panel(kind)` builds
  it, `close_panel()` tears it down — `lv_obj_del` frees the panel + children). This keeps the live
  object count and per-frame render cost down (LVGL's heap is in PSRAM now — see Memory reality).
  `prop_ui_goto(name)` is the thread-safe API entry.
- **Add a screen**: (1) add a `PK_*` value to `panel_kind_t`; (2) write `build_<x>_panel(parent)`
  using `make_panel(parent,"TITLE",back_to_menu_cb)` + your widgets, returning the panel; (3) add
  a `case PK_<X>` in `open_panel`; (4) add a `menu_item(...)` row in `build_menu_panel`; (5) add a
  `prop_ui_goto` name; (6) **NULL its widget pointers in `close_panel`** and guard any async/observer
  use with `s_cur_kind == PK_<X> && s_widget`. Live readouts update in `ui_observer` under that guard.
- **LVGL is 9.4** (`lv_*` v9 API; `esp_lvgl_port` 2.8 compiles its lvgl9 path — needs lvgl >=9.3 for
  `LV_COLOR_FORMAT_RGB565_SWAPPED`). Most v8 names still work via the unconditionally-included
  `lv_api_map_v8.h` shim, but structural things changed: **`lv_point_precise_t`** for line points,
  **no `lv_color_t.full`** (use `lv_color_eq`/`lv_color_to_u16`), **`lv_canvas_draw_rect` removed**,
  display frame monitor is event-based (`LV_EVENT_RENDER_READY`). LVGL calls from any non-LVGL task
  MUST hold `lvgl_port_lock()/unlock()` — but do NOT run the draw pipeline under that lock from a
  non-LVGL task: canvas layer-draw and `lv_snapshot` deadlock (that's why `prop_fx` paints pixels
  directly and `/screenshot` reads the FB). Display registration is also locked (esp_lvgl_port 2.8
  doesn't lock internally — see the comment in `bsp_illuminate.c`; removing it reintroduces a
  null-screen crash). **The panel takes native RGB565 — keep `swap_bytes=false`** in the v9 display
  cfg (true renders the near-black background as dark magenta over the whole screen).

## Cassette-futurism style kit (keep new UI consistent)

Palette + helpers are at the top of `prop_ui.c`:

- `COL_BG` (0x0A0A06), `COL_AMBER` (0xE0B000), `COL_MUTE` (0xB58A00), `COL_DIM` (0x6B5300),
  `COL_ALERT` (0xFF3030), `COL_PANEL_ITEM` (0x141008).
- **Camera legibility is a requirement** (7" panel filmed on set): never use `COL_DIM` for text the
  viewer must read — use `COL_MUTE` + a bold font. `COL_DIM` is for unlit/de-emphasised elements.
- `style_btn()`, `style_field()` (textarea/dropdown), `style_keyboard()` — apply these to new
  widgets so they match (square corners, amber-on-black). `make_btn()` builds a themed button.
- **Font is Eurostile** (cassette-futurism), not montserrat: `FONT_BODY` (`eurostile_14`, 16 px
  Bold) and `FONT_HEAD` (`eurostile_24`, 24 px Bold), generated from `resources/Eurostile-*.ttf`.
  `build_screen()` sets `FONT_BODY` on the screen so all widgets inherit it. To regen/add a size,
  use the `lv_font_conv` recipe in the `communicator-ui` skill (must merge the FontAwesome symbol
  range or `LV_SYMBOL_*` glyphs vanish; fix the generated `#include` to plain `"lvgl.h"`).
- Default LVGL widgets render blue/white — always theme indicators (e.g. checkbox `LV_PART_INDICATOR`)
  or they clash.
- **Never call WiFi/SDIO APIs (e.g. `esp_wifi_sta_get_rssi`) under the LVGL lock** — it stalls the
  panel (caused ~30 s boot flicker). Cache such values from a background task; the UI reads the cache.

## Adding images / custom fonts

- **Images**: convert PNG→C array with the LVGL image converter (LVGL v9, RGB565 / with alpha as
  needed; v9 image headers carry an explicit `stride`), drop the `.c` in `main/`, `LV_IMAGE_DECLARE(name)`,
  use with `lv_image_set_src`. PSRAM holds big assets; keep `buff_spiram` true. The factory app
  (`factory_sourcecode` Setting.cpp + assets) is a good reference for image-heavy LVGL on this exact panel.
- **Pixel/retro fonts**: convert TTF→LVGL font (lv_font_conv) at the sizes you need, add the `.c`,
  `LV_FONT_DECLARE`, set via `lv_obj_set_style_text_font`.

## Memory reality (don't relearn the hard way)

- **LVGL's heap is routed entirely to PSRAM** via a custom allocator (`main/lv_port_mem.c`,
  `CONFIG_LV_USE_CUSTOM_MALLOC`). LVGL 9 allocates line/shape anti-alias masks via `lv_malloc`
  *during drawing*, and the old 32 KB builtin internal pool exhausted on instrument screens → NULL →
  store-fault in `draw_line_skew`. PSRAM removes that ceiling and leaves internal RAM for esp_hosted.
  **Cost:** ~8 fps on the heavy spectrum screen (vs ~18 static) — a hybrid allocator (small allocs
  internal, big draw masks PSRAM) is the tuning lever if framerate matters.
- **Do not switch back to the builtin pool / raise `LV_MEM`** — it starves esp_hosted ("mempool: no
  mem" boot loop). Other big buffers/assets also go in **PSRAM** (`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`).
  If WiFi RAM gets tight, the lever is `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`.

## Module map

| File | Role |
|------|------|
| `main/prop_ui.c` | **All LVGL UI** (console home + function rail + SETUP + instruments + ARCHIVE). `prop_ui_input()` is the dial/tab/action nav entry — nav lives in the view |
| `main/prop_content.c` | **Author-editable archive content** (sections → entries; real-Earth desert placeholders). Edit here to change the ARCHIVE — no UI change |
| `main/prop_engine.c` | Scene state machine + 10 Hz animation; single source of truth |
| `main/prop_net.c` | WiFi AP+STA via C6, scan, STA state, NVS creds |
| `main/prop_api.c` | HTTP: `/`, `/state`, `/cmd`, `/ws`, `/ota`, `/screenshot` |
| `main/prop_settings.c` | NVS key/value (survives reflash) |
| `main/prop_fx.c` | CRT post overlay on `lv_layer_top()` (scanlines/vignette/phosphor + refresh band); paints ARGB pixels **directly** into the canvas buffer (v9 canvas layer-draw deadlocks under the lock); lazy-allocated |
| `main/lv_port_mem.c` | Custom LVGL allocator → routes `lv_malloc` to PSRAM (see Memory reality) |
| `main/prop_mic.c` | PDM mic capture (I2S0) + FFT → cached spectrum bands + dB level |
| `peripheral/bsp_*` | display/touch/backlight (bsp_illuminate, bsp_display, bsp_i2c), LEDs+buttons (bsp_io) |

### Hardware notes (non-obvious; confirmed from vendor examples)

- **Microphone is PDM** straight into the P4's I2S0 (no codec): **CLK GPIO24, DATA GPIO26**,
  16 kHz / 16-bit / mono, `driver/i2s_pdm.h`. The root `readme.md` documents only audio-OUT
  (I2S LRCLK 21 / BCLK 22 / SDATA 23, amp ctrl 30); the input path came from
  `example/*/Lesson11-Playback_After_Recording/peripheral/bsp_mic/`. See the `board-mic-path` memory.
- **LVGL heap lives in PSRAM** (custom allocator, see Memory reality) — the old 32 KB builtin pool
  exhausted during v9 line drawing. Don't restore the builtin pool: esp_hosted's SDIO DMA mempool
  needs the internal RAM ("HS_MP: mempool create failed: no mem" boot loop). Panels are still
  **lazily built, one alive at a time** (`open_panel`/`close_panel` in `prop_ui.c`), which keeps the
  object count and per-frame render cost down.

## API quick reference (for scripting/tests)

`POST /cmd` (JSON): `{"cmd":"scene","value":"SCANNING"}`, `{"cmd":"ui","screen":"<name>"}`
(screens: `home`=console, `scanner archive cassette insights menu wifi display audio leds
vitals scan spectrum about`), `{"cmd":"input","control":"selector|tab|action","arg":"cw|ccw|press"|N}`
(simulated dial/tab/action nav; boots to `home`),
`{"cmd":"sens","value":0-100}`, `{"cmd":"fx","on":true,"value":0-100}`,
`{"cmd":"led","name":"alert","on":true}`, `{"cmd":"status","value":"..."}`,
`{"cmd":"channel","value":"..."}`, `{"cmd":"wifi","ssid":"..","pass":"..","remember":true}`.
`GET /state` JSON (scene/status/channel/link/sensitivity/channel_pos/ip/leds);
`GET /screenshot` RGB565 read from the DPI framebuffer (whole panel incl. the fx overlay); `WS /ws` same.

**Dev CLI:** `python tools/prop.py shot out.png --screen spectrum --wait` (wait→drive→
capture), plus `state/scene/screen/sens/fx/trace/decode`. `trace`/`decode` resolve a crash
or boot-hang PC against `build/communicator.elf`. See the `communicator-ui` skill.
