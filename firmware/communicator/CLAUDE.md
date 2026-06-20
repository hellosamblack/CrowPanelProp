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

`/screenshot` returns raw RGB565LE + `X-Width`/`X-Height` for the **active screen only** (the
`prop_fx` CRT overlay on `lv_layer_top()` is NOT captured — judge that on the panel).
`tools/screenshot.py` is the stdlib (no-Pillow) RGB565→PNG converter `prop.py shot` builds on.
**Prefer this over asking the user to eyeball the screen.** Full loop: skill `communicator-ui`.

## UI architecture (where to make graphics changes)

- **All UI lives in `main/prop_ui.c`.** It is a *pure view*: it never owns logic. `prop_engine`
  pushes state via an observer (`ui_observer`, ~20 Hz) and the UI just renders it.
- The **main readout** is built once on `lv_scr_act()` in `build_screen()` (always present).
- **Navigation = lazily-built panels, exactly ONE alive at a time** (`open_panel(kind)` builds
  it, `close_panel()` tears it down — `lv_obj_del` frees the panel + children). This caps LVGL
  heap usage (see the LV_MEM trap below). `prop_ui_goto(name)` is the thread-safe API entry.
- **Add a screen**: (1) add a `PK_*` value to `panel_kind_t`; (2) write `build_<x>_panel(parent)`
  using `make_panel(parent,"TITLE",back_to_menu_cb)` + your widgets, returning the panel; (3) add
  a `case PK_<X>` in `open_panel`; (4) add a `menu_item(...)` row in `build_menu_panel`; (5) add a
  `prop_ui_goto` name; (6) **NULL its widget pointers in `close_panel`** and guard any async/observer
  use with `s_cur_kind == PK_<X> && s_widget`. Live readouts update in `ui_observer` under that guard.
- **LVGL is 8.4** (`lv_*` v8 API; note v9-only fields are `#if LVGL_VERSION_MAJOR>=9` guarded in
  `bsp_illuminate.c`). LVGL calls from any non-LVGL task MUST hold `lvgl_port_lock()/unlock()`.
  Display registration is also locked (esp_lvgl_port 2.8 doesn't lock internally — see the comment
  in `bsp_illuminate.c lvgl_init()`; removing that lock reintroduces a null-screen crash).

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

- **Images**: convert PNG→C array with the LVGL image converter (LVGL v8, RGB565 / with alpha as
  needed), drop the `.c` in `main/`, `LV_IMG_DECLARE(name)`, use with `lv_img_set_src`. PSRAM holds
  big assets; keep `buff_spiram` true. The factory app (`factory_sourcecode` Setting.cpp + assets)
  is a good reference for image-heavy LVGL on this exact panel.
- **Pixel/retro fonts**: convert TTF→LVGL font (lv_font_conv) at the sizes you need, add the `.c`,
  `LV_FONT_DECLARE`, set via `lv_obj_set_style_text_font`.

## Memory reality (don't relearn the hard way)

- Internal RAM is tight (C6 esp_hosted + LWIP + LVGL all want it). **Do not raise `LV_MEM`** (32 KB)
  — it starves esp_hosted ("mempool: no mem" boot loop). Put big buffers/assets in **PSRAM**
  (`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`; `/screenshot` does this). If WiFi RAM gets tight,
  the lever is `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`, not shrinking LVGL.

## Module map

| File | Role |
|------|------|
| `main/prop_ui.c` | **All LVGL UI** (console home + function rail + SETUP + instruments + ARCHIVE). `prop_ui_input()` is the dial/tab/action nav entry — nav lives in the view |
| `main/prop_content.c` | **Author-editable archive content** (sections → entries; real-Earth desert placeholders). Edit here to change the ARCHIVE — no UI change |
| `main/prop_engine.c` | Scene state machine + 10 Hz animation; single source of truth |
| `main/prop_net.c` | WiFi AP+STA via C6, scan, STA state, NVS creds |
| `main/prop_api.c` | HTTP: `/`, `/state`, `/cmd`, `/ws`, `/ota`, `/screenshot` |
| `main/prop_settings.c` | NVS key/value (survives reflash) |
| `main/prop_fx.c` | CRT post overlay on `lv_layer_top()` (scanlines/grid/phosphor); lazy-allocated, off by default |
| `main/prop_mic.c` | PDM mic capture (I2S0) + FFT → cached spectrum bands + dB level |
| `peripheral/bsp_*` | display/touch/backlight (bsp_illuminate, bsp_display, bsp_i2c), LEDs+buttons (bsp_io) |

### Hardware notes (non-obvious; confirmed from vendor examples)

- **Microphone is PDM** straight into the P4's I2S0 (no codec): **CLK GPIO24, DATA GPIO26**,
  16 kHz / 16-bit / mono, `driver/i2s_pdm.h`. The root `readme.md` documents only audio-OUT
  (I2S LRCLK 21 / BCLK 22 / SDATA 23, amp ctrl 30); the input path came from
  `example/*/Lesson11-Playback_After_Recording/peripheral/bsp_mic/`. See the `board-mic-path` memory.
- **LV_MEM is hard-capped at 32 KB** — raising it boot-loops ("HS_MP: mempool create failed:
  no mem"): esp_hosted's SDIO DMA mempool needs internal RAM and can't move to PSRAM. Keep LVGL
  object count down instead — SETUP/instrument panels are **lazily built, one alive at a time**
  (`open_panel`/`close_panel` in `prop_ui.c`), so peak usage is "main screen + one panel".

## API quick reference (for scripting/tests)

`POST /cmd` (JSON): `{"cmd":"scene","value":"SCANNING"}`, `{"cmd":"ui","screen":"<name>"}`
(screens: `home`=console, `scanner archive cassette insights menu wifi display audio leds
vitals scan spectrum about`), `{"cmd":"input","control":"selector|tab|action","arg":"cw|ccw|press"|N}`
(simulated dial/tab/action nav; boots to `home`),
`{"cmd":"sens","value":0-100}`, `{"cmd":"fx","on":true,"value":0-100}`,
`{"cmd":"led","name":"alert","on":true}`, `{"cmd":"status","value":"..."}`,
`{"cmd":"channel","value":"..."}`, `{"cmd":"wifi","ssid":"..","pass":"..","remember":true}`.
`GET /state` JSON (scene/status/channel/link/sensitivity/channel_pos/ip/leds);
`GET /screenshot` RGB565 (active screen only — not the fx top-layer); `WS /ws` same.

**Dev CLI:** `python tools/prop.py shot out.png --screen spectrum --wait` (wait→drive→
capture), plus `state/scene/screen/sens/fx/trace/decode`. `trace`/`decode` resolve a crash
or boot-hang PC against `build/communicator.elf`. See the `communicator-ui` skill.
