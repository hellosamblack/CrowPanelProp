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

```powershell
# 1) drive the UI to any screen:  home | menu | wifi | display | audio | leds | about
Invoke-RestMethod -Uri "http://comm-unit-7.local/cmd" -Method Post -Body '{"cmd":"ui","screen":"wifi"}'
# 2) capture it to a PNG you can open/Read (add --crop X,Y,W,H --zoom N for small details):
python tools/screenshot.py comm-unit-7.local wifi_page.png
python tools/screenshot.py comm-unit-7.local corner.png --crop 764,8,260,86 --zoom 3
```

`/screenshot` returns raw RGB565LE + `X-Width`/`X-Height`; `tools/screenshot.py` converts to PNG
with the stdlib (no Pillow) and can crop/zoom a region. **Prefer this over asking the user to
eyeball the screen.** For the full iteration loop see the project skill `communicator-ui`.

## UI architecture (where to make graphics changes)

- **All UI lives in `main/prop_ui.c`.** It is a *pure view*: it never owns logic. `prop_engine`
  pushes state via an observer (`ui_observer`, ~10 Hz) and the UI just renders it.
- Everything is built on `lv_scr_act()` in `build_screen()`. The main readout is built inline;
  the SETUP flow is built by `build_setup_screens()`.
- **Navigation** = full-screen overlay panels, one shown at a time via `show_only(panel)`.
  Panels are registered (`register_panel`) so `show_only` can hide the rest. `prop_ui_goto(name)`
  is the thread-safe entry the API uses.
- **Add a screen**: `make_panel(parent, "TITLE", back_cb)` gives you a themed, registered panel
  with a BACK button + centered title; add your widgets; add a `menu_item(...)` row in
  `build_setup_screens()`. Replace a `build_stub(...)` call to make a stub real.
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
| `main/prop_ui.c` | **All LVGL UI** (main readout + SETUP menu + sub-screens) |
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

`POST /cmd` (JSON): `{"cmd":"scene","value":"SCANNING"}`, `{"cmd":"ui","screen":"wifi"}`,
`{"cmd":"led","name":"alert","on":true}`, `{"cmd":"status","value":"..."}`,
`{"cmd":"channel","value":"..."}`, `{"cmd":"wifi","ssid":"..","pass":"..","remember":true}`.
`GET /state` JSON; `GET /screenshot` RGB565; `WS /ws` same commands + state broadcasts.
