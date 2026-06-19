---
name: communicator-ui
description: Iterate on the CrowPanel communicator prop firmware UI (firmware/communicator) — build/flash, see the live screen via mDNS + /screenshot, and keep the cassette-futurism look (Eurostile font, amber palette, camera-legible text). Use for any graphics/LVGL change in firmware/communicator/main/prop_ui.c.
---

# Communicator prop — UI iteration loop

Visual development workflow for `firmware/communicator`. The goal is a tight
build → flash → **look at the screen yourself** → adjust loop, so you never ask a
human to eyeball the panel. Read `firmware/communicator/CLAUDE.md` first for the
deeper architecture; this skill is the *how to iterate* checklist.

## 1. Build + flash (ESP-IDF 6.0.1, not on PATH)

```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" -p COM7 flash
```

## 2. See the screen (no IP hunting — the prop advertises mDNS)

The firmware sets hostname `comm-unit-7` and runs mDNS, so it's reachable at
**`comm-unit-7.local`** without scraping the serial log for an IP.

```bash
# drive to a screen:  home | menu | wifi | display | audio | leds | about
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"ui","screen":"wifi"}'
# capture full screen, or zoom a detail (x,y,w,h) so small text is readable:
python tools/screenshot.py comm-unit-7.local shot.png
python tools/screenshot.py comm-unit-7.local corner.png --crop 764,8,260,86 --zoom 3
```

Then `Read` the PNG and judge it. The board may be idle (boot logs already
scrolled) — poll readiness with `until curl -s -o /dev/null --max-time 4
http://comm-unit-7.local/state; do sleep 2; done` before screenshotting.

## 3. Keep the house style (see top of `main/prop_ui.c`)

- **Font is Eurostile** — `FONT_BODY` (16 px, from `resources/Eurostile-Bold.ttf`)
  and `FONT_HEAD` (24 px, Bold). The screen sets `FONT_BODY` on `lv_scr_act()` so
  everything inherits it. To regenerate or add a size, see section 5.
- **Palette:** `COL_BG`, `COL_AMBER` (primary text), `COL_MUTE` (secondary text
  that still must read on camera), `COL_DIM` (de-emphasised / unlit), `COL_ALERT`.
- **Camera legibility is a hard requirement** — it's a 7" panel filmed on set.
  Never use `COL_DIM` for text the viewer needs to read; use `COL_MUTE` and a bold
  font. Square corners, amber-on-black, no default blue/white widgets.

## 4. Hard-won gotchas (don't relearn these)

- **Don't call WiFi/SDIO APIs under the LVGL lock.** `esp_wifi_sta_get_rssi()` is
  a slow round-trip to the C6; calling it in the `ui_observer` (which holds
  `lvgl_port_lock`) caused ~30 s of display flicker on boot. Poll such values in a
  background task and have the UI read a cached int (`prop_net_get_rssi()`).
- **Prefer absolute `lv_obj_align` / `lv_obj_align_to` over nested flex** for small
  HUD clusters. A flex row holding a custom meter container silently failed to lay
  out / clipped; explicit alignment is what the rest of this UI uses and it works.
- The signal meter is plain `lv_obj` rectangles (same vocabulary as the scan blip),
  re-anchored to the LINK label each frame so it tracks the label's width.

## 5. Eurostile / custom font pipeline

The LVGL `built_in_font_gen.py` helper crashes on Python 3.14 (argparse bug), so
call `lv_font_conv` directly and **merge the FontAwesome symbol range** or
`LV_SYMBOL_*` glyphs (BACK arrow, eye, keyboard keys) render as tofu:

```bash
npm install -g lv_font_conv   # once
SYMS="61441,61448,61451,61452,61452,61453,61457,61459,61461,61465,61468,61473,61478,61479,61480,61502,61507,61512,61515,61516,61517,61521,61522,61523,61524,61543,61544,61550,61552,61553,61556,61559,61560,61561,61563,61587,61589,61636,61637,61639,61641,61664,61671,61674,61683,61724,61732,61787,61931,62016,62017,62018,62019,62020,62087,62099,62212,62189,62810,63426,63650"
FA="managed_components/lvgl__lvgl/scripts/built_in_font/FontAwesome5-Solid+Brands+Regular.woff"
lv_font_conv --no-compress --no-prefilter --bpp 4 --size 16 \
  --font resources/Eurostile-Bold.ttf -r "0x20-0x7F,0xB0,0x2022" \
  --font "$FA" -r "$SYMS" \
  --format lvgl -o main/eurostile_14.c --force-fast-kern-format
```

Then fix the generated include (this project includes `lvgl.h` directly, not
`lvgl/lvgl.h`): replace the `#ifdef LV_LVGL_H_INCLUDE_SIMPLE … #endif` block with a
plain `#include "lvgl.h"`. The output symbol name == the output filename stem
(`eurostile_14`); `main/CMakeLists.txt` GLOB-includes `main/*.c` automatically.

## 6. Housekeeping

Screenshot PNGs are scratch — write them inside `firmware/communicator/` and delete
them when done (or keep them out of commits). Don't commit `*_verify.png`,
`crop*.png`, `full*.png`, etc.
