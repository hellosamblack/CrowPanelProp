---
name: communicator-ui
description: Iterate on the CrowPanel communicator prop firmware UI (repo root) — build/flash, see the live screen via mDNS + /screenshot, and keep the cassette-futurism look (Eurostile font, amber palette, camera-legible text). Use for any graphics/LVGL change in main/prop_ui.c.
---

# Communicator prop — UI iteration loop

Visual development workflow for the repo root. The goal is a tight
build → flash → **look at the screen yourself** → adjust loop, so you never ask a
human to eyeball the panel. Read `AGENTS.md` first for the
deeper architecture; this skill is the *how to iterate* checklist.

## 1. Build + flash (ESP-IDF 6.0.1, not on PATH)

**Windows (PowerShell):**
```powershell
pwsh tools/dev.ps1 bf -Port COM7    # build + flash (one shot)
```
Or raw (activates IDF + forces UTF-8 to dodge the component-manager emoji crash):
```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" build
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash
```

**Linux / Debian (Bash):**
```bash
./tools/dev.sh bf -Port /dev/ttyUSB0   # build + flash (one shot)
```
Or raw:
```bash
. ~/.local/esp/esp-idf/export.sh
idf.py -C . build
idf.py -C . -p /dev/ttyUSB0 flash
```

**Added a new `main/*.c`?** run `dev.ps1 reconfigure` / `dev.sh reconfigure` (or `idf.py reconfigure`) once —
`CMakeLists.txt` GLOBs sources and won't see the new file until you re-glob (you'll get
an `undefined reference` link error otherwise). New driver header → add its component to
`REQUIRES` in `main/CMakeLists.txt` (e.g. `esp_driver_tsens`, `esp_driver_i2s`).

## 2. See + drive the screen — use `tools/prop.py` (the fast loop)

`prop.py` collapses wait → drive UI → screenshot (and crash decoding) into one command.
Reaches the prop at its mDNS name `comm-unit-7.local` (no IP hunting). Run it from
``:

```bash
python tools/prop.py shot out.png --screen spectrum --wait   # wait ready, navigate, settle, capture
python tools/prop.py shot out.png --scene SIGNAL_ACQUIRED     # set a scene then capture
python tools/prop.py shot corner.png --crop 764,8,260,90 --zoom 3   # zoom a detail
python tools/prop.py state                                    # GET /state (pretty JSON)
python tools/prop.py scene ALERT      # screens: home menu wifi display audio leds
python tools/prop.py screen vitals    #          vitals scan spectrum about
python tools/prop.py sens 80          # receiver sensitivity (scales the waveform)
python tools/prop.py fx on 70         # CRT overlay on @ intensity; `fx off` to disable
```

Then `Read` the PNG and judge it yourself. (`tools/screenshot.py` is still the bare
RGB565→PNG converter that `prop.py shot` builds on.)

**Batch loop — capture/compare ALL screens at once** (use this for kit refactors):

```bash
python tools/gallery.py --out baselines     # snapshot every screen -> baselines/<screen>.png
#   ...refactor / build / flash...
python tools/gallery.py --out after
python tools/diff_png.py baselines after     # per-screen "% changed  maxΔ" regression gate
python tools/diff_png.py baselines/scan.png after/scan.png d.png   # + amber heatmap of what moved
```

A screen you *didn't* touch showing a big % is the red flag; a few % from AA/animation is normal.

**`/screenshot` now reads the MIPI-DPI framebuffer directly** (LVGL 9's `lv_snapshot`
deadlocks under the port lock), so a capture shows the **whole panel INCLUDING the `prop_fx`
CRT overlay** — scanlines/vignette appear in captures now. Reads are cache-invalidated (live).

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

- **LVGL's heap lives in PSRAM** (custom allocator `main/lv_port_mem.c`, selected by
  `CONFIG_LV_USE_CUSTOM_MALLOC`). Do NOT switch back to the builtin internal pool / raise
  `LV_MEM`: esp_hosted's SDIO DMA mempool needs that internal RAM and boot-loops
  (`HS_MP: mempool create failed: no mem`; `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` does not rescue it).
  Why PSRAM: v9 allocates line/shape AA mask buffers via `lv_malloc` *while drawing*, and the
  old 32 KB pool exhausted on instrument screens → NULL → store-fault in `draw_line_skew`.
  Cost: ~8 fps on the heavy spectrum screen (vs ~18 static); a hybrid allocator (small allocs
  internal, big draw masks PSRAM) is the tuning lever if you need the frames back.
- **SETUP/instrument panels are lazily built, ONE alive at a time** (`open_panel` /
  `close_panel` in `prop_ui.c`). Peak LVGL usage = "main screen + one panel", so adding
  new screens is free. To add one: add a `PK_*` enum, a `build_*_panel()` builder, a
  case in `open_panel`, a `menu_item` row, a `prop_ui_goto` name, and NULL its widget
  pointers in `close_panel`. Async users (scan tasks, the observer) must guard on
  `s_cur_kind == PK_x && s_widget != NULL` since the panel can be torn down underneath.
- **Scaling label text via a transform is unreliable** (it rasterises at the base size then
  scales the bitmap — fuzzy/clipped). For a punch-in/headline, **swap to a bigger pre-generated
  font** (`FONT_STATUS`/`FONT_PUNCH` = `eurostile_40`/`eurostile_56`), not a transform.
- **LVGL's `lv_label_set_text_fmt` has NO `%f`** (prints a literal `f`). Format floats
  with stdio `snprintf` into a buffer, then `lv_label_set_text`.
- **Don't render synchronously on a command flood.** A dragged web slider fires many
  `{"cmd":"sens"}`/s; rendering each (full observer pass + I2C LED write) saturated LVGL
  and froze the panel. Setter just updates a cached value; the ~20 fps animate loop
  repaints. LED I2C writes are gated to *changes* only (`publish_locked`). The web slider
  is also throttled (~20/s) in the console JS.
- **Don't call WiFi/SDIO APIs under the LVGL lock.** `esp_wifi_sta_get_rssi()` is a slow
  C6 round-trip; in `ui_observer` (holds `lvgl_port_lock`) it caused ~30 s of boot
  flicker. Poll in a background task; the UI reads a cached int (`prop_net_get_rssi()`).
- **Prefer absolute `lv_obj_align`/`lv_obj_align_to` over nested flex** for HUD clusters
  and meters. Bottom-anchored bars (spectrum/SENS) need a re-`align` *after* `set_height`
  each frame so the bottom stays put while the top grows.

## 4b. Decode a crash / boot hang

`MCAUSE 0xdeadc0de` = task watchdog (a task stuck, often a `while(1)` from
`LV_ASSERT_*`); a real exception shows `Guru Meditation`. Either way, resolve the PC:

```bash
python tools/prop.py trace --seconds 12      # capture serial, auto-flag + addr2line decode
python tools/prop.py decode 0x40034286 0x40034206   # decode specific PCs vs build/communicator.elf
```

`trace` opens the serial port (which **resets the board** via DTR/RTS → you capture a fresh
boot), prints lines matching mempool/assert/WDT/Guru, and addr2line-decodes the code
addresses. Note PCs are only meaningful against the **current** ELF (they shift every build).

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
(`eurostile_14`); `main/CMakeLists.txt` GLOB-includes `main/*.c` automatically (but run
`idf.py reconfigure` so a brand-new `.c` is picked up).

**Headline fonts** (`eurostile_40` resting status, `eurostile_56` punch-in) are
**ASCII-only** — they render plain text, so skip the FontAwesome merge and just use
`-r "0x20-0x7F"`. Sizes in use: `FONT_BODY` 14/16, `FONT_HEAD` 24, `FONT_STATUS` 40,
`FONT_PUNCH` 56. The TTFs live at the **repo root** `resources/` (`../../resources/...`
from the project dir), not under ``.

## 7. API quick reference (for scripting/tests)

`POST /cmd` (JSON): `{"cmd":"scene","value":"SCANNING"}`, `{"cmd":"ui","screen":"<name>"}`
(screens: `home`=console, `scanner archive cassette insights menu wifi display audio leds
vitals scan spectrum rfband ble csi instruments sensors dircal minimap range about`; `instruments`/`sensors`
are the rail submenus, `dircal` is the SCANNER travel-direction calibration (opened
by tapping the apex operator dot), the rest deep-link straight to a panel),
`{"cmd":"input","control":"selector|tab|action","arg":"cw|ccw|press"|N}`
(simulated dial/tab/action nav; boots to `home`),
`{"cmd":"sens","value":0-100}`, `{"cmd":"fx","on":true,"value":0-100}` (CRT overlay on/off + intensity),
`{"cmd":"fx","fps":true}` (toggle FPS HUD — separate from the overlay),
`{"cmd":"led","name":"alert","on":true}`, `{"cmd":"status","value":"..."}`,
`{"cmd":"channel","value":"..."}`, `{"cmd":"wifi","ssid":"..","pass":"..","remember":true}`.
`GET /state` JSON (scene/status/channel/link/sensitivity/channel_pos/ip/version/leds,
plus `ble:{count,strongest,known}` when BLE is up, `csi_live` bool, and
`ftm:{tracked,capable,ranged}` when FTM ranging is up);
`GET /telemetry` JSON — richer sibling of `/state` for live sensor/instrument values
(`screen`, `imu:{yaw_deg,pitch_deg,roll_deg,accel,gyro,temp_c,steps}`, `radar:[{x_mm,y_mm,speed_mm_s}]`,
`track:{x,y,heading_deg}`, `mic:{db,bands}`, `aux_radar:{seeed,sen0395}`, `ble`, `csi`) — see
"2. See + drive the screen" above;
`GET /screenshot` RGB565 read from the DPI framebuffer (whole panel incl. the fx overlay); `WS /ws`
pushes both `/state` (`type:"state"`, on change) and `/telemetry` (`type:"telemetry"`, ~5 Hz) payloads.

**Dev CLI:** `python tools/prop.py shot out.png --screen spectrum --wait` (wait→drive→
capture), plus `state/telemetry/watch/scene/screen/sens/fx/trace/decode`. `trace`/`decode` resolve a
crash or boot-hang PC against `build/communicator.elf`; `coredump-info` (see the `crash-forensics`
skill) decodes a persisted panic without needing serial capture at all.

## 8. Housekeeping

Screenshot PNGs are scratch — write them inside `` and delete
them when done (or keep them out of commits). Don't commit `*_verify.png`,
`crop*.png`, `full*.png`, etc.
