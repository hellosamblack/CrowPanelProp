# Communicator / Scanner Prop Firmware

Cassette-futurism communicator/scanner prop for the **CrowPanel Advance ESP32-P4 (hardware V1.0)**.
Custom ESP-IDF firmware structured as a generic *prop engine*: stable firmware whose behavior is driven
live over WiFi, so scenes/LEDs/screen text can be cued and iterated without reflashing.

Forked from `example/V1.0/idf-code/Lesson09-LVGL_Lighting_Control`.

## Architecture

| Module | Role |
|--------|------|
| `peripheral/bsp_display`, `bsp_illuminate`, `bsp_i2c` | Display + touch + backlight bring-up (reused from Lesson09) |
| `peripheral/bsp_io` | Discrete on/off LEDs + debounced buttons (`espressif/button`) |
| `main/prop_engine` | **The brain.** Scene state machine + 10 Hz animation task; single source of truth, fans state out to observers |
| `main/prop_net` | WiFi **AP+STA** via the onboard ESP32-C6 (`esp_hosted`); STA credentials in NVS |
| `main/prop_api` | HTTP server: operator web console, REST `/cmd`, WebSocket `/ws`, OTA `/ota` |
| `main/prop_ui` | LVGL amber-on-black retro readout — a pure view of engine state |

Inputs (physical buttons, remote commands) all funnel through `prop_engine`, which drives both the LEDs
and the screen, so they never drift out of sync.

## Scenes

`IDLE`, `SCANNING`, `SIGNAL_ACQUIRED`, `COMMS`, `ALERT` — each defines status text + an animated LED pattern.

## Build & flash

Requires **ESP-IDF ≥ 5.4.2**. Use the VS Code ESP-IDF extension terminal, or `export.ps1` from your IDF install.

```bash
idf.py set-target esp32p4
idf.py build
idf.py -p <PORT> flash monitor     # first flash over USB (UART0); needs CH341 driver
```

The `espressif/button`, display, and WiFi components download automatically on first build
(declared in the `idf_component.yml` files).

## Live control (no reflash)

Find the device IP from the serial log (or use the AP `PROP-COMMS` / pass `scanner99`, then `192.168.4.1`).

```bash
# REST
curl -X POST http://<ip>/cmd -d '{"cmd":"scene","value":"SCANNING"}'
curl -X POST http://<ip>/cmd -d '{"cmd":"led","name":"alert","on":true}'
curl -X POST http://<ip>/cmd -d '{"cmd":"channel","value":"CH 04 / 147.55 MHz"}'
curl      http://<ip>/state

# Operator cue board (scene buttons, live state): open http://<ip>/ in a browser
# WebSocket: ws://<ip>/ws  — send the same JSON commands, receive state broadcasts
```

Command schema: see the top of [main/include/prop_api.h](main/include/prop_api.h).

### WiFi OTA (code changes, still no USB)

**CLI (recommended)** — build + push in one command:

```powershell
pwsh tools/dev.ps1 ota                          # defaults to comm-unit-7.local
pwsh tools/dev.ps1 ota -DeviceHost 172.17.2.167 # explicit IP fallback
```

**Browser** — open `http://comm-unit-7.local/`, scroll to **FIRMWARE UPDATE**, pick the
`.bin` (usually `build/communicator.bin`), click **UPLOAD**. A progress percentage appears
and the page reports "done — rebooting…" when the device is switching over.

**Raw curl** (if you prefer):

```bash
idf.py build
curl -X POST "http://<ip>/ota?token=prop-ota-2024" --data-binary @build/communicator.bin
```

**Rollback safety:** the bootloader is configured with `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`.
A freshly-OTA'd image boots as `PENDING_VERIFY`; `app_main` calls
`esp_ota_mark_app_valid_cancel_rollback()` after the full stack is up. If the new image
crashes before reaching that point the bootloader automatically reverts to the previous
partition on the next reset — no USB reflash required.

**Confirm the update landed:** `GET /state` includes a `"version"` field (from `git
describe`), and the web console state line shows `v<version>`.

Change `PROP_OTA_TOKEN` in [main/include/prop_api.h](main/include/prop_api.h) and the AP password in
[main/include/prop_net.h](main/include/prop_net.h) before any real deployment.

## Wiring — IMPORTANT

GPIO assignments are placeholders in [peripheral/bsp_io/bsp_io.c](peripheral/bsp_io/bsp_io.c). Only **GPIO48**
(the UART1-RX header pin) is confirmed broken-out and free. **Verify every other pin against the V1.0
schematic in `Eagle_SCH&PCB/1.0/` and the connector silkscreen before wiring.** Defaults used:

- LEDs: `power`=GPIO48, `signal`=GPIO47 (UART1-TX), `alert`=GPIO20
- Buttons (active-low, internal pull-up): `mode`=GPIO33 (UART3-IN), `action`=GPIO11

Buttons: `mode` cycles scenes, `action` jumps to SCANNING (edit `on_button()` in `main/main.c`).

## ESP-IDF version

**Builds clean on ESP-IDF v6.0.1** (target esp32p4). The Lesson09/17 examples were written for 5.4.2; this
project was migrated to 6.0. Changes required for 6.0 (all already applied):

- cJSON moved out of IDF core → added `espressif/cjson` dependency; `REQUIRES cjson` (not `json`).
- WiFi stack: `esp_wifi_remote` bumped to **1.x** (0.x referenced the removed `esp_interface.h`); paired with `esp_hosted` 2.12.9.
- `driver` component split → BSP components now require `esp_driver_i2c` / `esp_driver_gpio` / `esp_driver_ledc`.
- esp_lcd DPI API: `lcd_color_rgb_pixel_format_t`→`lcd_color_format_t`, `LCD_COLOR_PIXEL_FORMAT_*`→`LCD_COLOR_FMT_*`, `.pixel_format`→`.in_color_format`/`.out_color_format`, `.flags.use_dma2d` removed, RGB666 enum gone.
- Stricter warnings-as-errors: fixed a header-guard typo in `bsp_i2c.h`.
- `sdkconfig.defaults`: 16 MB flash + `CONFIG_LV_FONT_MONTSERRAT_24`.

## Status

**Compiles successfully** (`communicator.bin` builds). **Not yet flashed/run on hardware** — pending the
CH341 USB-serial driver + the board appearing on a COM port. Then work through the verification checklist
in the project plan.
