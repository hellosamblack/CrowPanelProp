# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

This is the support repository for the **CrowPanel Advance ESP32-P4 7-inch display** (a 1024×600 IPS capacitive-touch HMI board). It is **not a single application** — it is a collection of hardware design files, example projects, and factory firmware. There is no top-level build; each example is an independent, self-contained project.

The board is an **ESP32-P4** (RISC-V dual-core HP @ up to 400 MHz, 32 MB PSRAM, 16 MB flash) paired with an **ESP32-C6** companion module that provides Wi-Fi/Bluetooth (the P4 has no native radio). Optional plug-in wireless modules: SX1262 (LoRa), nRF24L01.

### This fork: the prop firmware (active development)

The real work in this fork is **`firmware/communicator/`** — a custom ESP-IDF app turning the board
into a cassette-futurism communicator/scanner prop. **It has its own `CLAUDE.md` — read
`firmware/communicator/CLAUDE.md` before working there.** Highlights that differ from the stock examples:

- Builds on **ESP-IDF 6.0.1** (not the examples' 5.4.2); the board is **chip rev v1.3** so config pins
  the pre-v3 silicon line. Activate IDF + build/flash quirks are in that CLAUDE.md and the
  `idf6-migration` memory.
- **Remote visual development:** the firmware serves `GET /screenshot` (live screen as RGB565) and
  accepts `{"cmd":"ui","screen":"..."}` to navigate. Use `firmware/communicator/tools/screenshot.py`
  to grab a PNG and *look at the UI yourself* instead of asking the user to eyeball it.

## Repository layout

- `example/V1.0`, `V1.1`, `V1.2` — example code per hardware revision. **V1.2 is the latest; default to it** unless asked otherwise. Each version directory contains parallel implementations:
  - `idf-code/` — ESP-IDF projects (the primary, most complete set), organized as numbered `LessonNN-*` folders.
  - `Arduino_Code/` — Arduino IDE sketches (`.ino`) for the same lessons.
  - `Micropython/`, `ESPHome/`, `Squareline_Studio/` — alternative toolchains.
  - `Upgrade P4 to C6 firmware/` — flashing the C6 companion radio firmware.
- `factory_firmware/` — pre-compiled binaries to flash a device back to factory state.
- `factory_sourcecode/` — source for the factory firmware (delivered as archives, e.g. a Brookesia phone-launcher `.zip`).
- `Eagle_SCH&PCB/` — Eagle schematic (`.sch`) / PCB (`.brd`) per revision.
- `3D file/` — `.stp` enclosure models. `readme.md` (root) — full pin map and hardware spec.

### Hardware revision differences (matter when reading/writing pin code)

- **V1.1**: C6 module gains 4-wire SDIO; SDIO data lines D0–D3 reordered from IO14/15/16/17 to IO17/16/15/14.
- **V1.2**: improved LoRa stability; wireless socket signal pins IO53/IO54 ⇄ IO27/IO28 swapped relative to V1.1.

When editing pin assignments, confirm which version directory the file lives in — the same logical pin differs across revisions.

## ESP-IDF project structure (the dominant pattern)

Each `LessonNN-*` IDF project follows a consistent layout:

```
LessonNN-Name/
├── CMakeLists.txt        # sets EXTRA_COMPONENT_DIRS "peripheral", project(...)
├── main/
│   ├── main.c            # app entry (app_main)
│   ├── CMakeLists.txt    # GLOB_RECURSE main/*.c, REQUIRES <bsp components>
│   └── idf_component.yml # managed deps (idf >=5.4.2, esp_lcd_ek79007, esp_lvgl_port, lvgl, ...)
├── peripheral/           # board-support-package components, one dir per peripheral
│   └── bsp_<thing>/      #   e.g. bsp_illuminate, bsp_display, bsp_i2c, bsp_sd, bsp_uart, bsp_usb
│       ├── *.c
│       ├── include/
│       └── CMakeLists.txt  # idf_component_register(... INCLUDE_DIRS "include" REQUIRES ...)
└── partitions.csv, sdkconfig, dependencies.lock
```

The `peripheral/` BSP components are the reusable hardware-driver layer; `main/` wires them together. Dependencies come from the ESP Component Registry via `idf_component.yml` (downloaded into `managed_components/` at build time).

- **LVGL version differs by toolchain**: IDF examples pin `lvgl ^8.3.11`; the README's dependency table and some Arduino examples reference LVGL 9.2. Check the actual `idf_component.yml` / `lv_conf.h` in the project you're touching rather than assuming.

## Building and flashing

**Requires ESP-IDF v5.4.2 or higher**, target `esp32p4`. Examples are intended to be opened in VS Code with the Espressif IDF extension (select port → build → flash), but the CLI equivalents from inside a specific lesson directory:

```bash
idf.py set-target esp32p4
idf.py menuconfig          # peripheral/radio options live under BSP config (e.g. CONFIG_BSP_SX1262_ENABLED)
idf.py build
idf.py -p PORT flash monitor
```

To enter download mode: hold **Boot**, press **Reset**. There are no automated tests in this repo (the `pytest_*.py` references in stock IDF lesson READMEs are upstream Espressif boilerplate, not wired up here).

### Arduino examples

Open the `LessonNN-*.ino` in Arduino IDE with the ESP32 board package (board: ESP32P4 Dev Module). Some lessons bundle a `libraries/` subfolder (e.g. `ESP32_Display_Panel`, `ESP32_IO_Expander`, `RadioLib`, `lvgl`, `esp-lib-utils`) that must be installed into the Arduino libraries path. Pin/config lives in per-sketch `board_config.h` / `esp_panel_*conf.h` / `lv_conf.h`.

## Key pin references (from root `readme.md`)

- **Display (MIPI-DSI)**: data IO40/39/36/35, CLK IO37/IO38, REXT IO34. Panel driver EK79007.
- **Touch (GT911, I2C)**: SCL IO46, SDA IO45, INT IO42, RST IO40. I2C addr 0x5D (INT low at reset) or 0x14 (INT high).
- **Audio (I2S out)**: LRCLK IO21, BCLK IO22, SDATA IO23, amp control IO30.
- **Radio SPI**: CLK 8, MISO 7, MOSI 6. SX1262: BUSY 9, IRQ 53, NRST 54, NSS 10. nRF24: IRQ 9, CE 53, CS 54. (IRQ/CE/CS pins differ on V1.2 per the revision note above.)
