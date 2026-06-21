# Communicator render cost model (read before reviewing)

This is what's already known and measured about framerate on *this* firmware/board. Treat it as
ground truth so you don't re-derive it or recommend things already proven not to work. Numbers come
from in-code comments (`prop_ui.c` FPS-meter block ~line 2908), the firmware `CLAUDE.md`
("Memory reality"), and `bsp_illuminate.c` / `sdkconfig`.

## The dominant fact: software rendering

- Panel: **1024×600, RGB565 (2 bytes/px)**, MIPI-DSI (EK79007), 2 lanes @ 900 Mbps, ~51 MHz DPI.
- **No GPU / no PPA** — the PPA 2D path breaks on chip **rev v1.3**, so all compositing is the CPU's
  software renderer (`CONFIG_LV_USE_DRAW_SW=y`, `LV_DRAW_SW_DRAW_UNIT_CNT=1`, no ASM accel).
- Measured: a **whole-screen 1024×600 software render ≈ 250 ms (~4 fps)** and saturates the CPU.
  Therefore framerate is a function of *invalidated area*, not the refresh cap.
- Observed FPS: heavy spectrum screen ~**8 fps**, mostly-static screens ~**18 fps**. The gap is
  dominated by per-frame PSRAM allocation during drawing (below), not by raw pixel count.

## Current configuration (already tuned — verify, don't churn)

| Knob | Value | Source |
|---|---|---|
| CPU freq | 360 MHz (dual HP core) | `sdkconfig` `ESP_DEFAULT_CPU_FREQ_MHZ=360` |
| PSRAM | 200 MHz, HEX/octal | `sdkconfig` `SPIRAM_SPEED_200M`, `SPIRAM_MODE_HEX` |
| Compiler | `-O2` (performance) | `sdkconfig` `COMPILER_OPTIMIZATION_PERF` |
| FreeRTOS tick | 1000 Hz | `sdkconfig` `FREERTOS_HZ=1000` |
| Draw buffers | full-screen, **double**, **in PSRAM** | `bsp_illuminate.c` `double_buffer=true`, `buff_spiram=true` |
| Refresh mode | partial (not full_refresh, not direct_mode) | `bsp_illuminate.c` |
| Tearing | `avoid_tearing=false` | `bsp_illuminate.c` |
| LVGL color | RGB565, `swap_bytes=false` | `bsp_illuminate.c` |
| LVGL heap | **entirely PSRAM**, custom allocator | `lv_port_mem.c`, `CONFIG_LV_USE_CUSTOM_MALLOC` |
| LVGL port task | prio `configMAX_PRIORITIES-4`, **affinity -1**, 16 KB stack, 5 ms tick, 10 ms max sleep | `bsp_illuminate.c` |
| Refresh period | 30 ms default / 8 ms while the FPS HUD is on | `prop_ui.c` |
| Engine/observer | ~20 Hz animate tick (`ANIM_PERIOD_MS=50`), prio 5, unpinned | `prop_engine.c` |

The first four rows are near the ceiling — there's little to gain from clock/optimization knobs.
Real wins are in **area, memory location, contention, and per-frame widget work.**

## The named tuning lever (highest-value, not yet done)

CLAUDE.md "Memory reality": LVGL 9 allocates line/shape **anti-alias masks via `lv_malloc` during
drawing**. The heap is in PSRAM, so every draw pays PSRAM allocation latency — this is *the* reason
spectrum is ~8 fps vs ~18 static. The documented fix:

> a hybrid allocator (small allocs internal, big draw masks PSRAM) is the tuning lever if framerate matters.

Implement in `lv_port_mem.c`: route small/short-lived allocations to internal SRAM
(`MALLOC_CAP_INTERNAL`) with a PSRAM fallback for large buffers. Keep enough internal RAM headroom
for esp_hosted (see constraint below). This is usually the single biggest throughput win available.

## Untried experiments worth proposing (with measurement)

- **Partial draw buffers in internal SRAM.** Full-screen ×2 (≈2.4 MB) can't fit internal RAM, which
  is why `buff_spiram=true`. But a *partial* N-line buffer in internal RAM can, and rendering into
  internal SRAM is far faster than into PSRAM. Worth A/B-ing for the redraw sizes this UI produces.
- **Second SW draw unit** (`LV_DRAW_SW_DRAW_UNIT_CNT=2`) to use both HP cores for rendering — only
  pays off if the second core isn't already saturated by radio tasks, so it's coupled to the
  core-affinity decision. Measure together.
- **Core pinning** LVGL ↔ radio tasks (see below) — primarily a *consistency* win.

## Dead ends — already tried, lifted nothing (do NOT re-propose)

- **Dropping the refresh period / forcing faster refresh.** Tried. Lifted nothing and made the device
  network-sluggish. Framerate is bound by the full-screen double-buffered pipeline + Wi-Fi CPU
  contention, **not** the refresh cap. The FPS HUD already lowers the period to 8 ms as far as it
  helps; it is a *passive* counter on purpose.
- **Forcing a full-screen redraw every frame** to "smooth" animation — counter-productive (~4 fps).
- **Translucent object on `lv_layer_top()`** — makes LVGL recomposite the whole layer each frame;
  thrashed `lv_mem_buf_get` into a watchdog hang. (Why the FPS HUD is an opaque child, and why
  `prop_fx` paints pixels directly.)

## Hard constraints (a fix that breaks one is a regression)

- **Don't restore the LVGL builtin pool / raise `LV_MEM`.** Internal RAM is needed by esp_hosted's
  SDIO DMA mempool; starving it → "HS_MP: mempool create failed: no mem" boot loop. Any hybrid
  allocator must leave internal RAM headroom (there's ~332 KB internal free at runtime with all
  radio instruments live — that's the budget to stay well under).
- **Don't touch `CONFIG_ESP32P4_*REV*`** — board is rev v1.3.
- **Keep `swap_bytes=false`** — native RGB565; `true` paints the background dark magenta.
- **No PPA/GPU rotation path** — breaks on rev v1.3.
- **Never call WiFi/SDIO under the LVGL lock**, and never run canvas layer-draw / `lv_snapshot` from a
  non-LVGL task under the lock (deadlock).
- If WiFi RAM gets tight from any change, the intended lever is
  `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` — not shrinking internal reserves.

## Concurrency map (for the contention review)

All unpinned today (no core affinity) — this is the main consistency liability:

| Task | File | Prio | Role / rate |
|---|---|---|---|
| LVGL port | `bsp_illuminate.c` | MAX-4 | render + input, 5 ms tick |
| engine/observer | `prop_engine.c` | 5 | 20 Hz state + UI fan-out |
| rssi poll | `prop_net.c` | 3 | ~1 Hz `esp_wifi_sta_get_ap_info` (SDIO) |
| ble prune | `prop_ble.c` | 3 | ~2 Hz |
| NimBLE host | `prop_ble.c` | (nimble) | event-driven BLE adverts (SDIO/VHCI) |
| CSI fold | `prop_csi.c` | 4 | ~15 Hz |
| mic FFT | `prop_mic.c` | 5 | ~10 Hz I2S + FFT |
| wifi/sig/rf scan | `prop_ui.c` | 4 | on-demand, **blocking** 1–5 s scan (off the UI task — good) |

A radio task migrating onto LVGL's core mid-render is the textbook cause of the swinging frame times.
Pin LVGL to one HP core and the radio/background tasks to the other, then re-measure consistency.
