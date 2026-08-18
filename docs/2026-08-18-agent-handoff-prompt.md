# Agent Handoff Prompt: CrowPanel Communicator Prop Implementation & Performance Overhaul

> **Instructions for the Incoming Agent**:  
> You are tasked with executing the bug fixes, performance optimizations, and LIDAR UI redesign established in the comprehensive audit of the **CrowPanel Communicator Prop** (`hellosamblack/CrowPanelProp`).
>
> Follow the strict execution order below. Ground all edits in verified file paths, obey all `AGENTS.md` constraints, and test incrementally.

---

## 1. Ground Truth & Core Constraints

* **Hardware Target**: CrowPanel Advance ESP32-P4 (7.0" 1024×600 IPS Display, chip rev v1.3 ES) + ESP32-C6 companion module over SDIO.
* **Toolchain**: ESP-IDF v6.0.1 (`target esp32p4`).
* **Non-Negotiable Architectural Rules (`AGENTS.md`)**:
  1. **Do not change `CONFIG_ESP32P4_*REV*`** in `sdkconfig.defaults` (must support pre-v3 silicon line).
  2. **Do not restore the LVGL builtin heap / do not raise `LV_MEM`** (LVGL heap is routed to PSRAM via `main/lv_port_mem.c` to preserve internal SRAM for SDIO DMA mempool).
  3. **Main task stack must remain 8192 bytes** (`CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192`).
  4. **Display takes native little-endian RGB565** (`swap_bytes = false`).
  5. **Never call Wi-Fi/SDIO APIs under the LVGL port lock** (causes UI freeze).
  6. **No drawing pipeline calls (`lv_snapshot` / canvas layer-draw) from non-LVGL tasks under lock** (deadlocks).
  7. **`lv_label_set_text_fmt` has NO `%f` float support** (always format floats via `snprintf` into a buffer first).

---

## 2. Phase 1: Critical Bug Fixes & Repository Hygiene

### Task 1.1: Resolve GPIO 47/48 Contention & GPIO 11 Placeholder (P1 Hardware Bug)
* **Problem**: `components/bsp_io/bsp_io.c` configures GPIO 47 and 48 as LED digital outputs, driven at 20 Hz in `prop_engine.c:239`. Concurrently, `main/prop_aux_radar.c:35-37` uses GPIO 47 (TX) and 48 (RX) for UART3 to interface with the Seeed MR24HPC1 radar, corrupting UART communication. Furthermore, GPIO 11 is assigned to a placeholder `[BTN_ACTION]` despite being hardwired to `CSI_RESET` on the camera connector.
* **Action**:
  1. Open `components/bsp_io/bsp_io.c`.
  2. Remove `[LED_POWER] = { 48, "power" }` and `[LED_SIGNAL] = { 47, "signal" }` from `led_table`.
  3. Remove or reassign `[BTN_ACTION]` placeholder on GPIO 11.
  4. In `bsp_io.c`, ensure `bsp_io_led_set_mask` safely handles unassigned LED indices.

### Task 1.2: Remove Orphaned Gitlink Entry (P2 Defect)
* **Problem**: The git index records `docs/hardware/schematic/kandle` (`160000 a9fc0f1...`) without a matching `.gitmodules` entry, causing `git submodule status / update` to fail with exit code 128.
* **Action**:
  ```bash
  git rm --cached docs/hardware/schematic/kandle
  ```

### Task 1.3: Thread-Safe Atomic Struct Access in `prop_net.c` (P2 Concurrency)
* **Problem**: `s_uplink` in `main/prop_net.c` is written field-by-field in `rssi_task` and read by UI/telemetry via `*out = s_uplink;` without synchronization.
* **Action**:
  1. Add a `portMUX_TYPE s_uplink_mux = portMUX_INITIALIZER_UNLOCKED;` to `main/prop_net.c`.
  2. Guard all writes in `rssi_task` and reads in `prop_net_get_uplink` with `taskENTER_CRITICAL(&s_uplink_mux)` / `taskEXIT_CRITICAL(&s_uplink_mux)`.

### Task 1.4: Dynamic Buffer Allocation in `prop_api.c` (P2 API Bug)
* **Problem**: `cmd_post_handler` and `ws_handler` in `main/prop_api.c` use fixed 256-byte stack buffers, rejecting valid JSON payloads ≥ 256 bytes with HTTP 400.
* **Action**:
  1. Replace `char buf[256]` in `cmd_post_handler` with dynamic allocation sized to `req->content_len + 1` (capped at 2048 bytes).
  2. In `ws_handler`, increase message receive buffer to 1024 bytes.

### Task 1.5: NimBLE Scan Auto-Retry Daemon (P3 Subsystem Bug)
* **Problem**: In `main/prop_ble.c:277`, if `ble_gap_disc()` fails on `BLE_GAP_EVENT_DISC_COMPLETE`, the scan stalls permanently.
* **Action**:
  1. Add a retry flag `s_scan_restart_pending` in `main/prop_ble.c`.
  2. In `prune_task` (~2 Hz loop), check `s_scan_restart_pending` and attempt `start_scan()` until successful.

### Task 1.6: Update CLI Reference
* **Action**: In `tools/prop.py`, add `lidar` to the docstring screen list and examples.

---

## 3. Phase 2: Performance & Framerate Optimizations

### Task 2.1: FreeRTOS Core Pinning & Task Affinity
* **Problem**: Unpinned tasks on Core 0/1 cause background network bursts to preempt LVGL mid-render, causing 18 FPS → 6 FPS stutter.
* **Action**:
  1. In `components/bsp_illuminate/bsp_illuminate.c:248`, set `task_affinity = 1` for the LVGL port task.
  2. In `main/prop_engine.c:299`, pin `animate_task` to Core 0 (`xTaskCreatePinnedToCore(..., 0)`).
  3. Verify all network and sensor tasks (`prop_lidar`, `prop_net`, `prop_ble`, `prop_motion`, `prop_imu`) are pinned to Core 0.

### Task 2.2: Eliminate Engine Tick Drift
* **Problem**: `main/prop_engine.c:283` uses `vTaskDelay(pdMS_TO_TICKS(50))`, creating a drifting 52–56 ms loop with high jitter.
* **Action**:
  1. In `main/prop_engine.c::animate_task`, initialize `TickType_t last_wake = xTaskGetTickCount();`.
  2. Replace `vTaskDelay(pdMS_TO_TICKS(50))` with `vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ANIM_PERIOD_MS))`.

### Task 2.3: Tune Hybrid PSRAM / SRAM Allocator Threshold
* **Problem**: `main/lv_port_mem.c:32` routes allocations > 512 B to PSRAM. LVGL 9 line-draw anti-aliasing masks (256–1024 B) pay high PSRAM latency penalties.
* **Action**:
  1. In `main/lv_port_mem.c`, update `#define LV_SRAM_THRESHOLD 1024`.

### Task 2.4: Integrate Hardware PPA A8 CRT Blending
* **Action**:
  1. Wire the verified `ppa_do_blend` pipeline from `main/prop_ppa_spike.c` into `main/prop_fx.c::paint_canvas` to perform hardware-accelerated 2D-DMA alpha blending for scanlines and vignettes.

### Task 2.5: Bar Height Shadow-Compare (Spectrum / CSI)
* **Action**:
  1. In `main/prop_ui.c`, add `s_spec_last_bands[PROP_MIC_BANDS]` shadow array.
  2. Only invoke `lv_obj_set_height` and `lv_obj_set_style_bg_color` when a band's delta exceeds ±1%.

---

## 4. Phase 3: LIDAR Protocol & UI Redesign (`PK_LIDAR`)

### Task 3.1: Ingest Updated `thin_telemetry` & Binary Header
* **Verified Server Schema (`thin_telemetry`)**:
  ```json
  {
    "type": "thin_telemetry",
    "fps": 12.4,
    "point_count": 2450,
    "recording": false,
    "mode": "point_cloud",
    "link": "ok",
    "heading_deg": 248.5,
    "pitch_deg": -2.4,
    "roll_deg": 1.1,
    "yaw_rate_dps": 0.0,
    "orientation_valid": true,
    "ir_grid": [12, 14, 18, 22, ... 64 ints ...]
  }
  ```
* **Binary Frame Ingest Architecture**:
  * **Tag 1**: Uncompressed RGB565 (8-byte header: `tag=1`, `w=480`, `h=480` + 460,800 bytes).
  * **Tag 2 (Hardware JPEG Roadmap)**: TurboJPEG bitstream (8-byte header: `tag=2`, `w`, `h` + ~20 KB JPEG payload decoded via ESP32-P4 Hardware JPEG Decoder block).
* **Action**:
  1. In `main/include/prop_lidar.h`, update `prop_lidar_telemetry_t`:
     - Add `heading_deg`, `pitch_deg`, `roll_deg`, `yaw_rate_dps`, `orientation_valid`.
     - Add `uint8_t ir_grid[64]`, `bool has_ir_grid`.
     - Remove `power_mode` and `i3c_airtime_pct`.
  2. In `main/prop_lidar.c::on_telemetry_json`, parse the new JSON fields with `cJSON`.

### Task 3.2: Layout & Widget Refactoring in `main/prop_ui.c`
* **Layout Specifications (948 × 600 Content Area)**:
  * **Left (480 × 480 @ x=24, y=24)**: 3D Point Cloud / SLAM Canvas (`s_lidar_canvas`).
  * **Right Sidebar (396 × 480 @ x=528, y=24)**:
    1. **Telemetry Block**:
       * Retain: `LINK`, `FPS`, `POINTS`, `REC` Status.
       * **Remove**: `POWER` Mode and `I3C BUS` Airtime %.
    2. **Orientation & Heading Block**:
       * `s_lidar_hdg`: When `orientation_valid == true`, displays `HDG: 248.5° WSW` in `COL_AMBER`. When `false` (vertical boresight / uncalibrated), displays `HDG: ---` in `COL_DIM`.
       * `s_lidar_attitude`: Displays `P: -2.4°  R: +1.1°`.
       * Yaw rate stability indicator (`yaw_rate_dps`).
    3. **IR Preview Block**:
       * `s_lidar_ir_canvas`: 160 × 120 RGB565 PSRAM canvas (`s_lidar_ir_buf`).
       * In `ui_observer`: map the 64-element `ir_grid` array (integers 0–255) to an 8×8 thermal-gradient raster across all viewport modes.
    4. **Action Control**:
       * Full-width themed `REC` toggle button at the bottom of the sidebar.

### Task 3.3: Zero-Copy PSRAM Buffer Swap for LIDAR Canvas
* **Action**:
  1. In `main/prop_lidar.c`, maintain pointer swapping between front and back buffers without intermediate `memcpy`.
  2. In `main/prop_ui.c::ui_observer`, call `lv_canvas_set_buffer(s_lidar_canvas, frame_ptr, 480, 480, LV_COLOR_FORMAT_RGB565)`.
  3. Invalidate *only* `s_lidar_canvas`.
  4. Throttle sidebar text formatting to 5 Hz (every 4th engine tick).

---

## 5. Verification & Smoke Testing Plan

1. **Reconfigure & Build**:
   ```bash
   idf.py reconfigure
   idf.py build
   ```
2. **Flash & Launch Diagnostics**:
   ```bash
   idf.py -p /dev/ttyUSB0 flash
   python tools/prop.py wait
   python tools/prop.py state
   python tools/prop.py telemetry
   ```
3. **Validate UI & Performance with Live Capture**:
   ```bash
   # Enable FPS HUD and capture screens
   curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'
   python tools/prop.py shot lidar.png --screen lidar --wait
   python tools/prop.py shot spectrum.png --screen spectrum --wait
   python tools/prop.py shot scanner.png --screen scanner --wait
   ```
4. **Verify Telemetry Stream**:
   ```bash
   python tools/prop.py watch --only imu.yaw_deg,radar,track --count 20
   ```

---

## 6. Key Documentation References

* Comprehensive Audit Report: `docs/2026-08-18-comprehensive-firmware-hardware-audit.md`
* LIDAR UI Redesign & Perf Plan: `docs/2026-08-18-lidar-ui-redesign-and-framerate-optimization-plan.md`
* Performance Cost Model: `.agents/skills/communicator-perf/references/cost-model.md`
