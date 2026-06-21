# Communicator framerate review

## Measured baseline

Static review — not measured on hardware. No device reachable during this analysis.

Known baselines from cost-model and in-code comments (`prop_ui.c:2935-2948`):
- Full-screen software render: ~250 ms (~4 fps)
- Spectrum screen (live FFT bars): ~8 fps
- Static/mostly-static screens (home, vitals, ble idle): ~18 fps
- The gap between 8 and 18 fps is dominated by per-frame PSRAM allocation during draw-mask allocation, not raw pixel count.

The user reports 10-20 fps overall with inconsistent frame times — consistent with these baselines plus the contention issues described below.

---

## Findings (ranked by frame-time impact)

---

### **[P1] LVGL task unpinned — all background tasks share both HP cores** — affects: **consistency**

- **Where:** `peripheral/bsp_illuminate/bsp_illuminate.c:159` (`task_affinity = -1`)
- **Cost:** The LVGL port task has no core affinity. The engine task (`prop_engine.c:302`, prio 5), mic FFT task (`prop_mic.c:186`, prio 5), CSI fold task (`prop_csi.c:151`, prio 4), BLE prune task (`prop_ble.c:331`, prio 3), rssi task (`prop_net.c:255`, prio 3), and wifi/scan tasks all run unpinned. When any of these migrate to the core LVGL is using mid-render, the frame stalls until the scheduler preempts the intruder. This is the classic cause of swinging frame times (e.g. 18->6->18 spikes) — not bad average throughput but variance, which is exactly the user's priority-1 complaint.
- **Fix:** Pin LVGL to core 0 (`task_affinity = 0` in `lvgl_port_cfg_t`). Pin the radio/background tasks to core 1 by changing their `xTaskCreate` calls to `xTaskCreatePinnedToCore(..., 1)`:
  - `animate_task` in `prop_engine.c:302` -> core 1
  - `mic_task` in `prop_mic.c:186` -> core 1
  - `csi_task` in `prop_csi.c:151` -> core 1
  - `prune_task` in `prop_ble.c:331` -> core 1
  - `rssi_task` in `prop_net.c:255` -> core 1
  - `rfband_scan_task` in `prop_ui.c:1108` -> `xTaskCreatePinnedToCore(..., 1)`
  - NimBLE host task: set `CONFIG_BT_NIMBLE_PINNED_TO_CORE=1` in sdkconfig.defaults
- **Estimated win:** Eliminates frame-time spikes from task migration. Expected: HUD reads stabilize to a consistent number instead of swinging. This is the highest-confidence fix for consistency.
- **Risk:** Core 1 will carry more load. Do not also enable `LV_DRAW_SW_DRAW_UNIT_CNT=2` until after measuring this change, since a second draw unit on core 0 is fine, but on core 1 it would fight radio tasks.
- **Constraint check:** Clean. No LVGL heap, display config, or WiFi stack changes.

---

### **[P1] Bar chart observer calls `lv_obj_align()` unconditionally every tick on every bar** — affects: **consistency + throughput**

- **Where:** `prop_ui.c:3044-3045` (SPECTRUM, 24 bars), `prop_ui.c:3066-3067` (RF BAND, 13 bars), `prop_ui.c:3109-3110` (CSI, 32 bars)
- **Cost:** `lv_obj_align()` in LVGL 9 re-evaluates alignment relative to the parent and writes new coordinates — even if the bar did not move. Each call marks the object dirty. The SPECTRUM block calls `lv_obj_set_height()` + `lv_obj_align()` + `lv_obj_set_style_bg_color()` for each of 24 bars = 72 LVGL calls per observer tick. CSI does the same for 32 bars = 96 calls/tick. The `lv_obj_align()` in these loops is entirely redundant: bars are always aligned to `LV_ALIGN_BOTTOM_LEFT` at a fixed `x = SPEC_X0 + i * (SPEC_BW + SPEC_GAP)`. After the initial align at build time (`prop_ui.c:1043`, `1305`), only `lv_obj_set_height()` needs to be called per tick.
- **Fix:**
  1. Remove the `lv_obj_align(s_spec_bars[i], ...)` call at line 3044. Repeat for `s_rf_bars[i]` at 3066 and `s_csi_bars[i]` at 3109.
  2. Add height shadow-compare: track previous pixel height in `static int s_spec_h[PROP_MIC_BANDS]` and skip `lv_obj_set_height` + `lv_obj_set_style_bg_color` when the value hasn't changed (same pattern as `s_wave_shadow[]` at prop_ui.c:59). In a quiet room the spectrum becomes nearly zero-invalidation.
- **Estimated win:** Eliminates 48-96 spurious LVGL calls/tick on the heavy screens. Expected: spectrum fps improves toward static-screen rates (~15-18 fps) when signal is stable.
- **Confidence:** Code-evident. The alignment geometry is fixed at build time; the per-tick re-align is dead work.
- **Risk:** None. Bar x-positions are static; removing the redundant align cannot change visual output.

---

### **[P2] LVGL heap entirely in PSRAM — every draw-mask allocation pays PSRAM latency** — affects: **throughput (primarily spectrum/CSI)**

- **Where:** `main/lv_port_mem.c:39` (`lv_malloc_core` -> `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`)
- **Cost:** LVGL 9 allocates anti-alias line/shape masks via `lv_malloc` during draw. These are small, short-lived (a few KB, freed after each draw call), but they land in PSRAM. PSRAM random-access latency is ~3-5x slower than internal SRAM. With 24-32 bars drawn per frame, each with its own mask allocation round-trip, this is the documented reason spectrum runs at ~8 fps vs ~18 fps static. Cost-model names this "the highest-value, not yet done" lever.
- **Fix:** Implement a hybrid allocator in `lv_port_mem.c`: route small allocations (<= ~4 KB) to `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT` with PSRAM fallback on failure; large allocations go directly to PSRAM.
  ```c
  void *lv_malloc_core(size_t size) {
      if (size <= 4096) {
          void *p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          if (p) return p;
      }
      return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  // Same pattern for lv_realloc_core
  ```
  Must leave >=150 KB internal RAM free for esp_hosted SDIO DMA mempool (332 KB free at runtime is the budget).
- **Estimated win:** Cost-model calls this the single biggest throughput win. Expect spectrum to approach 12-18 fps.
- **Confidence:** High in mechanism, medium in exact number.
- **Risk:** Internal RAM budget. Monitor `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` before and after. If headroom tightens, lower the size threshold to 2 KB. Do NOT restore the LVGL builtin pool or raise `LV_MEM` — that is the confirmed boot-loop path.

---

### **[P2] BLE CONTACTS: `lv_obj_clean()` + full row rebuild every ~400 ms** — affects: **consistency (periodic hitch)**

- **Where:** `prop_ui.c:3091` (`lv_obj_clean(s_ble_list)`) called at `tick % 8 == 0` (every 8 x 50 ms = 400 ms)
- **Cost:** `lv_obj_clean()` tears down the entire widget hierarchy inside `s_ble_list` (up to 24 rows x ~6 widgets each = ~144 widget deletions + recreations per rebuild). This is a periodic allocation spike in PSRAM under the LVGL lock — a frame-time hitch every 400 ms regardless of whether the BLE device list actually changed. Reads as a periodic stutter even on an otherwise smooth screen.
- **Fix:** Replace teardown-and-rebuild with a fixed-size row pool. At `build_ble_panel()` time, pre-allocate `PROP_BLE_MAX` (24) row widget sets (hidden initially). In the observer, update label text and bar widths in-place; show/hide rows to match the current device count. Add string-compare guards so rows whose data didn't change skip all LVGL calls entirely.
- **Estimated win:** Eliminates the ~30-50 ms periodic hitch on the CONTACTS screen. Frame times become smooth.
- **Risk:** Low. Visual output is identical.

---

### **[P3] CRT overlay static canvas is full-screen ARGB8888 on `lv_layer_top()`** — affects: **throughput (when FX enabled)**

- **Where:** `prop_fx.c:182-193` (`s_canvas`, 1024x600 ARGB8888); `prop_fx.c:207` (`s_band_timer` at 55 ms)
- **Cost:** When FX is enabled, any dirty region on any screen forces LVGL to software alpha-blend the 1024x600 ARGB8888 static canvas over that region before flush. This is per-pixel alpha math in software, using the slow ARGB8888 composite path instead of a fast RGB565 blit. The band timer at 55 ms additionally forces an 80x1024 px recomposite 18 times/sec even on otherwise static screens. Quantify with A/B: `{"cmd":"fx","on":false}` vs `on:true` with HUD.
- **Fix options:**
  1. Bake `s_canvas` as RGB565 by premultiplying scanlines and vignette against `COL_BG` (0x0A0A06) at build time. Store as RGB565, set `LV_COLOR_FORMAT_RGB565`. Removes per-pixel alpha from the static layer (the scrolling band stays ARGB8888 — it must composite over dynamic content).
  2. Raise `FX_BAND_MS` from 55 to 100 ms (~10 fps sweep) — halves band-recomposite work with imperceptible visual difference at screen-capture distances.
- **Estimated win:** Depends on how many dirty-region operations per second overlap with the FX layer. Quantify with A/B before implementing. Expected: 2-5 fps recovery on active screens with FX on.
- **Risk:** Option 1 changes the visual slightly (premultiplied against black background; check on lighter panels). Option 2 is safe.

---

### **[P3] `LV_DRAW_SW_DRAW_UNIT_CNT=1` — single draw unit** — affects: **throughput**

- **Where:** `sdkconfig` (`CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`)
- **Cost:** A second software draw unit can parallelize independent draw operations across the two HP cores, but only if core affinity separates LVGL from radio tasks first (otherwise the second unit fights radio tasks for time on the busy core).
- **Fix:** Set `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2` **after** the core-pinning fix (P1). The second unit should run on core 0 alongside LVGL, handling independent dirty regions in parallel.
- **Estimated win:** 20-30% throughput on screens with multiple simultaneous dirty regions. Confidence: medium.
- **Risk:** Additional internal stack. Monitor stack watermarks. Do not enable without P1 first.

---

### **[P3] Partial draw buffers in internal SRAM (experiment)** — affects: **throughput**

- **Where:** `bsp_illuminate.c:172-174` (`buffer_size = H_size * V_size`, `buff_spiram = true`)
- **Experiment:** Replace full-screen PSRAM buffers with a ~40-row partial buffer in internal SRAM (`buffer_size = H_size * 40`, `buff_spiram = false`). A 40-row double buffer uses ~164 KB of internal RAM, within the ~180 KB budget (332 KB free minus 150 KB reserve). Rendering into internal SRAM is faster; LVGL issues more DMA transfers (15 instead of 1) but they overlap with CPU rendering.
- **Estimated win:** Uncertain — depends on dirty-region size. Small regions get faster; large dirty regions may be slower due to DMA overhead. Measure, do not assume.
- **Risk:** Low correctness risk. Performance may not improve or may regress on some screens. Last experiment to try.

---

## Suggested order of attack

1. **Core pinning** (P1, bsp_illuminate.c + all xTaskCreate sites). Zero visual change. Measure HUD swing before/after — should flatten immediately.
2. **Remove redundant `lv_obj_align()` from bar loops + add height shadow-compare** (P1, prop_ui.c:3044, 3066, 3109). Measure spectrum/CSI fps.
3. **Hybrid allocator** (P2, lv_port_mem.c). Monitor internal RAM headroom. Measure spectrum fps.
4. **BLE row pool** (P2, prop_ui.c around 3091). Verify CONTACTS screen frame-time consistency over 30 s.
5. **CRT overlay A/B** (P3) — measure cost of FX, then decide between baking to RGB565 or raising FX_BAND_MS.
6. **Second draw unit** (P3) — only after P1 confirmed working.
7. **Partial SRAM buffers** (P3) — last experiment, measure carefully.

---

## Measurement plan

```bash
# Enable FPS HUD
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'

# Per-screen steady-state FPS (watch HUD over ~10 s, note mean AND swing)
python firmware/communicator/tools/prop.py shot spectrum.png --screen spectrum --wait
python firmware/communicator/tools/prop.py shot csi.png     --screen csi     --wait
python firmware/communicator/tools/prop.py shot ble.png     --screen ble     --wait
python firmware/communicator/tools/prop.py shot home.png    --screen home    --wait

# A/B: CRT overlay cost (run on spectrum screen for worst case)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'
# navigate to spectrum, note FPS HUD for 10 s
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
# navigate to spectrum, note FPS HUD for 10 s
# delta = FX composite overhead

# After P1 (core pinning): re-run per-screen test
# Success: HUD reads a stable number (e.g. "15") instead of swinging

# After P2 (hybrid allocator): spectrum fps
# Expect: spectrum approaches 12-18 fps instead of ~8

# After BLE fix: watch CONTACTS HUD over 30 s
# Success: no periodic dips every ~400 ms
```
