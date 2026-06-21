# Communicator framerate review

## Measured baseline

Static review — not measured on hardware (no device available).

Known baselines from the cost model and in-code comments:
- Home/static screens: ~18 fps
- Spectrum screen (mic FFT bars, live): ~8 fps
- Full-screen software render: ~250 ms (~4 fps)
- The ~10 fps gap between home and spectrum is documented in the cost model as being dominated by
  PSRAM allocation latency during drawing, not by raw pixel count.

The user describes spectrum as "a crawl" with "stuttering bars" but home running fine. The code
confirms this: home makes few LVGL calls per frame (throttled to ~2 Hz via `tick % 10`), while
spectrum makes 72+ unconditional LVGL calls every observer tick (~20 Hz) and generates per-frame
PSRAM allocations for every draw mask.

---

## Findings (ranked by frame-time impact)

---

### [P1] PSRAM allocator for every draw mask — affects: throughput (and contributes to jitter)

- **Where:** `firmware/communicator/main/lv_port_mem.c:39` (`lv_malloc_core`)
- **Cost:** LVGL 9 allocates line/shape anti-alias masks via `lv_malloc` *during* the software
  render pass. `lv_malloc_core` routes 100% of allocations to PSRAM via `heap_caps_malloc(...,
  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)`. PSRAM latency is significantly higher than internal SRAM
  (~10-20x for small random allocations). Every bar redraw that triggers a shaped/anti-aliased region
  pays this tax. On the spectrum screen, 24 bars updating every frame means a burst of small PSRAM
  alloc/free pairs on every observer tick. This is explicitly called out in the cost model as *the*
  reason spectrum reads ~8 fps vs ~18 fps for static screens.
- **Fix:** Implement a hybrid allocator in `lv_port_mem.c`: route small/short-lived allocations
  (< 512-1024 bytes) to internal SRAM (`heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)`)
  with PSRAM as a fallback for large buffers. The threshold should be tuned to keep total internal RAM
  consumption well under the ~332 KB free at runtime (the esp_hosted SDIO DMA mempool needs that
  headroom). Large widget buffers and canvas data stay in PSRAM.
  ```c
  // lv_port_mem.c -- replace lv_malloc_core with:
  #define SMALL_ALLOC_THRESHOLD 1024
  void *lv_malloc_core(size_t size) {
      if (size <= SMALL_ALLOC_THRESHOLD) {
          void *p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          if (p) return p;
      }
      return heap_caps_malloc(size, LV_PSRAM_CAPS);
  }
  // Same pattern for lv_realloc_core.
  ```
  Constraint: must not deplete internal RAM below ~200 KB free or the SDIO mempool boot-loops.
  Verify with the Vitals instrument (`heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` is shown live).
- **Estimated win:** Moderate-to-high throughput gain on spectrum (~2-5 fps estimated). Confidence:
  high (documented mechanism in cost model as the named highest-value lever).
- **Risk:** Internal RAM exhaustion causes boot loop. Keep threshold conservative; monitor Vitals.

---

### [P1] Unconditional `lv_obj_align` every frame on all 24 spectrum bars — affects: throughput

- **Where:** `firmware/communicator/main/prop_ui.c:3043-3045` (spectrum); also CSI at `3108-3110`
- **Cost:** Inside the spectrum observer loop (24 bars, every ~50 ms / 20 Hz), three LVGL calls
  are made unconditionally per bar every frame: `lv_obj_set_height`, `lv_obj_align`, and
  `lv_obj_set_style_bg_color` — 72 LVGL calls per frame total. `lv_obj_align` is not a no-op in
  LVGL 9: it recomputes position, marks layout dirty, and triggers an invalidation. The bars' X
  coordinates are fixed by construction (`SPEC_X0 + i * (SPEC_BW + SPEC_GAP)`) and never change
  after `build_spectrum_panel`. The `lv_obj_align` is being called on every frame to re-anchor
  the bottom edge after each height change, even on frames where height and color did not change
  (slow decay means many bars are at the same value two frames in a row). Contrast with the scanner
  wave (same file, `prop_ui.c:2566-2580`) which uses a shadow-compare to skip unchanged segments.
  Spectrum never shadow-compares: it always fires all 72 calls.
- **Fix:** Add per-bar shadow arrays and gate each call on an actual change:
  ```c
  // Add near other static shadow arrays (top of prop_ui.c):
  static int s_spec_h_last[PROP_MIC_BANDS];
  static uint8_t s_spec_col_last[PROP_MIC_BANDS]; // 0=mute,1=amber,2=alert

  // In the spectrum observer block, replace the inner loop body with:
  uint8_t col_idx = pct > 85 ? 2 : (pct > 35 ? 1 : 0);
  if (h != s_spec_h_last[i]) {
      lv_obj_set_height(s_spec_bars[i], h);
      lv_obj_align(s_spec_bars[i], LV_ALIGN_BOTTOM_LEFT,
                   SPEC_X0 + i * (SPEC_BW + SPEC_GAP), -SPEC_BASE);
      s_spec_h_last[i] = h;
  }
  if (col_idx != s_spec_col_last[i]) {
      lv_color_t c = col_idx == 2 ? COL_ALERT : (col_idx == 1 ? COL_AMBER : COL_MUTE);
      lv_obj_set_style_bg_color(s_spec_bars[i], c, 0);
      s_spec_col_last[i] = col_idx;
  }
  ```
  Reset `s_spec_h_last` and `s_spec_col_last` in `close_panel()` (alongside the existing
  `s_spec_bars[i] = NULL` nulling). Apply the same shadow-compare to the CSI bars (32 bars, same
  three-call pattern at `prop_ui.c:3107-3112`).
- **Estimated win:** Medium throughput gain. In steady state (slow decay, bars mostly stable),
  reduces 72 LVGL calls/frame down to only the changed bars (often 0-5). Confidence: high
  (code-evident; this is exactly what the scanner wave optimization comment describes at line 2563-2564).
- **Risk:** Low. Must initialize and reset shadow arrays correctly. The `lv_obj_align` call on first
  open still runs once per bar during `build_spectrum_panel`, which is correct.

---

### [P1] No core affinity — LVGL and all radio tasks compete on both HP cores — affects: consistency (the stutter)

- **Where:** `peripheral/bsp_illuminate/bsp_illuminate.c:159` (`task_affinity = -1`); plus
  `prop_net.c:244,255`, `prop_ble.c:331`, `prop_csi.c:151`, `prop_engine.c:302` (all `xTaskCreate`,
  all unpinned)
- **Cost:** FreeRTOS schedules all tasks freely across both HP cores. When a NimBLE event,
  CSI fold (~15 Hz), RSSI poll (~1 Hz), or the engine/observer task (~20 Hz) lands on the same
  core as the LVGL render task mid-render, the frame stalls until that task yields. This produces
  the frame-time swings (18->6->18 fps pattern) that read as stutter. On spectrum, the LVGL render
  takes longer per frame than home (more dirty area, more draw masks), so the window for a radio
  task to collide with it is wider. The engine/observer task (prio 5, same as mic FFT task) also
  runs unpinned: when it fires on LVGL's core it both delays the render AND delays the next observer
  callback, creating a double-stall.
- **Fix:** Pin LVGL to core 1; push all background tasks to core 0:
  ```c
  // bsp_illuminate.c:159:
  .task_affinity = 1,  /* LVGL pinned to HP core 1 */

  // prop_engine.c:302:
  xTaskCreatePinnedToCore(animate_task, "prop_anim", 4096, NULL, 5, NULL, 0);

  // prop_net.c:244:
  xTaskCreatePinnedToCore(ap_fallback_task, "ap_fallback", 3072, NULL, 3, NULL, 0);
  // prop_net.c:255:
  xTaskCreatePinnedToCore(rssi_task, "rssi", 4096, NULL, 3, NULL, 0);

  // prop_ble.c:331:
  xTaskCreatePinnedToCore(prune_task, "ble_prune", 2560, NULL, 3, NULL, 0);

  // prop_csi.c:151:
  xTaskCreatePinnedToCore(csi_task, "prop_csi", 4096, NULL, 4, NULL, 0);
  ```
  Also verify `CONFIG_BT_NIMBLE_PINNED_TO_CORE=0` in sdkconfig so NimBLE's internal tasks also
  stay on core 0.
- **Estimated win:** Primary fix for bar stutter. Confidence: high in mechanism; magnitude needs
  on-device measurement (A/B with the FPS HUD watching consistency over ~10 s).
- **Risk:** Core 0 will be busier. If it saturates, the second SW draw unit experiment (P3)
  becomes counterproductive. Measure core affinity first before enabling two draw units.

---

### [P2] CRT overlay alpha-composite on every dirty region — affects: throughput (when FX is on)

- **Where:** `firmware/communicator/main/prop_fx.c:187-188` (static canvas), `prop_fx.c:199-207`
  (refresh band)
- **Cost:** `s_canvas` is a full-screen 1024x600 ARGB8888 canvas on `lv_layer_top()`. Because it
  has per-pixel alpha (scanlines are partially transparent, vignette edges vary), LVGL must
  alpha-blend this canvas over every dirty region of every frame. On spectrum, every bar height
  change invalidates its bounding box, and each dirty region triggers the full ARGB8888-over-RGB565
  alpha-blend path through the SW renderer. The refresh band (thin ~80px canvas, also ARGB8888 on
  `lv_layer_top()`, scrolling at ~18 fps) adds a separate dirty-region composite per band tick.
  The static canvas design (baked at build time, not animated) is already the right approach; the
  cost is the mandatory per-dirty-region recomposite inherent in any translucent `lv_layer_top()`
  object.
  NOTE: The cost model says FX is off by default (`fx_on` stored in NVS, default 0). If the user
  has FX disabled, this finding does not apply to them. Confirm with `GET /state` or Vitals/FX
  panel before investigating.
- **Fix (A/B first):** Toggle overlay off with `{"cmd":"fx","on":false}` and measure spectrum FPS.
  If FPS jumps meaningfully (e.g., 8->12 fps), the overlay is a significant contributor and
  optimizing the canvas opacity model (opaque segments for uniform regions, alpha only at edges) is
  worth pursuing. If the jump is small (< 1-2 fps), the P1 fixes are the right focus.
- **Estimated win:** Unknown until A/B. Confidence: medium (code-evident cost, unknown magnitude).
- **Risk:** Any fix must not use a translucent full-screen animated object on `lv_layer_top()` --
  documented watchdog dead end in the cost model.

---

### [P2] Spectrum update cadence at 20 Hz with mic data at ~10 Hz — affects: throughput

- **Where:** `firmware/communicator/main/prop_ui.c:3034` (no `tick %` guard on spectrum block)
- **Cost:** The observer fires at 20 Hz. The mic FFT (`prop_mic.c`, I2S-driven) produces new data
  at ~10 Hz. Half the observer calls re-render already-decayed values with no actual data change.
  Without the shadow-compare fix (P1), all 72 LVGL calls still fire on those no-change frames. The
  decay coefficient (`*= 0.80` per 50 ms tick) is calibrated for 20 Hz.
- **Fix:** After applying the P1 shadow-compare fix, the no-change frames become nearly free. If
  further gains are needed, add a `tick % 2 == 0` guard on the spectrum block (matching mic rate at
  10 Hz) and adjust decay to `*= 0.64` per 100 ms tick to preserve visual appearance. The
  `s_spec_db` label is already throttled at `tick % 4`.
- **Estimated win:** Small additional gain on top of P1 shadow-compare. Confidence: medium.
  The P1 shadow-compare largely subsumes this.
- **Risk:** Low. Adjust decay coefficient to match the halved cadence.

---

### [P3] Second SW draw unit — affects: throughput (experiment, pairs with core affinity)

- **Where:** `firmware/communicator/sdkconfig:4925` (`CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`)
- **Cost/opportunity:** One draw unit today. With LVGL pinned to core 1 (after P1 affinity fix),
  core 0 handles radio tasks and may have idle headroom. A second draw unit would let LVGL dispatch
  render jobs to core 0 in parallel. Pay-off depends on core 0 saturation.
- **Fix:** Set `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2` in `sdkconfig.defaults`. Measure together with
  the core-affinity change; do not apply in isolation.
- **Estimated win:** Unknown; could be neutral or negative if core 0 is saturated. Confidence: low.
- **Risk:** If core 0 saturates, the second draw unit competes with radio tasks and makes frame-time
  variance *worse*. Confirm by watching consistency (not just average fps) with HUD.

---

### [P3] Partial draw buffers in internal SRAM — affects: throughput (experiment)

- **Where:** `peripheral/bsp_illuminate/bsp_illuminate.c:173,188`
  (`buffer_size = H_size * V_size`, `buff_spiram = true`)
- **Cost/opportunity:** Full-screen x2 PSRAM buffers (~2.4 MB each). Rendering into PSRAM is slow.
  For spectrum's actual dirty area (union of 24 bar bboxes, a strip ~900x312 px at most), a partial
  N-line internal SRAM buffer may render faster and flush to the panel via DMA in strips.
- **Fix (experiment):** Try `buffer_size = H_size * 40` with `buff_spiram = false`. Verify no
  display corruption (partial-strip mode requires multiple flush callbacks per frame; `esp_lvgl_port`
  2.8 supports this but must be validated visually).
- **Estimated win:** Potentially significant but unproven. Confidence: low (untried experiment per
  cost model).
- **Risk:** Panel artifacts possible if strip alignment or DMA burst size doesn't match the EK79007
  panel's internal buffer. Test with `/screenshot` endpoint after each build.

---

## Suggested order of attack

1. **Core affinity (P1)** — change `task_affinity=1` in `bsp_illuminate.c:159` and pin all
   background/radio tasks to core 0. Check `CONFIG_BT_NIMBLE_PINNED_TO_CORE=0`. Flash and measure
   frame-time consistency with HUD over 10 s on spectrum. This is the cheapest-to-implement fix
   and directly targets the stutter.

2. **Shadow-compare for bar updates (P1)** — add `s_spec_h_last[]` / `s_spec_col_last[]`, gate
   the three LVGL calls per bar on actual change. Apply same to CSI bars. Reset arrays in
   `close_panel()`. Flash and measure spectrum average fps.

3. **Hybrid PSRAM allocator (P1)** — modify `lv_port_mem.c` small-alloc path to try internal SRAM
   first. Monitor Vitals to confirm internal free RAM stays above ~200 KB. Flash and measure.

4. **CRT overlay A/B (P2)** — confirm whether the user has FX enabled. If yes, A/B toggle it and
   quantify the cost before deciding whether to pursue optimizing the canvas opacity model.

5. **Spectrum cadence throttle (P2)** — only if gains are still needed after steps 1-3. The
   shadow-compare largely subsumes it.

6. **Second draw unit + partial buffers (P3)** — experiments only, after measuring core affinity.
   Accept only if A/B with HUD shows both better average fps AND better consistency.

---

## Measurement plan

```bash
# Confirm device is reachable
python firmware/communicator/tools/prop.py state

# Enable the FPS HUD (amber readout, top-right)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'

# --- Baseline capture: spectrum (watch HUD for ~10 s, note range and swing) ---
python firmware/communicator/tools/prop.py shot spec_baseline.png --screen spectrum --wait

# --- Home screen baseline (should be ~18 fps and stable throughout) ---
python firmware/communicator/tools/prop.py shot home_baseline.png --screen home --wait

# --- CRT overlay A/B (only if user has FX enabled) ---
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'
python firmware/communicator/tools/prop.py shot spec_nofx.png --screen spectrum --wait
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
python firmware/communicator/tools/prop.py shot spec_withfx.png --screen spectrum --wait

# --- After core affinity fix (flash, then): ---
python firmware/communicator/tools/prop.py shot spec_affinity.png --screen spectrum --wait
# Observe HUD stability over 10 s -- look for consistent vs swinging number

# --- After shadow-compare fix: ---
python firmware/communicator/tools/prop.py shot spec_shadow.png --screen spectrum --wait

# --- After hybrid allocator: ---
# Check internal RAM headroom via Vitals first
python firmware/communicator/tools/prop.py shot vitals_alloc.png --screen vitals --wait
python firmware/communicator/tools/prop.py shot spec_alloc.png --screen spectrum --wait

# --- Second draw unit experiment (if pursuing P3, after affinity fix): ---
# sdkconfig.defaults: CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2, fullclean + rebuild + flash
python firmware/communicator/tools/prop.py shot spec_2du.png --screen spectrum --wait
# Check consistency as well as average fps -- if it swings more, revert
```

For consistency measurement, watch the HUD over ~10 s on spectrum and note whether the number is
stable (e.g., 10 fps +/- 1) or swinging (e.g., 12->4->12->3). A stable lower number is better
than a swinging higher one -- the stutter perception disappears even if average fps stays modest.

---

## Hard constraints cross-check

All proposed fixes satisfy the documented constraints:
- Hybrid allocator leaves internal SRAM headroom for esp_hosted (no LV_MEM/builtin pool change).
- No `CONFIG_ESP32P4_*REV*` touched.
- `swap_bytes=false` unchanged.
- No PPA/GPU path proposed.
- No WiFi/SDIO calls added under the LVGL lock.
- No canvas layer-draw or `lv_snapshot` from non-LVGL task.
- No translucent full-screen object on `lv_layer_top()` proposed.
- Refresh period change not proposed (documented dead end).
- Forced full-screen redraw not proposed (documented dead end, ~4 fps).
