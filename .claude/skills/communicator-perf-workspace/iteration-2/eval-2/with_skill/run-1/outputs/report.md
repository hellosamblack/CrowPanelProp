# Communicator framerate review

## Measured baseline

Static review — not measured on hardware. No physical device available.

Known baselines from cost-model (treat as ground truth):
- Full-screen 1024×600 software render: ~250 ms (~4 fps). Saturates the CPU.
- SPECTRUM screen (live mic FFT, bar updates every tick): ~8 fps.
- Mostly-static screens (HOME, VITALS throttled to 2 Hz, etc.): ~18 fps.
- The gap between ~8 and ~18 fps is dominated by PSRAM allocation latency during drawing,
  not raw pixel count. This is the cost model's primary documented lever.
- All tasks currently run unpinned (task_affinity = -1), which is the documented cause of
  swinging frame times.

---

## Findings (ranked by frame-time impact)

---

### [P1] All tasks unpinned — the primary cause of ~1 Hz hitches and inconsistent frame times
**Affects: consistency (priority-1)**

- Where: `peripheral/bsp_illuminate/bsp_illuminate.c:159` (`task_affinity = -1`); `main/prop_engine.c:302` (`xTaskCreate`); `main/prop_net.c:244,255`; `main/prop_ble.c:331`; `main/prop_csi.c:151`
- Cost: The FreeRTOS scheduler freely migrates tasks across the two HP cores. When a radio
  or background task lands on the same core as the LVGL port task mid-render, it preempts
  it. Since LVGL's port task holds the mutex during the render-and-flush cycle, the
  preemption stalls the frame until the interloper yields. The most disruptive tasks:
  - `rssi_task` (prio 3, unpinned, ~1 Hz) calls `esp_wifi_sta_get_ap_info` — an SDIO call
    that can take several ms. If it lands on the LVGL core during a render, it injects a
    multi-ms hitch every second. This directly matches the reported "hitch roughly once a
    second."
  - `prop_anim` engine task (prio 5, ~20 Hz) fans out to `ui_observer`, which also
    acquires `lvgl_port_lock`. If the engine task and the LVGL task land on the same core,
    the lock handoff itself adds latency to every observer frame.
  - `csi_task` (prio 4, ~15 Hz), `ble_prune` (prio 3, ~2 Hz), and the NimBLE host task
    add similar unpredictable contention.
  Because input (`prop_ui_input`) acquires `lvgl_port_lock` before acting, any render stall
  directly blocks dial and touch response — the reported laggy input and the hitches are
  the same root cause.
- Fix: Pin the LVGL port task to core 0 and push all radio/background tasks to core 1:
  - In `bsp_illuminate.c:159`: change `task_affinity = -1` to `task_affinity = 0`.
  - Replace `xTaskCreate` with `xTaskCreatePinnedToCore(..., 1)` for: `prop_anim`
    (prop_engine.c:302), `rssi_task` (prop_net.c:255), `ap_fallback_task` (prop_net.c:244),
    `prune_task` (prop_ble.c:331), `csi_task` (prop_csi.c:151). The NimBLE port task
    is launched by nimble_port_freertos_init — pin it to core 1 if the port exposes that.
  - Win estimate: Eliminates the per-second SDIO hitch and reduces overall frame-time
    variance. This is primarily a consistency fix; average FPS may not change much
    until the PSRAM allocator is also fixed (P2 below), but the hitches should disappear.
  - Confidence: High. This is the textbook fix for LVGL frame-time variance on dual-core
    ESP32 and exactly matches the ~1 Hz hitch pattern (rssi_task fires at ~1 Hz).
- Risk: Core 1 must not be saturated by the radio tasks alone or they will fight each other.
  At current task loads (rssi at 1 Hz, csi at 15 Hz, ble_prune at 2 Hz) this is
  unlikely, but measure after pinning. If CSI + BLE together peg core 1, throttle csi_task
  to ~10 Hz (it publishes synthesized data anyway).

---

### [P2] LVGL heap entirely in PSRAM — every draw-mask allocation pays PSRAM latency
**Affects: throughput (spectrum ~8 fps vs ~18 fps gap)**

- Where: `main/lv_port_mem.c:39–43` (`lv_malloc_core` routes everything to MALLOC_CAP_SPIRAM)
- Cost: LVGL 9 allocates anti-alias masks and scratch buffers via `lv_malloc` during
  drawing. With the heap entirely in PSRAM (~200 MHz HEX), every line or shape draw pays
  PSRAM allocation round-trip latency. On the SPECTRUM screen this happens for each of the
  24 bars' anti-alias paths. The cost-model identifies this as the documented primary
  reason spectrum is ~8 fps while static screens reach ~18 fps.
- Fix: Implement a hybrid allocator in `lv_port_mem.c`. Route small allocations
  (e.g. size <= 2048 bytes) to `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT` with a PSRAM
  fallback; keep large allocations (canvas pixel buffers) in PSRAM. The ~332 KB internal
  RAM free at runtime is the budget — stay well under it.

  Concrete change to lv_port_mem.c:
  ```c
  #define LV_HYBRID_THRESHOLD 2048
  void *lv_malloc_core(size_t size) {
      if (size <= LV_HYBRID_THRESHOLD) {
          void *p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          if (p) return p;
      }
      return heap_caps_malloc(size, LV_PSRAM_CAPS);
  }
  void *lv_realloc_core(void *p, size_t new_size) {
      if (new_size <= LV_HYBRID_THRESHOLD) {
          void *q = heap_caps_realloc(p, new_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          if (q) return q;
      }
      return heap_caps_realloc(p, new_size, LV_PSRAM_CAPS);
  }
  ```
  - Win estimate: Likely brings spectrum from ~8 fps toward ~15–18 fps. High-value
    throughput change per the cost model. Medium-high confidence.
- Risk: Must leave enough internal RAM for esp_hosted's SDIO DMA mempool. Monitor
  `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` from the VITALS screen. If it drops
  below ~200 KB under full radio load, raise the threshold to 1024.

---

### [P3a] Spectrum observer: lv_obj_align called unconditionally every frame on all 24 bars
**Affects: throughput (reduces LVGL invalidation work on SPECTRUM and CSI)**

- Where: `main/prop_ui.c:2723–2728` (SPECTRUM block), `prop_ui.c:2782–2793` (CSI block),
  `prop_ui.c:2739–2750` (RF BAND block) — all inside `ui_observer`
- Cost: Every observer tick (20 Hz), for all 24 SPECTRUM bars (and 32 CSI bars, 13 RF bars):
  - `lv_obj_set_height` — triggers layout dirty if height changed
  - `lv_obj_align` — called unconditionally, even when height is unchanged. In LVGL 9,
    this recalculates coordinates and marks the object dirty if they changed. When bars
    are decaying slowly, most are at the same height as the previous tick, but align
    is called regardless. Spurious invalidations force LVGL to re-examine dirty regions.
  - `lv_obj_set_style_bg_color` — similarly unconditional; color thresholds (85/35%) rarely
    change tick-to-tick but the style call is paid every tick.
  - Total for SPECTRUM: 24 bars × 3 LVGL calls = 72 LVGL calls/frame at 20 Hz, many
    generating no visible change but still queuing dirty-region work.
- Fix: Cache the last-rendered height and color bucket per bar (same pattern as the scanner
  trace already uses: `s_wave_shadow`, `s_last_blip_x`, `s_last_blip_col`). Only call
  set_height + align + set_style_bg_color when the value actually changed:
  ```c
  static int16_t s_spec_last_h[PROP_MIC_BANDS];
  static uint8_t s_spec_last_col[PROP_MIC_BANDS]; // 0=mute, 1=amber, 2=alert
  // In observer, replace the current loop body:
  int h = 2 + pct * SPEC_MAXH / 100;
  uint8_t bucket = pct > 85 ? 2 : (pct > 35 ? 1 : 0);
  if (h != s_spec_last_h[i]) {
      lv_obj_set_height(s_spec_bars[i], h);
      lv_obj_align(s_spec_bars[i], LV_ALIGN_BOTTOM_LEFT,
                   SPEC_X0 + i * (SPEC_BW + SPEC_GAP), -SPEC_BASE);
      s_spec_last_h[i] = h;
  }
  if (bucket != s_spec_last_col[i]) {
      lv_obj_set_style_bg_color(s_spec_bars[i],
          bucket == 2 ? COL_ALERT : (bucket == 1 ? COL_AMBER : COL_MUTE), 0);
      s_spec_last_col[i] = bucket;
  }
  ```
  Apply the same pattern to CSI (prop_ui.c:2782) and RF BAND (prop_ui.c:2739).
  - Win estimate: On a quiet spectrum (slow decay), cuts 72 calls/frame to near zero for
    unchanged bars. Under active audio reduces dirty region to only the bars that moved.
    Moderate throughput win. Confidence: High — identical to the proven scanner pattern.
- Risk: Low. Initialize shadow arrays during panel build; null in close_panel.

---

### [P3b] BLE contact list: full widget teardown/rebuild every ~400 ms
**Affects: consistency (periodic hitch on the CONTACTS screen)**

- Where: `main/prop_ui.c:2771` (`lv_obj_clean(s_ble_list)`) guarded by `tick % 8 == 0`
- Cost: `lv_obj_clean` destroys the entire child widget hierarchy of `s_ble_list` and
  the loop immediately rebuilds it via `ble_add_row`. Each row creates several child widgets
  (labels, distance badge, signal bar), each allocation going to PSRAM. The teardown forces
  a full dirty-region recomposite of the list area. For many BLE devices this is a noticeable
  hitch every ~400 ms.
- Fix (quick): Throttle to `tick % 20 == 0` (~1 Hz). BLE device lists in a room don't
  change meaningfully at 2.5 Hz; 1 Hz is imperceptible. Halves the rebuild rate immediately.
  Fix (proper): Maintain a fixed pool of row widgets pre-allocated on panel build. On each
  update, walk the device list and update labels in place; hide excess rows.
  - Win estimate: Eliminates the periodic hitch on CONTACTS. Confidence: High.
- Risk: Low. Guard stale widget pointers in close_panel (already handled by `s_ble_list = NULL`).

---

### [P3c] CRT overlay: full-screen ARGB8888 alpha-blend cost when FX is enabled
**Affects: throughput and consistency when prop_fx is on**

- Where: `main/prop_fx.c:187–193` (s_canvas: 1024×600 ARGB8888 on lv_layer_top)
- Cost: When the CRT overlay is enabled, LVGL must alpha-blend the full-screen ARGB8888
  canvas over every dirty region. Even a 4-pixel blip move on the scanner forces LVGL to
  recomposite the full dirty stripe through the ARGB8888 layer. The scrolling band
  (80px tall, ~18 fps) adds a full-width 80px dirty stripe every 55 ms independent of the
  underlying screen — on an otherwise static screen this is the only thing forcing frames.
  The overlay is off by default (NVS fx_on=0), so this cost is only active when the user
  enables it.
- Fix: A/B with `{"cmd":"fx","on":false}` vs `on:true` to quantify the overhead first.
  If meaningful: reduce s_canvas to RGB565 format with scanlines pre-baked as solid stripes
  (no per-pixel alpha needed for horizontal scanlines — they're full-width rows). This halves
  the compositing math. The scrolling band can remain ARGB8888 (it is small).
  - Confidence: Medium. Depends on whether FX is used and A/B result.
- Risk: Medium. Canvas format change requires rewriting paint_canvas to emit RGB565. Do not
  change the lv_layer_top structure — translucent top-layer is a documented dead-end.

---

### [P4] Second SW draw unit — untried experiment
**Affects: throughput (pairs with core affinity)**

- Where: `firmware/communicator/sdkconfig:4925` (`CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`)
- Cost/rationale: With core affinity pinning LVGL to core 0 (P1), core 1 runs radio tasks.
  A second SW draw unit would run a rendering thread on core 1, parallelizing the
  software rasterizer. Only worth trying after P1 — measure whether core 1 has headroom.
- Fix: After verifying P1, set `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2` in sdkconfig.defaults
  and rebuild. Measure FPS on SPECTRUM. Revert if variance increases.
  - Confidence: Low-medium. Depends on core 1 headroom.
- Risk: Low to try, easy to revert. Do not apply before P1.

---

### [P5] Partial draw buffers in internal SRAM — untried experiment
**Affects: throughput**

- Where: `bsp_illuminate.c:173–174` (`buffer_size = H_size * V_size`, `buff_spiram = true`)
- Cost: Both draw buffers are full-screen in PSRAM. Rendering into PSRAM is slower than
  into internal SRAM. For this UI, most dirty regions are small (waveform segment, blip
  move, bar height change). A partial N-line internal SRAM buffer may be faster for the
  actual dirty-region sizes produced.
- Fix: Set `buffer_size = H_size * 40` (40-line buffer) and `buff_spiram = false`. Check
  that 2 × 1024 × 40 × 2 = 163 KB fits in internal RAM (~332 KB free, but tight).
  Monitor `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` after the change.
  - Confidence: Low-medium. Measure alongside P1+P2.
- Risk: Medium. Larger dirty regions (panel transitions, FX overlay) flush more often.

---

## Suggested order of attack

**Group 1 — consistency fix (do first, measure as a unit):**
1. Pin LVGL to core 0, all background/radio tasks to core 1 (P1).
   The ~1 Hz hitch from rssi_task and the frame-time swings are fixed here.
   Minimal code change; no constraint violations.

**Group 2 — throughput fix:**
2. Hybrid allocator in lv_port_mem.c (P2).
   Targets the 8 fps → 18 fps gap on SPECTRUM. Apply after P1 so the A/B baseline is stable.
   Monitor internal RAM on VITALS screen.

**Group 3 — safe incremental wins:**
3. Bar shadow-compare for SPECTRUM, CSI, RF BAND (P3a).
4. BLE list throttle or row-pool (P3b).
   Low-risk, proven-pattern changes. Apply after P2 so each delta is measurable.

**Group 4 — experiments (measure individually):**
5. Second SW draw unit (P4) — only after P1+P2+P3 and confirmed core 1 headroom.
6. Partial draw buffers in internal SRAM (P5) — only after P1+P2 and RAM headroom check.
7. CRT overlay format optimization (P3c) — only if A/B confirms meaningful FX overhead.

---

## Measurement plan

```bash
# Confirm device is reachable
python firmware/communicator/tools/prop.py state

# Enable FPS HUD (persisted in NVS)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'

# Baseline: capture FPS on each active screen (watch HUD for 10-30 s on each)
python firmware/communicator/tools/prop.py shot spectrum_before.png --screen spectrum --wait
python firmware/communicator/tools/prop.py shot ble_before.png      --screen ble     --wait
python firmware/communicator/tools/prop.py shot csi_before.png      --screen csi     --wait
python firmware/communicator/tools/prop.py shot home_before.png     --screen home    --wait

# A/B: CRT overlay off vs on (quantify FX cost)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'
python firmware/communicator/tools/prop.py shot spectrum_nofx.png --screen spectrum --wait
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
python firmware/communicator/tools/prop.py shot spectrum_fx.png   --screen spectrum --wait

# After P1 (core affinity): recheck consistency — HUD should be stable for 30 s
python firmware/communicator/tools/prop.py shot spectrum_after_p1.png --screen spectrum --wait

# After P2 (hybrid allocator): recheck SPECTRUM fps and internal RAM
python firmware/communicator/tools/prop.py shot spectrum_after_p2.png --screen spectrum --wait
python firmware/communicator/tools/prop.py shot vitals_after_p2.png   --screen vitals  --wait
# Target: heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > 200 KB

# Verify BLE hitch is gone (watch CONTACTS screen for 30 s)
python firmware/communicator/tools/prop.py shot ble_after.png --screen ble --wait

# Input latency check after P1
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"input","control":"selector","arg":1}'
python firmware/communicator/tools/prop.py shot input_after.png --screen home --wait
```

Key metric: frame-time **consistency** (not just average). Watch the HUD over 10–30 seconds.
A stable 12 fps beats a swinging 8–18 fps for the reported complaint. Record min/max swing
as the primary consistency indicator before and after each change.
