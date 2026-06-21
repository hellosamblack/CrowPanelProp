# Communicator framerate review

## Measured baseline

Static review — not measured on hardware. No device was available during this audit.

Known baselines from the cost model and in-code comments:
- Heavy SPECTRUM screen: ~8 fps
- Mostly-static screens (HOME, VITALS, SCANNER/IDLE): ~14–18 fps
- Theoretical floor: ~4 fps for a full 1024×600 software redraw (~250 ms)
- User-reported: 10–20 fps with inconsistent frame times

The gap between 8 fps (spectrum) and 18 fps (static) is dominated by PSRAM allocation
latency during drawing (established in cost model). The swinging frame times the user most
cares about are primarily a CPU-contention/scheduler issue — all tasks are unpinned and
radio tasks can land on LVGL's core mid-render.

---

## Findings (ranked by frame-time impact)

---

### [P1] No core affinity — radio tasks interrupt LVGL mid-render
**Affects: consistency (primary user concern)**

- **Where:**
  - `bsp_illuminate.c:159` — `task_affinity = -1` (LVGL port task, unpinned)
  - `prop_engine.c:302` — `xTaskCreate(animate_task, ..., 5, NULL)` — unpinned, prio 5
  - `prop_csi.c:151` — `xTaskCreate(csi_task, ..., 4, NULL)` — unpinned, prio 4
  - `prop_ble.c:331` — `xTaskCreate(prune_task, ..., 3, NULL)` — unpinned
  - `prop_net.c:255` — `xTaskCreate(rssi_task, ..., 3, NULL)` — unpinned
  - `prop_ble.c:330` — NimBLE host task (nimble_port_freertos_init) — unpinned

- **Cost:** Every unpinned task competes for both HP cores via the FreeRTOS scheduler. When the
  NimBLE host task, CSI task, or engine task migrates to the same core as the LVGL renderer
  mid-render, that render frame stalls until the scheduler pre-empts or the interloper sleeps.
  Because the render holds the LVGL lock for the entire frame, an interloper arriving early in
  a long render can delay it by a full scheduler quantum (1 ms at 1000 Hz) or more if it is at
  equal or higher priority. Multiple such interruptions compound — hence the 18->6->18 swings.
  Input lag is the same problem: `prop_ui_input()` acquires `lvgl_port_lock()`, so any render
  stall directly delays touch/dial response.

- **Fix:** Pin LVGL to HP core 0, push radio/background tasks to HP core 1:
  - `bsp_illuminate.c`: change `task_affinity = -1` to `task_affinity = 0` (LVGL on core 0)
  - `prop_engine.c:302`: replace `xTaskCreate` with `xTaskCreatePinnedToCore(..., 1)` (core 1)
  - `prop_csi.c:151`: same, pin to core 1
  - `prop_ble.c:331`: same, pin to core 1
  - `prop_net.c:255`: same, pin to core 1
  - NimBLE host: `nimble_port_freertos_init` does not expose affinity directly. If the IDF
    NimBLE port supports `nimble_port_freertos_init_with_affinity`, pin to core 1; otherwise
    accept it as unpinned (NimBLE is event-driven and mostly sleeps).
  - The `ap_fallback_task` (one-shot, sleeps 60 s) and HTTP server tasks (esp_httpd) do not
    need pinning — they are bursty and short-lived.

- **Win estimate:** This is the textbook fix for swinging frame times on dual-core FreeRTOS.
  Expect frame-time variance to drop substantially (swings should narrow from ±12 fps to
  ±2–3 fps). Throughput improvement is secondary. Confidence: high (well-established pattern;
  all unpinned tasks confirmed from code). Needs on-device measurement to quantify.

- **Risk:** Low. The two HP cores are symmetric and equally fast. The only concern is if core 1
  becomes saturated by radio tasks under heavy BLE + CSI + WiFi load — measure consistency
  after pinning and check if CSI/BLE panels get worse throughput. If so, move the engine
  task back to core 0 (it is the lightest background task).

---

### [P2] LVGL heap fully in PSRAM — every draw allocates anti-alias masks from PSRAM
**Affects: throughput (spectrum ~8 fps vs ~18 fps static)**

- **Where:** `lv_port_mem.c:39–41` — `lv_malloc_core` / `lv_realloc_core` / `lv_free_core`
  all route to `heap_caps_malloc(..., MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)`

- **Cost:** LVGL 9 allocates per-line/per-shape anti-alias mask buffers via `lv_malloc` during
  the software render path. These are small (~1 KB), short-lived, and hit every time a bar,
  line, or shaped object is drawn. PSRAM latency on this board is ~5–10x slower than internal
  SRAM for small random accesses. On the SPECTRUM screen (24 bars × 3 LVGL calls/frame = 72
  LVGL calls + anti-alias masks) this is what drives the ~8 fps figure vs ~18 fps on mostly-
  static screens. The PSRAM allocation is also non-deterministic in timing (depends on PSRAM
  bus arbitration with other users — Wi-Fi, DMA, FB), which contributes to frame-time variance.

- **Fix:** Implement a hybrid allocator in `lv_port_mem.c`: route small/short-lived allocations
  to internal SRAM (`MALLOC_CAP_INTERNAL`), fall back to PSRAM for allocations above a size
  threshold (e.g., >2 KB) or when internal RAM is exhausted. The key constraint is that ~332 KB
  internal RAM is free at runtime with all instruments live — keep a generous headroom for
  esp_hosted SDIO (~100 KB safe reserve). A simple threshold-based split:

      #define LV_HYBRID_THRESHOLD 2048   /* small masks -> internal SRAM */
      void *lv_malloc_core(size_t size) {
          void *p = NULL;
          if (size <= LV_HYBRID_THRESHOLD) {
              p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          }
          if (!p) {
              p = heap_caps_malloc(size, LV_PSRAM_CAPS);  /* fallback: PSRAM */
          }
          return p;
      }

  Note: `lv_realloc_core` needs the same two-tier logic. The existing pointer may be in PSRAM
  (a large object was later resized). Use `heap_caps_get_allocated_size` to detect which heap
  owns the pointer, OR accept that realloc always promotes to PSRAM (safe but misses some wins).

- **Win estimate:** This is the highest-value throughput change in the cost model. Spectrum
  should improve from ~8 fps toward ~14–18 fps. Frame-time consistency also improves slightly
  by removing PSRAM-arbitration jitter from the draw hot path. Confidence: high (mechanism
  documented in cost model; internal-SRAM budget confirmed at ~332 KB). Needs on-device
  measurement to tune the threshold.

- **Risk:** Must not starve esp_hosted SDIO DMA mempool ("HS_MP: mempool create failed: no
  mem" boot loop). Monitor `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` after the change.
  If it drops below ~150 KB under load, raise the threshold or add a low-water guard.

---

### [P3] Spectrum/CSI bars: unconditional lv_obj_align every frame even when bar did not move
**Affects: throughput + minor consistency**

- **Where:**
  - `prop_ui.c:3043–3045` — SPECTRUM: `lv_obj_set_height` + `lv_obj_align` called for all 24
    bars every observer tick (20 Hz) regardless of whether height changed.
  - `prop_ui.c:3108–3110` — CSI: same pattern, 32 bars × 3 calls = 96 calls/frame.
  - `prop_ui.c:3065–3067` — RF BAND: 13 bars, same pattern (data changes less often though).

- **Cost:** `lv_obj_align` in LVGL 9 computes the new position and calls `lv_obj_set_pos`,
  which marks the object dirty if the position changed. For a bar anchored to
  `LV_ALIGN_BOTTOM_LEFT` with a fixed x offset, the x position never changes — only y changes
  with height. Calling `lv_obj_align` every frame forces LVGL to re-evaluate parent bounds even
  when neither the bar size nor the parent size changed. At 24–32 bars × 20 Hz, this is
  ~480–640 extra LVGL operations per second. Each `lv_obj_set_style_bg_color` call also runs
  a style lookup even when the color did not change.

- **Fix (preferred):** Cache the last-rendered `pct` per bar and skip all three calls when it
  has not changed. Add `static int s_spec_last_pct[PROP_MIC_BANDS]` (initialized to -1) and
  gate the update block on `pct != s_spec_last_pct[i]`. In steady state (quiet room, stable
  bars) this drops LVGL calls near zero. In active use, only changed bars get updated —
  typically a few per frame. Apply the same shadow array to CSI (`s_csi_last_pct[PROP_CSI_BINS]`)
  and RF BAND (`s_rf_last_pct[RF_CHANNELS]`).

  Alternatively: remove the per-frame `lv_obj_align` call entirely and rely on the build-time
  alignment set in `build_spectrum_panel` at `prop_ui.c:1043`. LVGL bottom-align with changing
  height recomputes y automatically. Verify this in LVGL 9.4 before removing.

- **Win estimate:** In steady state: significant reduction in redundant LVGL work, potentially
  5–10% throughput improvement. In active use: modest gain. Confidence: medium — LVGL internal
  dirty-detection may already skip unchanged positions; on-device measurement needed.

- **Risk:** Low. If the per-frame `lv_obj_align` is removed, verify bar positions after a
  panel close/reopen (the build function already aligns at open, so it should be fine).

---

### [P4] CRT overlay (prop_fx): full-screen ARGB8888 canvas forces per-frame software alpha-blend
**Affects: throughput (when FX is enabled), minor consistency**

- **Where:** `prop_fx.c:187–188` — `s_canvas`: full-screen 1024×600 ARGB8888 canvas on
  `lv_layer_top()`. `prop_fx.c:199–207` — `s_band`: 1024×80 ARGB8888 canvas, animated 18 Hz.

- **Cost:** The static canvas carries per-pixel alpha (scanlines at `FX_SCAN_OPA=95`,
  vignette at varying opacity). LVGL must software alpha-composite it over every dirty region
  of every frame while FX is on. A dirty region from one animated bar on the spectrum screen
  triggers a composite of that stripe through the full-screen ARGB8888 layer. The scrolling
  refresh band advances 2 px every 55 ms, invalidating an ~82 px tall full-width stripe 18
  times/second — forcing a full-layer recomposite of that stripe even on otherwise-static
  screens.

  The design already mitigates the worst case (thin band, not a full-screen translucent layer;
  the full-screen translucent approach is the cost-model watchdog dead-end).

- **Fix (if FX must stay on by default):** Convert the static canvas to opaque RGB565. LVGL
  blits an opaque RGB565 canvas over a dirty region without alpha math — it is a direct
  memcpy. The phosphor wash (uniform tint) can be a layer opacity on the opaque canvas (no
  per-pixel math). Only the scrolling band needs per-pixel alpha. This could halve compositing
  cost for the static overlay.

  Simpler alternative (no code change): default FX to off in NVS. The effect is opt-in
  via DISPLAY->FX OVERLAY.

- **A/B test (required to quantify):**

      curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'
      # observe FPS HUD on spectrum and scanner for ~10 s
      curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
      # compare FPS and frame-time swing

- **Win estimate:** Likely 2–4 fps on heavy screens when FX is on. Confidence: medium —
  needs the A/B to quantify. Not the dominant issue.

- **Risk:** If the static canvas is converted to opaque RGB565, the vignette cannot use
  per-pixel alpha — must approximate with stacked opaque quads. Do NOT make the top-layer
  canvas translucent at the layer level (that is the watchdog dead-end from the cost model).

---

### [P5] BLE CONTACTS panel: full lv_obj_clean + row rebuild every ~400 ms
**Affects: consistency (periodic hitch)**

- **Where:** `prop_ui.c:3075–3094` — every 8 observer ticks (~400 ms) when PK_BLE is active,
  `lv_obj_clean(s_ble_list)` tears down all rows and `ble_add_row` rebuilds up to 24 rows
  (each row = 4 label widgets + 1 meter bar = 5 `lv_obj_create` calls). Up to 120 PSRAM
  object allocations and destructions in one observer tick.

- **Cost:** `lv_obj_clean` frees all children; the loop re-allocates them from PSRAM. At 24
  contacts × 5 widgets = 120 PSRAM malloc/free cycles in a single locked observer call. This
  is a periodic 5–15 ms hitch every 400 ms on the BLE panel — visible as a frame drop.

- **Fix:** Pre-allocate a fixed pool of `PROP_BLE_MAX` row objects at panel build time. Update
  labels/colors in place on each observer tick; hide extra rows with `LV_OBJ_FLAG_HIDDEN` when
  fewer contacts are present. This turns 120 PSRAM alloc/free into ~0–5 `lv_label_set_text`
  calls per tick (most frames nothing changes).

- **Win estimate:** Eliminates the periodic hitch on the BLE panel. Average FPS unchanged but
  worst-case frame time drops. Confidence: high (mechanism is clear; fix is standard widget
  pooling). Only matters on the CONTACTS screen.

- **Risk:** Low. The fixed row pool is allocated at `build_ble_panel` and freed at
  `close_panel` — consistent with the existing one-panel-at-a-time architecture.

---

### [P6] Second SW draw unit not enabled (LV_DRAW_SW_DRAW_UNIT_CNT=1)
**Affects: throughput (conditional on P1 being done first)**

- **Where:** `sdkconfig:4925` — `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`

- **Cost:** With two HP cores, a second draw unit could let LVGL parallelise software
  rendering. Currently only one unit exists. This only helps if [P1] (core affinity) is done
  first — without pinning, a second draw unit just competes with radio tasks.

- **Fix:** After pinning LVGL to core 0 and radio tasks to core 1, change to
  `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2` in `sdkconfig.defaults` and measure on spectrum and
  scanner screens.

- **Win estimate:** Uncertain. Panel renders mostly sequential dirty regions; inter-unit
  synchronization overhead may offset the gain. Confidence: low. Propose as an experiment
  after P1 and P2 are solid, not before.

- **Risk:** The second unit spawns a FreeRTOS task. If it lands on core 0, it competes with
  LVGL and hurts consistency. Measure carefully and revert if no win.

---

### Verify: already-maxed knobs

Confirmed from `sdkconfig` — no action needed, do not re-propose:
- CPU: 360 MHz (`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_360=y`)
- PSRAM: 200 MHz HEX mode (`CONFIG_SPIRAM_SPEED_200M=y`, `CONFIG_SPIRAM_MODE_HEX=y`)
- Compiler: -O2 performance (`CONFIG_COMPILER_OPTIMIZATION_PERF=y`)
- FreeRTOS tick: 1000 Hz (from cost model)
- Refresh mode: partial (not full_refresh or direct_mode; confirmed in `bsp_illuminate.c`)
- swap_bytes: false (confirmed `bsp_illuminate.c:195`)

---

## Suggested order of attack

**Phase 1 — Consistency wins (cheapest, highest-priority)**

1. [P1] Core pinning — change 5 `xTaskCreate` calls to `xTaskCreatePinnedToCore`. One
   afternoon's work. No risk if done correctly. Measure frame-time swing with the FPS HUD
   before and after on spectrum + scanner screens. Look for narrower swing, not just average.

2. [P5] BLE row pool — pre-allocate rows, update in place. One panel, self-contained.
   Eliminates the 400 ms hitch on the CONTACTS screen.

**Phase 2 — Throughput wins**

3. [P2] Hybrid LVGL allocator — the single largest throughput lever per the cost model.
   Implement in `lv_port_mem.c`, monitor internal RAM during testing. Expect spectrum to
   improve from ~8 fps toward ~15 fps.

4. [P3] Bar update dedup — add `s_spec_last_pct[24]` and `s_csi_last_pct[32]` shadow
   arrays; skip the three LVGL calls when `pct` unchanged. Low risk, measurable win.

**Phase 3 — Optional experiments (measure together, can revert independently)**

5. [P4] FX A/B — run the A/B test to quantify FX overhead. If it is >3 fps, consider
   baking the static canvas to opaque RGB565.

6. [P6] Second draw unit — only after P1 is solid. Set `DRAW_UNIT_CNT=2`, measure
   spectrum and scanner FPS; revert if no win.

---

## Measurement plan

    # 1. Confirm device reachable
    python firmware/communicator/tools/prop.py state

    # 2. Enable FPS HUD (amber readout top-right)
    curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'

    # 3. Baseline: FX off, measure each heavy screen for ~10 s each
    curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'
    python firmware/communicator/tools/prop.py shot spectrum_baseline.png --screen spectrum --wait
    python firmware/communicator/tools/prop.py shot scanner_baseline.png  --screen scanner  --wait
    python firmware/communicator/tools/prop.py shot ble_baseline.png      --screen ble      --wait
    python firmware/communicator/tools/prop.py shot csi_baseline.png      --screen csi      --wait

    # 4. FX overhead A/B (P4 quantification)
    curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
    python firmware/communicator/tools/prop.py shot spectrum_fx.png --screen spectrum --wait
    # Compare FPS readout in spectrum_baseline.png vs spectrum_fx.png

    # 5. After [P1] core pinning: repeat step 3 and compare
    #    Look for: narrower FPS swing (18->16->18 vs 18->6->18), not just average

    # 6. After [P2] hybrid allocator: compare spectrum FPS
    python firmware/communicator/tools/prop.py shot spectrum_after_p2.png --screen spectrum --wait
    # Target: spectrum FPS should rise from ~8 toward ~15

    # 7. After [P3] bar dedup: compare spectrum + CSI FPS
    python firmware/communicator/tools/prop.py shot spectrum_after_p3.png --screen spectrum --wait
    python firmware/communicator/tools/prop.py shot csi_after_p3.png      --screen csi      --wait

    # 8. Input lag check (drive dial, watch frame reaction)
    curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"input","control":"selector","arg":"cw"}'
    # Should be visually immediate after P1 (< 1 frame lag)

---

## Static review confidence summary

| Finding | Confidence | Needs hardware confirmation? |
|---|---|---|
| P1 no core affinity | High (code-evident) | Yes: measure swing improvement |
| P2 PSRAM allocator | High (documented in cost model) | Yes: measure spectrum FPS |
| P3 bar update dedup | Medium | Yes: confirm if LVGL already dedupes |
| P4 FX canvas cost | Medium | Yes: A/B required to quantify |
| P5 BLE row churn | High (code-evident) | Yes: confirm hitch disappears |
| P6 second draw unit | Low | Yes: measure carefully after P1 |
