# Communicator framerate review

## Measured baseline

Static review — no physical device reached. Measurements quoted below come from the cost model
(`references/cost-model.md`) and in-code comments in `prop_ui.c` (~line 2940):

- Full-screen software render: ~250 ms (~4 fps)
- Heavy animation (SPECTRUM screen): ~8 fps
- Mostly-static screens: ~18 fps
- The 8 fps vs 18 fps gap is dominated by per-frame PSRAM allocation, not by pixel count.

No on-device consistency data is available in this static review. The swinging frame times the
user describes ("hitches roughly once a second") are most plausibly caused by the core-affinity
issue (P1-A below), and to a lesser extent by the BLE list rebuild hitch (P2-B).

---

## Findings (ranked by frame-time impact)

---

### [P1-A] All tasks unpinned — radio tasks land on LVGL's core mid-render
**Affects: consistency (primary), throughput (secondary)**

- **Where:** `bsp_illuminate.c:159` (`task_affinity = -1`); `prop_engine.c:302`
  (`xTaskCreate(..., NULL)` = no affinity); `prop_net.c:244,255`; `prop_ble.c:331`;
  `prop_csi.c:151`; `prop_mic.c:186` — all unpinned.
- **Cost:** The ESP32-P4 has two HP cores. With `task_affinity = -1`, FreeRTOS is free to
  schedule any task on any core at any tick. When the rssi task (~1 Hz SDIO round-trip),
  NimBLE host task (event-driven, bursty), or CSI fold task (~15 Hz with `sqrtf` per bin)
  migrates onto the same core as the LVGL render task mid-frame, the render stalls for the
  length of that task's timeslice before the scheduler can preempt it. At FreeRTOS 1000 Hz,
  one stolen timeslice = 1 ms minimum; a blocking SDIO call in rssi_task can be much more.
  This is the classic source of irregular frame-time spikes: the average looks okay but the
  tail latency hits 50-100 ms once a second whenever a radio task migrates in. It is also the
  direct cause of input lag — `prop_ui_input()` acquires `lvgl_port_lock()`, so any render
  stall blocks input acknowledgment for the same duration.
- **Fix:** Pin LVGL to HP core 0 and all radio/background tasks to HP core 1:
  - `bsp_illuminate.c:159`: change `task_affinity = -1` to `task_affinity = 0`
  - `prop_engine.c:302`: `xTaskCreate` to `xTaskCreatePinnedToCore(..., 1)` for `prop_anim`
  - `prop_net.c:244`: `xTaskCreate(ap_fallback_task, ...)` to pin to core 1
  - `prop_net.c:255`: `xTaskCreate(rssi_task, ...)` to pin to core 1
  - `prop_ble.c:331`: `xTaskCreate(prune_task, ...)` to pin to core 1
  - `prop_csi.c:151`: `xTaskCreate(csi_task, ...)` to pin to core 1
  - `prop_mic.c:186`: `xTaskCreate(mic_task, ...)` to pin to core 1
  - NimBLE host task: `nimble_port_freertos_init` spins up its own task; set
    `BT_NIMBLE_PINNED_TO_CORE` Kconfig to 1 to pin it to core 1.
- **Win estimate:** Eliminates the inter-core migration source of frame-time spikes. Expected
  to convert the "18 to 6 to 18" swings into a steady 15-18 fps on static screens. High
  confidence (this is the textbook fix for this symptom on dual-core ESP32 + LVGL).
- **Risk:** The engine observer (`prop_anim`, core 1) calls `lvgl_port_lock()` to notify the
  UI. That lock crosses cores, which is designed for exactly that use case. No known conflicts.
  Does not affect the SDIO/WiFi constraint.

---

### [P1-B] LVGL heap entirely in PSRAM — every draw pays PSRAM allocation latency
**Affects: throughput (primary); also contributes to variance via allocator contention**

- **Where:** `lv_port_mem.c:39` — `lv_malloc_core` routes all allocations to
  `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`. No internal SRAM path exists.
- **Cost:** LVGL 9 allocates anti-alias masks via `lv_malloc` during draw. Every skewed line,
  rounded corner, or AA text glyph fires a `heap_caps_malloc` into PSRAM. PSRAM (200 MHz hex
  quad, accessed over SPI) is ~5-10x slower than internal SRAM for short random-access bursts.
  The spectrum screen is the worst case: 24 bars x per-frame height-change triggers LVGL layout
  + style resolution + AA mask allocation per bar per frame -> the documented 8 fps vs 18 fps
  gap. The allocator also creates contention variance: PSRAM is shared between the display DMA
  buffers, the FX canvas (~1.8 MB), CSI/BLE buffers, and LVGL's own objects — a large
  allocation elsewhere can stall a small LVGL mask allocation in the same heap.
- **Fix:** Hybrid allocator in `lv_port_mem.c`: route small/short-lived allocations (below a
  threshold, e.g. 512 bytes) to `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT` with a PSRAM fallback;
  keep large object buffers in PSRAM. The AA mask is typically 64-512 bytes. The constraint is
  the ~332 KB internal RAM free at runtime (per CLAUDE.md); leave >= 100 KB headroom for
  esp_hosted. A 128 KB internal slab for small LVGL allocs is safe. Implementation sketch for
  `lv_port_mem.c`:

  ```c
  #define LV_SMALL_THRESH 512
  void *lv_malloc_core(size_t size) {
      if (size <= LV_SMALL_THRESH) {
          void *p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          if (p) return p;
      }
      return heap_caps_malloc(size, LV_PSRAM_CAPS);
  }
  ```
  `lv_realloc_core` needs the same threshold logic. `lv_free_core` stays `heap_caps_free`
  (works for both caps).

- **Win estimate:** The cost-model cites this as "the single biggest throughput win available."
  Expected to lift SPECTRUM from ~8 fps toward ~14-16 fps. Confidence: high (mechanism is
  documented in the cost-model and the allocator comment). Exact gain needs on-device
  measurement due to internal RAM fragmentation state.
- **Risk:** Must stay well below the ~332 KB internal free budget. Hard constraint from the
  cost-model: do NOT raise `LV_MEM` or restore the builtin pool. The hybrid allocator only
  routes small transient allocs to internal RAM; they free immediately after each frame, so
  incremental internal usage is bounded.

---

### [P2-A] SPECTRUM/CSI/RF BAND: redundant `lv_obj_align` every frame even when value unchanged
**Affects: throughput**

- **Where:** `prop_ui.c:2724-2727` (SPECTRUM loop, 24 bars), `prop_ui.c:2744-2749` (RF BAND
  loop, 13 bars), `prop_ui.c:2787-2792` (CSI loop, 32 bars).
- **Cost:** Each bar update calls `lv_obj_set_height` + `lv_obj_align` +
  `lv_obj_set_style_bg_color` unconditionally every observer tick (~20 Hz). Totals:
  - SPECTRUM: 24 bars x 3 LVGL calls = 72 calls/frame
  - RF BAND: 13 bars x 3 calls = 39 calls/frame
  - CSI: 32 bars x 3 calls = 96 calls/frame (highest)
  Even when a bar's height has not changed (e.g. a bar at its decay floor), `lv_obj_set_height`
  marks the object dirty and `lv_obj_align` triggers a layout pass and invalidation. On SPECTRUM
  with silent audio, all 24 bars invalidate every 50 ms unnecessarily. The scanner trace already
  uses a `s_wave_shadow[]` compare to gate re-renders to changed segments only (`prop_ui.c:2569`).
  The bar panels have no equivalent guard.
- **Fix:** Add a shadow-compare before each bar update, following the scanner pattern already in
  the codebase. Track last-rendered `pct` (0-100) and last color per bar:

  ```c
  static int s_spec_pct[PROP_MIC_BANDS];
  static uint16_t s_spec_col[PROP_MIC_BANDS];  /* lv_color_to_u16 cache */
  // in the observer loop per bar i:
  lv_color_t col = pct > 85 ? COL_ALERT : (pct > 35 ? COL_AMBER : COL_MUTE);
  uint16_t col16 = lv_color_to_u16(col);
  if (pct != s_spec_pct[i] || col16 != s_spec_col[i]) {
      lv_obj_set_height(s_spec_bars[i], h);
      lv_obj_align(...);
      lv_obj_set_style_bg_color(s_spec_bars[i], col, 0);
      s_spec_pct[i] = pct; s_spec_col[i] = col16;
  }
  ```
  Apply the same pattern to RF BAND and CSI bar loops.

- **Win estimate:** On a silent SPECTRUM screen, cuts from 72 to 0 LVGL calls per frame ->
  approaches the ~18 fps static-screen baseline. With active audio, wins are proportional to how
  many bars are stable. Confidence: high (direct analogy to the scanner optimization).
- **Risk:** Low. Shadow arrays are small (< 200 bytes total across all three panels). Initialize
  to -1 sentinel so the first frame always renders. No memory budget concern.

---

### [P2-B] BLE CONTACTS: full widget hierarchy teardown/recreate every ~400 ms
**Affects: consistency (periodic hitch)**

- **Where:** `prop_ui.c:2771` — `lv_obj_clean(s_ble_list)` followed by up to
  `PROP_BLE_MAX` (24) calls to `ble_add_row()`, at `st->tick % 8 == 0` (~2.5 Hz).
- **Cost:** `lv_obj_clean` tears down the entire child widget tree (up to 24 rows x ~5 widgets
  each = up to 120 widget deletions from the PSRAM heap), then `ble_add_row` rebuilds them
  (another ~120 PSRAM allocations from `lv_malloc_core`). This burst of allocator activity on
  the PSRAM heap every ~400 ms produces a measurable stall in the LVGL task. Combined with
  style resolution and layout passes for 120 newly created objects, this maps directly to the
  "hitch roughly once a second" symptom on the BLE screen.
- **Fix:** Replace teardown/recreate with a pre-allocated fixed-size row pool. During
  `build_ble_panel` (`prop_ui.c:1182`), create `PROP_BLE_MAX` row containers with their child
  label/meter widgets. In the observer, update each row's label text and style in-place; hide
  rows beyond the current device count with `LV_OBJ_FLAG_HIDDEN`. No allocation after initial
  build. This is the same structural fix that the scanner trace underwent (from full-track
  recreate to segment-compare).
- **Win estimate:** Removes a burst of ~120 PSRAM alloc/free operations every 400 ms from the
  render path. Eliminates the periodic BLE hitch entirely. Confidence: high (mechanism is clear
  and the fix is well-precedented in this codebase).
- **Risk:** Pool approach requires pre-sizing to `PROP_BLE_MAX` = 24 rows. `lv_obj_add_flag`
  for hidden rows is cheap. Net memory usage is essentially the same as the current teardown/
  recreate approach but without the churn.

---

### [P3-A] prop_fx refresh band forces a full-width stripe recomposite ~18x/s while FX on
**Affects: throughput (while FX enabled)**

- **Where:** `prop_fx.c:159-170` (`band_tick`), timer period `FX_BAND_MS = 55` ms (~18 Hz).
  Band is `FX_W x FX_BAND_H = 1024 x 80` pixels ARGB8888.
- **Cost:** Every 55 ms, `lv_obj_set_y(s_band, s_band_y)` invalidates the old-position union
  new-position stripe (~160 px tall x 1024 wide). LVGL must software-alpha-blend this ARGB8888
  canvas over the underlying screen content. At ~250 ms for a full 600-row redraw, a 160-row
  strip costs ~(160/600) x 250 = ~67 ms per tick. At 18 Hz that is ~1.2 full-frame-equivalents
  per second spent on the band alone when FX is on. This taxes SPECTRUM and CSI most severely.
  The static scanline/vignette canvas is baked and does not move, so it does not cause repeated
  recomposites on its own.
- **Fix:** Two constant changes in `prop_fx.c`:
  - `FX_BAND_H` from 80 to 40 (halves the recomposite area per tick)
  - `FX_BAND_MS` from 55 to 100 (reduces tick rate from 18 Hz to 10 Hz)
  Combined, the band cost drops to ~(80/600) x 250 x 10 Hz = ~0.33 frame-equivalents/s, a 4x
  reduction. The drift still reads as a slow CRT phosphor sweep; halving the height and slowing
  the drift is visually almost imperceptible at prop-watching distance.
- **Win estimate:** ~1-2 fps improvement on SPECTRUM/CSI screens while FX is on. Confidence:
  medium (depends on how much other work overlaps with the band tick).
- **Risk:** Pure aesthetic tradeoff. Explicitly safe per `prop_fx.c` comment ("Do NOT widen it
  to full height or speed it up much") — this change goes the opposite direction.

---

### [P3-B] Draw buffers in PSRAM — partial internal SRAM buffer experiment (untried)
**Affects: throughput (experiment)**

- **Where:** `bsp_illuminate.c:188` — `buff_spiram = true`, full-screen double buffer
  (`buffer_size = H_size * V_size = 614,400 pixels x 2 bytes = ~1.2 MB per buffer`).
- **Cost:** Rendering into PSRAM is slower than rendering into internal SRAM. Full-screen x2
  (~2.4 MB) cannot fit internal RAM, but a partial N-line buffer can. The `esp_lvgl_port`
  supports partial buffers: set `buffer_size = H_size * 40` (40 lines x 1024 x 2 = ~80 KB),
  set `buff_spiram = false`, and the port renders dirty regions in strips using internal SRAM
  as the intermediate. For the narrow dirty regions this UI actually produces (one wave segment,
  a few bar updates), partial internal SRAM rendering may beat full-screen PSRAM buffering.
- **Fix to try:** In `bsp_illuminate.c:173`, change `buffer_size` from `H_size * V_size` to
  `H_size * 40` and set `buff_spiram = false`. Rebuild and measure SPECTRUM FPS.
- **Win estimate:** Unknown — this is a genuine "try it and measure" lever. The cost-model
  lists it as an untried experiment. Could improve throughput for narrow dirty regions; could
  hurt if dirty regions routinely exceed the strip height.
- **Risk:** 80 KB from the ~332 KB internal free budget; leaves ~252 KB for esp_hosted — safe.
  If the partial buffer produces slower results (e.g. on large dirty regions during panel
  transitions), revert.

---

### [P3-C] Second SW draw unit — coupled to core affinity, measure after P1-A
**Affects: throughput (experiment)**

- **Where:** `sdkconfig` — currently `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`.
- **Cost/Fix:** Set `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2` in `sdkconfig.defaults`. MUST be
  done after P1-A (core pinning): with radio tasks unpinned, the second draw unit competes with
  them on core 0, adding variance rather than throughput. With radio tasks on core 1 and LVGL
  on core 0, the second unit can use core 0's idle capacity to parallelize render bands.
- **Win estimate:** Unknown without measurement. Could lift SPECTRUM toward 12-14 fps. Confidence:
  low — parallelization overhead on small dirty regions can negate the gain.
- **Risk:** Increases stack and memory usage for the second draw task. Measure consistency before
  and after — if the second unit introduces its own synchronization spikes, back it out.

---

## Suggested order of attack

**Step 1 (cheapest + highest consistency impact):**
Apply P1-A (core pinning). Seven one-line xTaskCreate changes + one constant in `bsp_illuminate.c`
+ one Kconfig for NimBLE. No logic changes. Rebuild, flash, watch the FPS HUD for 10+ seconds on
SPECTRUM to confirm swings become steady.

**Step 2 (highest throughput impact):**
Apply P1-B (hybrid allocator in `lv_port_mem.c`). Measure SPECTRUM FPS before/after. Watch
VITALS -> RAM to confirm internal RAM stays > 150 KB free.

**Step 3 (medium impact, quick):**
Apply P2-A (shadow-compare for bar panels, three loops in `prop_ui.c`). Measure SPECTRUM and CSI
FPS in a quiet room with minimal audio.

**Step 4 (BLE consistency):**
Apply P2-B (pre-allocated BLE row pool). Eliminates the periodic hitch on the CONTACTS screen.

**Step 5 (optional constant changes):**
Apply P3-A: reduce `FX_BAND_H` 80->40 and increase `FX_BAND_MS` 55->100 in `prop_fx.c`.

**Step 6 (experiments — measure, do not assume):**
Try P3-B (partial internal SRAM draw buffer) and P3-C (second draw unit) in that order after
P1-A is confirmed working.

---

## Measurement plan

```bash
# Prerequisite: device at comm-unit-7.local, FPS HUD on
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'

# Baseline: steady-state FPS per screen (watch HUD over 10 s on each)
python tools/prop.py shot spectrum.png --screen spectrum --wait
python tools/prop.py shot csi.png      --screen csi     --wait
python tools/prop.py shot ble.png      --screen ble     --wait
python tools/prop.py shot scanner.png  --screen scanner --wait

# CRT overlay A/B (measures prop_fx band cost on SPECTRUM)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'
python tools/prop.py shot spectrum_nofx.png --screen spectrum --wait
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
python tools/prop.py shot spectrum_fx.png   --screen spectrum --wait
# Crop the FPS readout for comparison:
python tools/prop.py shot crop.png --screen spectrum --crop 764,8,260,86 --zoom 3

# After P1-A (core pinning): re-run all screens, watch for stable vs swinging HUD.
# Key: watch the HUD number for 10 s. Stable "18" = fixed. "18->6->18" = still contending.

# After P1-B (hybrid allocator): SPECTRUM FPS should increase.
# Also verify internal RAM budget via VITALS:
python tools/prop.py shot vitals.png --screen vitals --wait

# After P2-A (shadow-compare): SPECTRUM + CSI FPS in a quiet room.
# Walk away from the device so the mic sees near-silence — bars should stabilize.
# FPS should approach the static-screen baseline (~18) when all bars are unchanged.
python tools/prop.py shot spectrum_quiet.png --screen spectrum --wait

# After P2-B (BLE row pool): verify no hitch on BLE screen over 30+ seconds.
python tools/prop.py shot ble_after.png --screen ble --wait

# Input latency spot-check (compare before/after P1-A):
time curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"input","control":"selector","arg":"cw"}'
```

