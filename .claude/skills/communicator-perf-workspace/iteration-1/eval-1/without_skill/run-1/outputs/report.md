# Communicator framerate review — SPECTRUM screen stutter

## TL;DR (the actual answer)

The spectrum screen is slow for **two compounding reasons**:

1. **Throughput / why it's ~8 fps:** This board has no GPU/PPA on chip rev v1.3, so LVGL renders
   everything in software (a full-screen redraw is ~250 ms). LVGL 9 allocates anti-alias draw
   masks via `lv_malloc` *during* drawing, and the LVGL heap is routed entirely to PSRAM
   (`lv_port_mem.c`). So every shape draw on a busy screen pays PSRAM allocation latency. The
   static home screen barely draws, so it sits at ~18 fps; the spectrum screen redraws 24 tall
   bars every frame and pays that allocation tax 24x a frame. This is the documented baseline gap
   and the named highest-value lever (hybrid allocator) is **not yet implemented**.

2. **The stutter specifically:** the spectrum observer code is the one instrument that does **not**
   follow the change-gated update discipline the rest of the UI uses. It unconditionally rewrites
   height + alignment + bg-color on all 24 bars every ~20 Hz frame, which forces a large invalidate
   every frame even when a bar didn't move. On top of that, the mic FFT task spins up only on this
   screen (~62 FFT/s, prio 5, unpinned) and lands on whichever core LVGL is rendering on — so frame
   times *swing* rather than sitting at a steady low number. That swing is what reads as "stutter."

This is a **static code review** — I do not have the device, so per-screen FPS is not measured live.
Findings are marked code-evident vs. needs-measurement.

## Measured baseline

Static review — not measured on hardware. Known from the cost model / firmware CLAUDE.md:
spectrum ~8 fps, static screens ~18 fps; full-screen software render ~250 ms.

## Findings (ranked by frame-time impact; consistency first per stated priority)

### [P1] Spectrum bars are rewritten every frame with no change-gating
- **Affects:** both (consistency + throughput), and is the cheapest high-value fix.
- **Where:** `main/prop_ui.c:2690-2704` (`ui_observer`, the
  `s_cur_kind == PK_SPECTRUM` block).
- **Cost / mechanism:** Each of the 24 bars (`PROP_MIC_BANDS = 24`,
  `main/include/prop_mic.h:20`) gets, every ~20 Hz tick, unconditionally:
  - `lv_obj_set_height()` (2699)
  - `lv_obj_align()` (2700-2701)
  - `lv_obj_set_style_bg_color()` (2702-2703)

  That's ~72 LVGL mutating calls/frame. `lv_obj_set_height` and `lv_obj_align` each invalidate the
  **union of the old and new bounding box** of the object — so a bar that grew then shrank dirties a
  tall amber column. With 24 bars spread across the panel width, the per-frame invalidated area
  approaches the full bar field. Re-issuing `align` every frame (even when x never changes — only
  height does) and re-setting `bg_color` (even when the color bucket didn't change) forces redraws
  that produce no visible difference. Software rendering cost is ~linear in invalidated pixels, and
  each redrawn bar also pays the PSRAM mask-alloc tax (see P2).

  Contrast: the scanner trace right above this (`prop_ui.c:2540-2606`) is meticulously
  change-gated — a 10-segment shadow-compare on the waveform plus per-write `if (changed)` /
  `if (col != s_last_col)` guards, with a comment that the gating took the trace "from ~14 fps
  (full-track redraw) toward 60." Spectrum got none of that treatment.
- **Fix (code-evident):** Shadow-compare each bar against its last drawn state and skip writes that
  don't change anything:
  - Keep a per-bar `int16_t s_spec_last_h[PROP_MIC_BANDS]` and only call `lv_obj_set_height` when
    the new height differs (by >=1 px). When you change height, you do **not** also need
    `lv_obj_align` every frame — the bars are bottom-aligned (`LV_ALIGN_BOTTOM_LEFT`) and the x
    offset never changes, so the alignment is fixed at build time. Setting height alone keeps the
    bottom edge anchored; drop the per-frame `lv_obj_align` entirely (it's redundant with
    `BOTTOM_LEFT` alignment and is a second invalidate per bar).
  - Track the last color *bucket* (mute/amber/alert) per bar and only call
    `lv_obj_set_style_bg_color` when the bucket flips.
  - Optionally quantize height to a few px (e.g. round to 4 px) so micro-jitter from the FFT doesn't
    trigger a redraw every frame on otherwise-steady bars.
- **Win estimate:** High confidence this removes a large share of per-frame invalidations on steady
  audio and most of the redundant work on moving audio. Eliminating the per-frame `lv_obj_align`
  alone roughly halves the invalidations. This is the single most direct fix for the stutter and
  costs nothing structurally.
- **Risk:** None against the hard constraints. Pure UI-thread change. Make sure decay still drives
  visible motion (it will — the decay math is unchanged, only the *write* is gated).

### [P1] No core pinning — mic FFT + radio tasks contend with LVGL mid-render
- **Affects:** consistency (the swinging frame times — the user's #1 complaint).
- **Where:** all tasks are unpinned (`task_affinity = -1` for the LVGL port in `bsp_illuminate.c`;
  plain `xTaskCreate` with no `PinnedToCore` for: `mic_task`
  `main/prop_mic.c:186` (prio 5, ~62 FFT/s), engine animate_task
  `prop_engine.c:302` (prio 5, 20 Hz), `rssi` `prop_net.c:240`, `ble_prune` `prop_ble.c:331`,
  `prop_csi` `prop_csi.c:151` (prio 4)).
- **Cost / mechanism:** The mic FFT task only runs while the spectrum screen is open (it's the data
  source for the bars). It does real DSP at prio 5 — the same priority as the engine task that
  drives the observer. With no affinity, FreeRTOS migrates it freely; when it lands on the HP core
  LVGL is rendering on, the in-progress software frame stalls until the FFT yields. That is the
  classic cause of the 18->6->18 swing, and it explains why the stutter shows up *specifically* on
  the spectrum screen (the FFT task is dormant elsewhere).
- **Fix (plausible, needs on-device measurement):** Pin LVGL to one HP core and push the
  background/radio/DSP tasks (mic FFT, engine, rssi, ble_prune, csi) to the other core via
  `xTaskCreatePinnedToCore`. Set the LVGL port `task_affinity` in `bsp_illuminate.c` to a fixed core
  (e.g. core 0) and pin the rest to core 1. This is the textbook fix for inconsistent frame times on
  a dual-core software-rendered setup.
- **Win estimate:** Medium-high confidence on consistency; should largely remove the swing. Smaller
  effect on average FPS. Measure before/after with the FPS HUD on the spectrum screen over ~10 s.
- **Risk:** Pinning all radio/SDIO work onto one core could throttle BLE/WiFi/CSI throughput if that
  core saturates; verify the radio instruments still update. Do not let the pinned background core
  starve esp_hosted's SDIO servicing. Re-measure CONTACTS/SIGNAL ENV after pinning.

### [P2] LVGL draw-mask allocations come from PSRAM (the ~8 vs ~18 fps gap)
- **Affects:** throughput (raises the spectrum ceiling).
- **Where:** `main/lv_port_mem.c` (custom `lv_malloc` -> PSRAM, via
  `CONFIG_LV_USE_CUSTOM_MALLOC`).
- **Cost / mechanism:** Documented in the firmware CLAUDE.md "Memory reality" and the cost model:
  LVGL 9 allocates line/shape anti-alias masks via `lv_malloc` during drawing. Routing the whole
  LVGL heap to PSRAM means every bar/shape draw pays PSRAM allocation latency, and PSRAM contention
  also adds frame-time variance. This is named as *the* reason spectrum is ~8 fps vs ~18 static.
- **Fix (named lever, not yet done):** Implement a **hybrid allocator** in `lv_port_mem.c`: route
  small / short-lived allocations to internal SRAM (`MALLOC_CAP_INTERNAL`) and fall back to PSRAM
  only for large buffers. Keep generous internal-RAM headroom for esp_hosted.
- **Win estimate:** Per the cost model this is usually the single biggest throughput win available
  for the heavy screens. Confidence medium-high that it materially closes the 8->18 gap, but it
  needs on-device measurement and careful RAM budgeting.
- **Risk (must cross-check):** Internal RAM is needed by esp_hosted's SDIO DMA mempool — there's
  ~332 KB internal free at runtime with all radio instruments live. The hybrid allocator must stay
  well under that or it triggers the "HS_MP: mempool create failed: no mem" boot loop. **Do NOT**
  restore the builtin pool or raise `LV_MEM` — both are settled regressions. This change is higher
  risk than P1 and should be done with a size cap and a fallback path, and measured.

### [P3] dB label/meter update is already throttled — leave it
- **Where:** `prop_ui.c:2705-2708` — `st->tick % 4 == 0` gates the dB text + meter to ~5 Hz.
- **Note:** This is correct and already cheap; no change needed. Mentioned so it isn't "fixed"
  redundantly. The cost is in the 24 bars, not the dB readout.

### [P3] Bars are 24 individual objects — possible structural follow-up
- **Where:** `prop_ui.c:1016-1025` (build), `2693-2704` (update).
- **Note:** 24 separate `lv_obj`s each invalidate independently. A future option (only if P1 doesn't
  get it smooth enough) is to draw the whole bar field into a single canvas/custom-draw object so one
  invalidate covers the field with one mask allocation, instead of 24. This is a larger rewrite;
  treat it as a fallback, not a first move. Lower confidence it's needed once P1+P2 land.

## Suggested order of attack

1. **P1 — change-gate the spectrum bars** (`prop_ui.c:2690-2704`). Cheapest, no risk, directly
   targets the stutter. Drop the per-frame `lv_obj_align`, shadow-compare height and color bucket.
   Measure first — this alone may make it acceptable.
2. **P1 — core pinning** (mic/engine/radio -> one core, LVGL -> the other). Targets the *swing*.
   Measure consistency over ~10 s. Pair this with evaluating `LV_DRAW_SW_DRAW_UNIT_CNT=2` (second
   draw unit only helps if the second core isn't saturated — so decide it together with pinning).
3. **P2 — hybrid allocator** in `lv_port_mem.c`. Highest throughput ceiling but highest risk
   (esp_hosted RAM). Do last, with size cap + measurement, watching for the mempool boot loop.

Group 1 first and re-measure before doing 2 and 3 — if gating the bars already lands a stable
frame time, you may not need to touch the allocator at all.

## Measurement plan (A/B each change with the on-device FPS HUD)

```bash
# confirm device reachable
python tools/prop.py state

# turn on the FPS HUD (top-right amber readout; also drops refresh period to 8 ms)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'

# capture the heavy screen and the baseline static screen
python tools/prop.py shot spectrum.png --screen spectrum --wait
python tools/prop.py shot home.png     --screen home     --wait
```

For each change:
- Open `spectrum`, watch the HUD for ~10 s with steady ambient sound and with a varying sound
  (clap/voice). Record both the **average** FPS and the **swing** (min->max). The swing is the
  number that matters for the stutter.
- After P1 (bar gating): expect the swing to shrink and average to rise on steady audio.
- After P2 (pinning): expect the swing to shrink further (fewer whole-frame stalls from the FFT
  task), especially under varying sound when the FFT task is busiest.
- After P3 (hybrid allocator): expect the average ceiling to rise; **also** re-open `ble` and `csi`
  and confirm they still populate (RAM/contention regression check), and watch the boot log for
  `HS_MP` mempool errors.

## Constraints cross-checked (none of the above violates these)

- P2 keeps the LVGL heap mostly in PSRAM with only small allocs internal — does not restore the
  builtin pool or raise `LV_MEM`, and must leave esp_hosted internal-RAM headroom.
- No change to `CONFIG_ESP32P4_*REV*`, `swap_bytes`, or the PPA/GPU path.
- All bar updates remain on the LVGL task under the existing `lvgl_port_lock` in `ui_observer`
  (`prop_ui.c:2487`); no WiFi/SDIO calls are added under the lock.
