# Communicator framerate review — SPECTRUM screen

## What's going on (short answer)

The home screen is essentially static, so LVGL invalidates almost nothing and the
software renderer coasts (~18 fps measured historically). The SPECTRUM screen does the
opposite: every ~50 ms the engine observer mutates **all 24 bars** (height + align +
bg-color, unconditionally), each mutation invalidates a tall rectangle, and the LVGL 9
software renderer then redraws those stripes — allocating its anti-alias / draw masks
from **PSRAM** on every draw because the LVGL heap is routed entirely to PSRAM. That
per-draw PSRAM allocation latency is the documented reason spectrum runs ~8 fps vs ~18
static. The *stutter* on top of that is CPU contention: LVGL and all the radio/engine
tasks run unpinned, so when a Wi-Fi/BLE/CSI task lands on LVGL's core mid-render a whole
frame stalls — that's the swing you see as "bars stutter."

So it is two distinct problems, in the user's priority order:
1. **Consistency (stutter):** no core pinning — radio tasks collide with the renderer.
2. **Throughput (crawl):** PSRAM draw-mask allocation + per-bar redraw work every frame.

## Measured baseline

**Static code review — not measured on hardware.** No device available in this
environment (no `python` / no `comm-unit-7.local` to run `prop.py state` or toggle the
FPS HUD), so I could not capture live per-screen FPS or frame-time swings. Findings below
are tagged **code-evident** vs **needs on-device confirmation**. The expected numbers
(~8 fps spectrum, ~18 fps static) come from the firmware's own cost model / CLAUDE.md, not
from a measurement I took.

The measurement plan at the end is what to run on real hardware to A/B each fix.

## Findings (ranked by frame-time impact; consistency first)

### [P1] No core affinity — radio tasks stall the renderer (the stutter)
- **Affects:** consistency (this is the cause of the "bars stutter")
- **Where:**
  - `peripheral/bsp_illuminate/bsp_illuminate.c:159` — LVGL port `task_affinity = -1`
  - `main/prop_engine.c:302` — `animate_task` (the observer that drives the bars), unpinned
  - `main/prop_csi.c:151`, `main/prop_ble.c:331`, `main/prop_net.c:240`,
    `main/prop_mic.c:186` — CSI / BLE-prune / RSSI-poll / mic-FFT tasks, all unpinned
- **Cost / mechanism:** With everything at `tskNO_AFFINITY`, FreeRTOS is free to schedule a
  radio/SDIO task onto whichever HP core LVGL is rendering on. A software full-stripe redraw
  is not preemptible into anything useful — the frame's wall-clock time balloons whenever it
  shares a core, producing exactly the intermittent 18->6->18-style swing the user reports.
  On SPECTRUM this is worst because the mic-FFT task (prio 5, same as the observer) and the
  CSI task are both chatty.
- **Fix:** Pin LVGL to one HP core and push background/radio tasks to the other.
  - LVGL port: `bsp_illuminate.c:159` `task_affinity = 0`.
  - Move the heavy/chatty producers to core 1: pass core 1 via `xTaskCreatePinnedToCore`
    for `prop_anim` (engine/observer), `prop_mic`, `prop_csi`, `prop_ble` prune, `rssi`,
    and the on-demand scan tasks. The observer itself does its LVGL work under the lock, so
    keep it off LVGL's core so its *non-LVGL* work (FFT read, decay math) doesn't compete.
  - Win: this is the single highest-value **consistency** fix — should largely remove the
    sporadic hitches even before throughput improves. Confidence: high on the mechanism,
    **needs on-device confirmation** of the exact win and of which task placement is best.
- **Risk:** Pinning the wrong producer onto LVGL's core makes it worse; A/B with the HUD.
  Does not violate any hard constraint. Pair this decision with the `DRAW_UNIT_CNT=2`
  experiment below (they compete for the second core).

### [P2] LVGL heap is 100% PSRAM — draw masks allocated from PSRAM every frame (the crawl)
- **Affects:** throughput (this is *the* reason spectrum is ~8 fps vs ~18 static)
- **Where:** `main/lv_port_mem.c:39` — `lv_malloc_core()` unconditionally
  `heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)`
- **Cost / mechanism:** LVGL 9 allocates line/shape anti-alias mask buffers via `lv_malloc`
  *during drawing*. With the heap entirely in PSRAM, every bar redraw pays PSRAM allocation
  + access latency, and PSRAM contention also adds frame-time variance. 24 bars redrawn each
  frame multiplies that penalty.
- **Fix (the named lever):** implement a **hybrid allocator** in `lv_port_mem.c` — route
  small / short-lived allocations to internal SRAM (`MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT`)
  with a PSRAM fallback for large buffers (e.g. threshold a few KB). Most draw masks are
  small and short-lived, so they land in fast internal SRAM; big object/buffer allocations
  still go to PSRAM. This is generally the biggest single **throughput** win available.
  Win: plausibly closes much of the 8->18 gap on spectrum. Confidence: **code-evident that
  the bottleneck exists; the size of the win needs on-device confirmation.**
- **Risk (must respect):** Internal RAM is needed by esp_hosted's SDIO DMA mempool — there's
  only ~332 KB internal free at runtime with all radio instruments live. The hybrid allocator
  **must** keep a comfortable internal headroom and fall back to PSRAM under pressure.
  **Do NOT** restore the LVGL builtin pool or raise `LV_MEM` — that starves esp_hosted into
  the "HS_MP / mempool: no mem" boot loop (hard constraint). If Wi-Fi RAM gets tight, the
  lever is `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`, not shrinking internal reserves.

### [P3] SPECTRUM bars are restyled and re-aligned every frame with no shadow-compare
- **Affects:** throughput (and some avoidable invalidation)
- **Where:** `main/prop_ui.c:2690-2704` (`ui_observer`, the `PK_SPECTRUM` block)
- **Cost / mechanism:** For all 24 bars, every frame the code calls `lv_obj_set_height`,
  `lv_obj_align`, and `lv_obj_set_style_bg_color` — ~72 LVGL calls/frame — regardless of
  whether the value changed. `lv_obj_align` is recomputed every frame even though each bar's
  X position is fixed (only its height changes), and `set_style_bg_color` re-invalidates even
  when the color bucket (mute/amber/alert) didn't change. Each height/color change invalidates
  the bar's bounding box; redundant ones still cost an invalidate + redraw pass. The scanner
  trace already avoids this with a 10-segment shadow-compare (`WAVE_SEGS`); spectrum doesn't.
- **Fix:**
  - Drop the per-frame `lv_obj_align`. The bars never move horizontally and they're
    `LV_ALIGN_BOTTOM_LEFT`, so changing height alone keeps them anchored — set position once
    in `build_spectrum_panel` (where it's already done at `prop_ui.c:1020`) and never re-align
    in the observer.
  - Shadow-compare each bar's last height and last color bucket; only call `set_height` /
    `set_style_bg_color` when they actually change (cache in small static arrays alongside
    `s_spec_decay`). With a slow decay many bars are unchanged frame-to-frame.
  - Win: cuts LVGL call count and, more importantly, cuts redundant invalidations ->
    less area redrawn per frame. Modest-to-moderate throughput gain; pairs well with P2.
    Confidence: code-evident that the work is redundant; size **needs confirmation**.
- **Risk:** Low. Keep the rise-instant/decay-slow ballistics intact (only skip work when the
  computed height/color is unchanged). The bands read is already a benign race (`prop_mic.c:199`).

### [P3] SPECTRUM refresh cadence is 20 Hz — likely faster than the camera needs
- **Affects:** throughput
- **Where:** `main/prop_engine.c:15` (`ANIM_PERIOD_MS = 50` -> 20 Hz observer);
  spectrum block runs every observer tick (`prop_ui.c:2690`)
- **Cost / mechanism:** The bars are redrawn 20x/s. HOME/VITALS already throttle with
  `tick % N`; the dB readout in this same block already throttles (`tick % 4`,
  `prop_ui.c:2705`) but the bars themselves do not. On camera, 8-10 Hz bar motion is
  indistinguishable from 20 Hz, and halving the redraws halves the per-frame cost directly.
- **Fix:** gate the bar update with `st->tick % 2 == 0` (-> 10 Hz) and re-check on hardware
  whether it still looks alive. Cheapest possible throughput win; reversible one-liner.
  Confidence: medium — depends on the look the user wants; **confirm visually on-device.**
- **Risk:** Too aggressive a throttle makes the analyser look steppy; tune N by eye.

### [P3 / experiment] Second SW draw unit to use both HP cores
- **Affects:** throughput
- **Where:** `sdkconfig:4925` `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`
- **Cost / mechanism:** Only one core renders today. A second draw unit can parallelize the
  software rasterization across both HP cores.
- **Fix:** try `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2`. **Coupled to P1** — it only helps if the
  second core isn't saturated by the radio tasks, so decide it together with core affinity and
  measure both arrangements. Confidence: experiment, **needs measurement.**
- **Risk:** competes with radio tasks for core 1; could increase variance if those aren't
  pinned sensibly. Measure consistency, not just average FPS.

## Verified, not findings (don't churn these)

- `full_refresh` is **off** (`bsp_illuminate.c:197-201`; `CONFIG_DISPLAY_LVGL_FULL_REFRESH`
  not set) — partial refresh is active, as intended. Good.
- `swap_bytes=false` (`bsp_illuminate.c:195`) — keep it; `true` paints the background magenta.
- CPU 360 MHz, PSRAM 200 MHz, `-O2`, FreeRTOS 1000 Hz — all confirmed at the tuned ceiling
  (`sdkconfig`); no headroom worth chasing there.
- Dropping the refresh period was already tried and lifted nothing — not re-proposed.
- The observer correctly **skips the SCANNER readout** when a panel is up (`prop_ui.c:2498`),
  and the FPS HUD is an opaque child (not `lv_layer_top`) — both already correct.
- `prop_net_get_rssi()` reads a cache (`prop_ui.c:2506`) — no WiFi/SDIO under the LVGL lock
  on the spectrum path. Good.

## Suggested order of attack

Cheapest-high-impact first; the user's priority is consistency, then throughput:

1. **P1 core affinity** (consistency) — pin LVGL to core 0, radio/engine/mic/csi to core 1.
   Biggest stutter fix, no constraint risk. Measure consistency first.
2. **P3 spectrum observer cleanup** (`prop_ui.c:2690`) — remove per-frame `lv_obj_align`,
   add height/color shadow-compare. Cheap, local, throughput + less invalidation.
3. **P3 cadence throttle** — `tick % 2` on the bars. One line; confirm it still looks alive.
4. **P2 hybrid allocator** (`lv_port_mem.c`) — the big throughput lever, but the riskiest
   (internal-RAM budget). Do it after the cheap wins so you can isolate its effect, and watch
   `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` stays well above the esp_hosted budget.
5. **P3 experiment `DRAW_UNIT_CNT=2`** — only after P1; A/B both core arrangements.

Group **1 + 5** in one measurement pass (they interact). Measure **2, 3, 4** individually.

## Measurement plan (run on real hardware — I could not)

```bash
# Confirm device reachable, then turn on the on-device FPS HUD (top-right amber readout)
python firmware/communicator/tools/prop.py state
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'   # toggles FPS HUD

# Baseline each screen BEFORE any change
python firmware/communicator/tools/prop.py shot spectrum_base.png --screen spectrum --wait
python firmware/communicator/tools/prop.py shot home_base.png     --screen home     --wait
# Watch the HUD on spectrum for ~10 s: record steady FPS AND the swing (e.g. 18->6->18).
```

For each change, rebuild/flash with the communicator-ui loop (`tools/dev.sh bf` /
`dev.ps1 bf`), then re-capture spectrum with the HUD on and record:
- **Steady-state FPS** on spectrum (throughput).
- **Frame-time consistency** — watch the HUD ~10 s; note min/max swing, not just average.
  This is the number that matters most for the stutter complaint.
- **Touch/dial latency** — drive `{"cmd":"input",...}` while on spectrum and watch reaction
  time (input is handled under the LVGL lock, so render stalls show up here too).

A/B notes:
- **P1 affinity / P3 DRAW_UNIT_CNT=2:** measure all four combos
  (1 unit vs 2, pinned vs current) and compare *consistency* first, FPS second.
- **P2 allocator:** after flashing, also log
  `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` at runtime and confirm it stays well above
  the esp_hosted budget (boot must not loop on "HS_MP: mempool ... no mem").
- Leave the HUD a *passive* counter — do not force faster refresh (proven to lift nothing).
