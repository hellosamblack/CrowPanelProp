# Framerate Investigation — Findings & Next Steps

**Date:** 2026-06-20
**Status:** Investigation complete; 60fps NOT achieved (architecture-bound). Genuine
CPU-saving wins kept. This documents what was measured so the next attempt starts informed.
**Trigger:** Goal "when the FPS meter is active, refresh as fast as possible, never below 60fps."

## TL;DR

60fps is **not reachable cheaply** on this board. It is limited by the **display
pipeline** (full-screen double-buffered rendering) and **CPU contention** with the
Wi-Fi/esp_hosted stack on the shared core — **not** by the UI content or the engine
rate. Three content/render optimizations were tried; none moved the ~13fps. The FPS
HUD, the lifted refresh cap, and per-frame style gating were kept (the last is a real
CPU win); everything else about "60fps" is deferred to a focused effort.

## Hard measurements (via display `monitor_cb`, shown on the HUD)

| Condition | Result |
|-----------|--------|
| Forced full-screen redraw every frame (`lv_obj_invalidate(lv_scr_act())`) | **FPS 4 / ~250 ms/frame**, CPU saturated |
| Scanner (waveform animating), baseline | **FPS 13, 72 ms avg, 75 ms pk** |
| Scanner, after waveform split into 10 segments (redraw only changed slice) | **FPS 13, 73 ms avg** — no change |
| Scanner, after gating every per-frame style/text write on actual change | **FPS 13, 77 ms avg** — no change |
| User observation, idle screen | **FPS swings 10 → mid-30s with nothing changing** |

## What we ruled OUT as the bottleneck

- **UI content / object count** — segmenting the waveform and gating the big-headline
  re-style (both confirmed to reduce *what* invalidates) changed render time by ~0ms.
- **Engine animation rate** — the engine runs at 20Hz (`ANIM_PERIOD_MS=50`), writing
  one wave sample/tick. Raising it can't help when each frame is render-bound at ~73ms.
- **Refresh-period cap** — already lifted from 30ms (33fps) to 8ms when FPS active;
  did not raise achieved fps.

## What the evidence points TO

1. **Full-screen double-buffered rendering.** `bsp_illuminate.c` configures
   `buffer_size = H_size * V_size` (full screen) with `double_buffer = true`. With two
   full-screen buffers, per-frame cost is largely fixed regardless of dirty-region size,
   and esp_lvgl_port may render/sync per buffer — so `monitor_cb` can fire ~2× per
   logical frame (explains FPS readings >20 from a 20Hz engine, e.g. 35).
2. **CPU contention with Wi-Fi.** LVGL renders on CPU0 alongside esp_hosted SDIO +
   LWIP + mDNS + the RSSI poll. Those spike periodically and steal CPU → frame-time
   jitter → the observed 10→35fps swing with a static screen.
3. The flush is already DMA (MIPI-DSI); the panel is ~60Hz, so the panel is not the cap.

## Genuine wins kept (committed)

- **Per-frame style/text gating** in `ui_observer` (PK_NONE block): the blip, headline
  color/font, channel marker, and SENS meter now only write when their value changes.
  Cuts wasted invalidation + CPU even though it didn't lift the fps ceiling.
- **Waveform segmentation** (`WAVE_SEGS` lines indexing the shared point buffer): only
  the changed slice re-renders. Neutral on fps here but correct and cheaper on big
  changes (ALERT/SENS).
- **FPS HUD** (`Setup ▸ Display ▸ FPS METER`, `/cmd {"fx":{"fps":true}}`), opaque
  on-screen child (the translucent `lv_layer_top` version caused the WDT crash). It is a
  **passive counter** — it does not force a faster refresh.

## Tried and reverted

- **Dropping the refresh period 30ms→8ms while FPS active.** Lifted no fps (pipeline-
  bound) and made the device network-sluggish (heavy CPU contention during the extra
  wakes). Removed; FPS-on is now lightweight.

## Next steps (in rough ROI / risk order)

1. **Task affinity + priority (medium risk, likely best ROI for *smoothness*).** Pin
   the LVGL port task to the core *away* from esp_hosted/Wi-Fi, and/or raise its
   priority, so render frame-times stop jittering. Goal: *stable* frame times (smooth),
   not necessarily 60. Investigate `esp_lvgl_port` task core/priority config and the
   esp_hosted task core. Measure jitter before/after with the HUD.
2. **Display buffer mode (high risk, highest fps potential).** Move off full-screen
   double-buffer to **partial buffers** (e.g. 1/10-screen bounce buffers) or a tuned
   **direct/`full_refresh=false`** mode so frame cost scales with dirty-region size.
   This is the fragile path — the display init already has crash history (null-screen on
   registration; the translucent-overlay WDT). Change one flag at a time, verify boot +
   `/screenshot` + no WDT after each.
3. **PPA for fill/blit-heavy work only (bounded).** Per
   `2026-06-20-ppa-acceleration-research.md`: PPA accelerates fills/blends/blits, not
   text/lines, so it won't raise general UI fps, but it *is* a clean win for the
   `prop_fx` transitions and any future image-heavy screen. Pursue independently of fps.

## Verification harness for the next attempt

- The FPS HUD (`Setup ▸ Display ▸ FPS METER`) is the live meter. For frame-time detail,
  re-add the `monitor_cb` `time_ms` avg/peak readout that was used here (was reverted to
  keep the shipping HUD clean — see git history of `prop_ui.c` fps_tick).
- `python tools/prop.py shot out.png --screen scanner --wait` captures the HUD (it's an
  on-screen child, so it *is* in `/screenshot`, unlike `lv_layer_top` overlays).
- Watch the task WDT over serial (`COM7`, 115200) for ≥15s after any display-path change;
  the failure mode here is a single `lv_timer_handler` >5s, not a reboot loop.
