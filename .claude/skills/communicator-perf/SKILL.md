---
name: communicator-perf
description: Audit the CrowPanel communicator prop firmware (repo root) for performance hogs that drag down LVGL display framerate and touch/dial responsiveness, and that make frame times inconsistent. Use whenever the user mentions the prop feeling slow, sluggish, janky, stuttery, low FPS, choppy animation, laggy touch/dial, uneven/inconsistent frame times, or asks to profile/optimize/speed up rendering on this board. Produces a prioritized findings report (impact-ranked, file:line, concrete fix) and a measurement plan — it does not blindly apply changes.
---

# Communicator performance review

This board renders **everything in software** on a 1024×600 RGB565 panel (no GPU/PPA on chip
rev v1.3). That single fact dominates the whole problem: a full-screen redraw is ~250 ms (~4 fps),
so framerate and responsiveness are won or lost by **how much area is redrawn, where the pixels and
draw-masks live, and who else is stealing the CPU**. Your job is to find the specific code that
violates those constraints, rank fixes by frame-time impact, and tell the user what to change —
grounded in real `file:line`, not generic LVGL advice.

The user's stated priority order: **(1) consistent frame times, (2) higher framerate.** Weight your
ranking accordingly — a fix that removes a sporadic 50 ms hitch can matter more than one that adds
2 fps to the average, because stutter is what reads as "broken" on camera and under the dial.

## Before you start: read the cost model

Read `references/cost-model.md`. It captures what's already known and measured about this exact
firmware (the ~250 ms full-screen number, the PSRAM-allocator penalty, dead ends that were already
tried and lifted nothing, and the hard "do not break" constraints). **Do not re-derive these or
recommend things that were already proven not to work** — that wastes the user's time and credibility.
If a finding contradicts the cost model, say so explicitly and explain why you think it's different now.

## Measure first, then read code

Guessing at perf is how you end up "optimizing" the wrong thing. Get numbers before and as you go.
The firmware has a built-in FPS HUD; turn it on and walk the screens.

```bash
# turn on the on-device FPS meter (top-right amber readout), then capture each screen
python tools/prop.py state                                    # confirm device is reachable (mDNS comm-unit-7.local)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'          # enables FPS HUD — calls prop_ui_set_fps() (prop_api.c:117)
# NOTE: {"cmd":"fx","on":true} is a SEPARATE toggle — it enables the CRT overlay (prop_fx), not the HUD
python tools/prop.py shot spectrum.png --screen spectrum --wait
python tools/prop.py shot scanner.png  --screen scanner --wait
python tools/prop.py shot ble.png      --screen ble     --wait
```

What to record per screen, and why it matters for *this* review:

- **Steady-state FPS** of each instrument (scanner, spectrum, rfband, csi, ble, vitals, home). The
  heavy ones are the ones doing per-frame widget work — they're where the throughput wins are.
- **Frame-time consistency**, not just the average. Watch the HUD over ~10 s on a moving screen: does
  it sit at a stable number or swing (e.g. 18→6→18)? Swings are the priority-1 problem. The usual
  causes are lock contention (whole frames skipped), allocator spikes, and CPU contention from the
  Wi-Fi/BLE/CSI tasks landing on the same core as LVGL.
- **Touch/dial latency**: drive `{"cmd":"input",...}` and watch how long the screen takes to react.
  Input is handled under the LVGL lock, so anything that holds that lock long (or any draw stall)
  shows up here too.

If you can't reach hardware, say so and do a static review — but make clear which findings are
code-evident vs. which need on-device confirmation.

## The hunt: where the frame time actually goes

Work through these in order. Each is "look here → why it costs → the lever." Cite `file:line` for
every finding. The big four are redraw area, pixel/mask memory location, CPU contention, and
per-frame widget churn — most real wins live there, not in micro-optimizations.

### 1. Redraw area & invalidation discipline (biggest throughput lever)
Software render cost is ~linear in invalidated pixels. A full-screen invalidate is ~250 ms; a small
region is cheap. Hunt for code that dirties more than it needs to:

- **CRT overlay alpha-composite (`prop_fx.c`) — check this first.** `prop_fx` places a full-screen
  ARGB8888 canvas on `lv_layer_top()`. Its pixels carry per-pixel alpha < 255 (scanlines, vignette,
  phosphor wash), so LVGL must software alpha-blend this 1024×600 ARGB8888 layer over every dirty
  region of every frame, on every screen, while FX is on. The scrolling refresh band also invalidates
  a full-width stripe ~18×/s, forcing a full-layer recomposite even on static screens. This is often
  the single largest per-frame overhead when FX is enabled. A/B: `{"cmd":"fx","on":false}` vs `on:true`
  with the HUD up. If it must stay on, bake the static scanlines/vignette into content and keep only
  the moving band as a small dirty region. Do NOT make it a *translucent* `layer_top` object — that's
  the watchdog dead-end in the cost-model.

- Full-screen or large-area `lv_obj_invalidate()` / `lv_obj_invalidate_area()`, or recreating
  big objects each frame (forces their whole bbox to redraw).

- Animations/waveforms that redraw the whole track instead of just the changed segment. The scanner
  trace already uses a 10-segment shadow-compare (`prop_ui.c`, `WAVE_SEGS`) — that's the pattern to
  prefer; flag anything that doesn't.

### 2. Where pixels and draw-masks live (PSRAM vs internal SRAM)
Rendering *into* PSRAM and allocating draw-masks *from* PSRAM is much slower than internal SRAM, and
PSRAM contention also adds frame-time variance.
- **Draw buffers**: `bsp_illuminate.c` sets `buff_spiram=true`, full-screen double buffer. Full-screen
  ×2 can't fit internal RAM, but a *partial* (N-line) buffer in internal RAM can — evaluate whether
  partial-refresh buffers in internal SRAM beat full-screen PSRAM buffers for the redraw sizes this UI
  actually produces. This is a real, untried lever; flag it as an experiment with a measurement plan,
  not a guaranteed win.
- **LVGL heap**: routed entirely to PSRAM (`lv_port_mem.c`, custom allocator). LVGL 9 allocates
  anti-alias masks via `lv_malloc` *during* drawing, so every line/shape draw pays PSRAM allocation
  latency — this is the documented reason spectrum is ~8 fps vs ~18 static. The fix:
  a **hybrid allocator** (small/short-lived allocs → internal SRAM, large buffers → PSRAM). This is
  usually the single highest-value throughput change. Details and constraints are in `references/cost-model.md`.

### 3. CPU contention & frame-time consistency (priority-1 lever)
This is the most likely cause of the *swinging* frame times the user cares most about.
- **Core affinity**: check every `xTaskCreate` / `lvgl_port_init` task config. Right now LVGL and the
  Wi-Fi/BLE/CSI/engine tasks all run with **no core pinning** (`task_affinity=-1` in
  `bsp_illuminate.c`; `NULL`/unpinned in `prop_net.c`, `prop_ble.c`, `prop_csi.c`, `prop_engine.c`).
  When a radio task lands on LVGL's core mid-render, the frame stalls. Pinning LVGL to one HP core and
  pushing the radio/background tasks to the other is the classic fix for inconsistency — propose it
  with the specific tasks and core assignments.
- **Task priority**: the LVGL port task runs at `configMAX_PRIORITIES-4`. Confirm no chatty
  background task sits at or above it on the same core.
- **Work under the LVGL lock**: input (dial/touch) is processed by `prop_ui_input()` which
  acquires `lvgl_port_lock()` before doing anything. This means a render stall *directly* causes
  input lag — the user's "laggy dial" and "slow frame times" are the same problem. Anything heavy
  under the lock blocks both render and response. Grep for `lvgl_port_lock` call sites. WiFi/SDIO
  calls under the lock are a known ~30 s flicker bug — flag any `esp_wifi_*` / SDIO access reachable
  from a UI callback or observer.
- **Single SW draw unit**: `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`. With two HP cores, a second draw
  unit can parallelize software rendering — evaluate `=2` (it competes with the radio tasks for the
  second core, so it pairs with the affinity decision; measure, don't assume).

### 4. Per-frame widget churn (throughput + occasional hitch)
The observer runs ~20 Hz. Per-frame work multiplied by widget count adds up fast, and full rebuilds
cause periodic hitches. In `prop_ui.c`'s `ui_observer`:
- **SPECTRUM / RF BAND / CSI**: update every bar every frame — e.g. spectrum is 32 bars ×
  (`set_height` + `align` + `set_bg_color`) ≈ 96 LVGL calls/frame. Look for: redundant calls when the
  value didn't change (shadow-compare like the scanner trace), restyling that could be set once, and
  alignment recomputation that could be cached.
- **BLE CONTACTS**: `lv_obj_clean(s_ble_list)` + full row rebuild every ~400 ms — a whole widget
  hierarchy teardown/recreate. That's a periodic hitch; prefer diffing rows and updating in place,
  or reusing a fixed pool of row widgets.
- **Update cadence vs. need**: is anything updating at 20 Hz that the eye/camera can't see faster
  than, say, 8–10 Hz? Throttling a screen's data refresh (as HOME/VITALS already do with `tick % N`)
  cuts redraw work directly.

### 5. Expensive draw features
Software-rendered, these are pure CPU. Audit usage and whether each is worth its cost on the heavy
screens: large opacity/alpha-blended fills (the ARGB8888 CRT overlay path), gradients, rounded
corners / radius, big anti-aliased fonts redrawn often, and image rescaling. Setting `radius=0`
(square corners — already the house style) and avoiding per-frame alpha fills are cheap wins.

### 6. Already-maxed knobs — verify, don't churn
CPU is at 360 MHz, PSRAM at 200 MHz hex/octal, compiler at `-O2`, FreeRTOS at 1000 Hz. These are
already tuned (see cost-model). Confirm they haven't regressed, but don't present "raise the CPU
clock" as a finding — there's little headroom left there.

## Output: the report

Produce a single prioritized report. Lead with the consistency fixes, then throughput. Structure:

```
# Communicator framerate review

## Measured baseline
(per-screen FPS + consistency notes, or "static review — not measured on hardware")

## Findings (ranked by frame-time impact)
For each:
- **[P1/P2/P3] Short title** — affects: {consistency | throughput | both}
  - Where: file:line (symbol)
  - Cost: why this burns frame time / causes variance, with the mechanism (not just "it's slow")
  - Fix: the concrete change, and an estimate of the win (and confidence)
  - Risk: anything it could break (cross-check against cost-model constraints)

## Suggested order of attack
(cheapest-high-impact first; group changes that should be measured together)

## Measurement plan
(exact prop.py/curl commands to A/B each change with the FPS HUD)

Key commands to include:
```bash
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'   # FPS HUD on
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'   # CRT overlay off (A/B)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'    # CRT overlay on
```

```

Rank by frame-time impact and the user's priority (consistency first). Be honest about confidence:
distinguish "code-evident win" from "plausible, needs measurement." A few sharp, verified findings
beat a long list of maybes — the user has to act on this.

## Hard constraints (cross-check every fix against these)

These are settled. A "fix" that violates one is a regression, not an improvement:
- **Don't restore the LVGL builtin pool or raise `LV_MEM`** — starves esp_hosted's SDIO DMA mempool
  → "HS_MP / mempool: no mem" boot loop.
- **Don't touch `CONFIG_ESP32P4_*REV*`** — board is rev v1.3; wrong rev → won't boot.
- **Keep `swap_bytes=false`** in the display cfg — the panel takes native RGB565; true renders the
  background dark magenta.
- **No PPA/GPU rotation path** — breaks on rev v1.3 (that's why software rendering in the first place).
- **Never call WiFi/SDIO under the LVGL lock** — cache from a background task; the UI reads the cache.
- **Don't run the draw pipeline (canvas layer-draw / `lv_snapshot`) from a non-LVGL task under the
  lock** — it deadlocks (that's why `/screenshot` reads the FB directly and `prop_fx` paints pixels).

## Companion skills
- **communicator-ui** — the build/flash/screenshot loop and house style. Use it to apply and verify
  any change you recommend here (build with `tools/dev.sh bf` / `dev.ps1 bf`, then screenshot).
- **design-kit** — the prop_kit components, if a fix involves restructuring a panel's layout.
