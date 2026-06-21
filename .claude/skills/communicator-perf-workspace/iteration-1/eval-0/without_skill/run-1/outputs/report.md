# Communicator framerate review

## Measured baseline

**Static review only — not measured on hardware.** No device was reachable (no
`python`/mDNS in this environment), so every number below is the *code-evident* mechanism
plus figures already recorded in the firmware (`prop_ui.c` FPS-meter comment, `CLAUDE.md`
"Memory reality"): full-screen 1024x600 software render ~= 250 ms (~4 fps); spectrum
~8 fps; mostly-static screens ~18 fps. Findings are tagged **[code-evident]** (cost is
structural and visible in source) vs **[needs on-device confirmation]** (plausible, must
be A/B'd with the FPS HUD).

Everything renders in software (no GPU/PPA on rev v1.3), so frame time is governed by
four things: invalidated area, where pixels/draw-masks live (PSRAM vs internal SRAM), CPU
contention from radio tasks, and per-frame widget work. All four are in play here.

---

## Findings (ranked by frame-time impact — consistency first, then throughput)

### [P1] CRT overlay is a full-screen ARGB8888 layer over every redraw — affects: BOTH
- **Where:** `main/prop_fx.c:178-193` (`fx_build`): `s_canvas` is a 1024x600
  `LV_COLOR_FORMAT_ARGB8888` canvas on `lv_layer_top()` with `opa = LV_OPA_COVER`. The
  scrolling band (`prop_fx.c:159-170` `band_tick`, `FX_BAND_MS=55`) moves an 80 px
  full-width ARGB8888 canvas ~18x/s.
- **Cost / mechanism:** Although the canvas pixels are static (baked once), it sits on the
  top layer above the whole panel. In LVGL's software renderer **every dirty region the UI
  produces underneath must be alpha-composited against the ARGB8888 overlay pixel-by-pixel**
  before flush — 4 bytes/px with a per-pixel src-over blend on top of the RGB565 base
  draw. So this multiplies the cost of every other invalidation, which is why
  spectrum/scanner feel heavy with FX on. Worse for consistency: the band timer forces an
  ~80px x 1024 full-width recomposite every 55 ms regardless of UI activity — a periodic,
  FX-on-only hitch at a steady ~18 Hz. The cost-model flags translucent top-layer objects
  as the recomposite hazard; this is one (alpha lives in the buffer even though layer
  opacity is COVER).
- **Fix:** (1) cheapest big consistency win: gate the overlay OFF during the heavy moving
  instruments (spectrum/scanner/csi/rfband), or off by default. (2) drop or slow the
  refresh band (wider step / longer period) if FX stays on during motion. (3) an opaque
  RGB565 cover can be blitted not blended, but an opaque cover would hide the UI, so the
  scanlines force alpha — (1) is the real lever.
- **Estimated win:** Large on FX-on screens (blend vs no-blend on every dirty pixel).
  **[code-evident]** that it adds per-pixel ARGB cost; exact fps delta **[needs on-device
  confirmation]**. Likely the single largest *consistency* win — the band stripe is the
  clearest periodic hitch in the codebase.
- **Risk:** Changes the on-camera look (deliberate aesthetic). Make it a per-screen policy
  (FX on for static/hero screens, off while an analyser moves), not a global kill. Do NOT
  move it off `lv_layer_top` onto a translucent screen child — reintroduces the
  watchdog-hang hazard in the cost-model.

### [P1] No core pinning — radio/engine tasks migrate onto LVGL's core mid-render — affects: CONSISTENCY
- **Where:** every task uses plain `xTaskCreate` (affinity -1; no `xTaskCreatePinnedToCore`
  exists anywhere, verified by grep):
  - LVGL port task: `peripheral/bsp_illuminate/bsp_illuminate.c:159` `.task_affinity = -1`
  - engine/observer: `main/prop_engine.c:302` (`prop_anim`, prio 5)
  - rssi poll + ap fallback: `main/prop_net.c:240,229` (prio 3)
  - ble prune + NimBLE host: `main/prop_ble.c:331` (prio 3)
  - csi fold: `main/prop_csi.c:151` (prio 4)
  - mic FFT: `main/prop_mic.c:186` (prio 5)
- **Cost / mechanism:** With no affinity the scheduler can land a Wi-Fi/BLE/CSI/FFT task on
  the same HP core as LVGL mid-frame. A software render is a long CPU burst; when a radio
  task preempts it, that frame stretches — the textbook cause of the 18->6->18 swing the
  user cares most about. The mic FFT (prio 5 = engine, radix-2 256-pt ~62x/s) and CSI fold
  (prio 4, ~15 Hz) are the most likely render-stealers on the spectrum/CSI screens.
- **Fix:** Pin LVGL to one HP core (`.task_affinity = 0` at `bsp_illuminate.c:159`) and push
  the producers to the other (`xTaskCreatePinnedToCore(...,1)` for `prop_anim`, `rssi`,
  `ble_prune`, `prop_csi`, `prop_mic`; NimBLE host via its Kconfig core option). The
  observer takes the LVGL lock, so test the engine on LVGL's core vs the opposite core and
  keep whichever gives flatter frame times.
- **Estimated win:** Mainly variance reduction (priority-1 goal); modest avg fps change.
  **[needs on-device confirmation]** — but "everything unpinned" is the clearest structural
  consistency liability in the build.
- **Risk:** Low. Don't starve the core esp_hosted/Wi-Fi prefers; verify no boot regression.
  Pairs with the second-draw-unit finding (competes for the core you free).

### [P2] Hybrid allocator not implemented — every line/shape draw pays PSRAM alloc latency — affects: THROUGHPUT
- **Where:** `main/lv_port_mem.c:39-43` — `lv_malloc_core`/`lv_realloc_core` route ALL LVGL
  allocations to PSRAM (`MALLOC_CAP_SPIRAM`) unconditionally.
- **Cost / mechanism:** LVGL 9 allocates anti-alias mask buffers via `lv_malloc` *during*
  drawing (lines, the scanner trace, rounded/AA shapes, large AA fonts). With the heap
  entirely in PSRAM, every such draw pays PSRAM alloc + access latency — the documented
  reason spectrum is ~8 fps vs ~18 static (named as *the* tuning lever in cost-model and
  firmware CLAUDE.md).
- **Fix:** Implement the hybrid allocator in `lv_port_mem.c`: route small/short-lived allocs
  (the draw masks, e.g. <= a few KB) to `MALLOC_CAP_INTERNAL` with a PSRAM fallback when
  internal is low; keep large buffers (objects, assets) in PSRAM. Leave generous internal
  headroom for esp_hosted (~332 KB free at runtime with all radios live — stay well under).
- **Estimated win:** Potentially the largest *throughput* win on line/AA-heavy screens.
  **[code-evident]** that the cost exists; magnitude **[needs on-device confirmation]**.
- **Risk:** Must not regress into the esp_hosted "HS_MP / mempool: no mem" boot loop — cap
  internal use, always fall back to PSRAM on exhaustion. Do NOT restore the builtin pool or
  raise `LV_MEM`. Re-test after a `fullclean` (RAM layout shifts).

### [P2] Analyser bars rewrite position + colour every frame unconditionally — affects: THROUGHPUT
- **Where:** `main/prop_ui.c` observer:
  - SPECTRUM `2690-2704`: 24 bars x (`set_height` + `align` + `set_style_bg_color`) = ~72
    LVGL calls/frame, every frame.
  - CSI `2755-2769`: 32 bars x 3 = ~96 calls/frame.
  - RF BAND `2714-2727`: 13 bars x 3 = ~39 calls/frame.
- **Cost / mechanism:** Three problems. (1) `lv_obj_align()` runs every frame though each
  bar's X never changes — only height does; re-aligning re-dirties the bbox needlessly.
  (2) `set_style_bg_color()` is rewritten every frame though colour only changes at a
  threshold (35/85), forcing a style invalidate + full-bar redraw each tick. (3)
  `set_height` dirties old∪new bbox (unavoidable when value changes) but is multiplied by
  24-32 bars all changing together — and with FX on (P1) each dirty bbox is then
  ARGB-composited. The scanner trace already solved this with a shadow-compare
  (`prop_ui.c:53-57, 2542-2557`, `s_wave_shadow`); the analysers don't use the pattern.
- **Fix:** Apply the scanner's shadow-compare to the bar loops: cache last height + last
  colour-band per bar; skip `set_height` when unchanged; drop the per-frame `lv_obj_align`
  entirely (X is already set once in the build functions); only call `set_style_bg_color`
  when the threshold band changes. RF BAND holds steady between scans so most frames become
  no-ops.
- **Estimated win:** Removes ~1/3 of calls (redundant align) + most colour invalidations;
  RF BAND nearly free between scans. **[code-evident]**.
- **Risk:** Low — pure update-elision, no visual change. Mirror the existing scanner shadow
  pattern for maintainability.

### [P2] Spectrum/CSI redraw at 20 Hz; can throttle to ~10 Hz — affects: THROUGHPUT
- **Where:** observer SPECTRUM `2690` and CSI `2755` run every tick (~20 Hz); only the dB
  readouts are throttled (`tick % 4`, `tick % 8`). HOME/VITALS/ABOUT already throttle the
  whole block (`tick % 10`, `tick % 20`).
- **Cost / mechanism:** Halving the bar-update rate halves the redraw work those screens
  generate. At 10 Hz the decay ballistics still read fine on camera. Same lever HOME/VITALS
  already pull.
- **Fix:** Gate the SPECTRUM and CSI bar loops on `st->tick % 2 == 0` (10 Hz). Keep the
  scanner at full rate (hero motion). Confirm decay constants still look smooth.
- **Estimated win:** ~halves analyser-screen redraw load. **[needs on-device confirmation]**
  that 10 Hz still looks good (likely, given the decay).
- **Risk:** Slightly chunkier motion; revert per-screen. Do NOT drop the global refresh
  period — documented dead end.

### [P3] Second SW draw unit unused — second HP core idle for rendering — affects: THROUGHPUT
- **Where:** `sdkconfig:4925` `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`.
- **Cost / mechanism:** One draw unit = single-core software rendering; the second HP core
  does no rendering. `=2` lets LVGL split a redraw across both cores.
- **Fix:** `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2`. **Couple with the affinity decision (P1):**
  the second draw thread competes with radio tasks for the second core, so it only pays off
  if those are pushed off it. Measure together.
- **Estimated win:** up to ~2x on large redraws if the second core is free, less in
  practice. **[needs on-device confirmation]** — can *hurt* consistency if it fights the
  radio tasks, hence bundled with P1.
- **Risk:** More internal RAM (second draw context + stack) — watch esp_hosted headroom.
  Revert if variance worsens.

### [P3] Partial draw buffers in internal SRAM (experiment) — affects: THROUGHPUT
- **Where:** `bsp_illuminate.c:173,188` — full-screen buffer, `double_buffer=true`,
  `buff_spiram=true`.
- **Cost / mechanism:** Rendering into PSRAM is slower than into internal SRAM and adds
  variance under contention. Full-screen x2 (~2.4 MB) can't fit internal RAM, but a partial
  N-line buffer can, and this UI is partial-refresh with mostly small dirty regions — a fast
  internal partial buffer can beat a slow full PSRAM one for those.
- **Fix:** A/B a partial buffer (e.g. ~1/8 screen, x2) in internal SRAM vs the current full
  PSRAM double-buffer. Experiment, not a guaranteed win (cost-model lists it untried).
- **Estimated win:** unknown; plausibly meaningful for small-region updates. **[needs
  on-device confirmation]**.
- **Risk:** internal-RAM budget vs esp_hosted; revert if no improvement. Keep
  `swap_bytes=false` and rev/PPA/`num_fbs` untouched (hard constraints).

### [P3 / housekeeping] Stale FPS-HUD comment — not a perf bug, but misleads tuning
- **Where:** `main/prop_ui.c:2908-2925` claims the HUD "drops the refresh-timer period from
  30 ms to 8 ms." No code path actually changes the refresh period when the HUD turns on
  (`prop_ui_set_fps` only shows/hides the label; grep finds no `lv_timer_set_period` /
  refresh-period write). The HUD is a pure passive counter. Real period is
  `CONFIG_LV_DEF_REFR_PERIOD=33` (sdkconfig:4861), lvgl_port timer 5 ms.
- **Why it matters:** anyone tuning will trust the comment and mis-attribute fps changes.
  Fix the comment. Do NOT "implement" the 8 ms drop — dropping the period is the documented
  dead end (no gain, network-sluggish).
- **Risk:** none (doc fix).

---

## Suggested order of attack

1. **Capture a real baseline.** Turn on the FPS HUD (DISPLAY panel "FPS METER" switch /
   `fps_on` setting) and record steady-state fps AND swing on scanner/spectrum/csi/rfband,
   FX-on and FX-off. You can't fix variance you can't see.
2. **P1 CRT overlay policy** (cheap, no RAM risk): FX-off on the moving analysers, slow/drop
   the refresh band. Re-measure. Clearest consistency win; isolates the ARGB recomposite cost.
3. **P2 analyser update-elision** (`prop_ui.c` bar loops): drop per-frame `align`,
   shadow-compare height + colour, throttle spectrum/CSI to 10 Hz. Pure software, no config
   risk, mirrors the scanner pattern.
4. **P1 core pinning** (variance lever): pin LVGL to one core, radios to the other. Measure
   swing on spectrum (FFT contention) and csi.
5. **P3 second draw unit** — flip `=2` in the SAME session as pinning (shared second core).
   Keep only if both fps and variance improve.
6. **P2 hybrid allocator** (`lv_port_mem.c`) — biggest throughput lever, riskiest
   (boot-loop). Do last, after `fullclean`, re-verify boot + ~332 KB internal headroom.
7. **P3 partial internal draw buffers** — final A/B experiment if more throughput is wanted.

Group for measurement: (4)+(5) together (shared core); (6) alone (boot risk); the rest
independent.

## Measurement plan

A/B with the on-device FPS HUD (counts real renders via `LV_EVENT_RENDER_READY`,
`prop_ui.c:2931`):

```bash
python tools/prop.py state    # confirm reachable (comm-unit-7.local)

# enable FPS HUD (setting key fps_on / DISPLAY panel switch), walk heavy screens ~10s each,
# noting the SWING not just the average:
python tools/prop.py shot scanner.png  --screen scanner  --wait
python tools/prop.py shot spectrum.png --screen spectrum --wait
python tools/prop.py shot csi.png      --screen csi      --wait
python tools/prop.py shot rfband.png   --screen rfband   --wait

# isolate the P1 recomposite cost: toggle the CRT overlay and re-walk the same screens
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'
# ...repeat shots / HUD readings, FX off...
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
```

For each fix record per-screen (min, typical, max) fps over ~10 s, FX on and off. Priority
is the SPREAD (max-min) shrinking, then typical rising. `/screenshot` reads the framebuffer
(incl. FX overlay), so a quick visual diff confirms ballistics still look right after
throttling/elision changes.

## Do NOT touch (cross-checked vs cost-model)

- Don't restore the LVGL builtin pool / raise `LV_MEM` (esp_hosted mempool boot loop).
- Don't change `CONFIG_ESP32P4_*REV*`, `swap_bytes` (keep false), or add a PPA/GPU path.
- Don't call WiFi/SDIO under the LVGL lock; don't run canvas layer-draw/`lv_snapshot` from a
  non-LVGL task under the lock.
- Don't drop the global refresh period or force full-screen redraws — documented dead ends.
- CPU 360 MHz, PSRAM 200 MHz HEX, `-O2`, FreeRTOS 1000 Hz are at the ceiling (confirmed in
  sdkconfig) — not worth churning.
