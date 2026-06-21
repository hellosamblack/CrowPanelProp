# Communicator framerate review

## Measured baseline

**Static review — not measured on hardware.** The device was unreachable from this environment
(`python` is not on PATH and `comm-unit-7.local` did not answer), so I could not turn on the FPS HUD
or A/B anything live. Every finding below is grounded in `file:line` and the known cost model; each
is flagged **code-evident** vs **plausible, needs measurement**.

Known numbers (in-code comments + CLAUDE.md), used as ground truth:
- Whole-screen 1024x600 software render ~250 ms (~4 fps); no GPU/PPA on rev v1.3.
- Heavy spectrum screen ~8 fps, mostly-static screens ~18 fps.
- LVGL heap is entirely in PSRAM (custom allocator), so per-draw AA-mask `lv_malloc` pays PSRAM latency.

The user's two symptoms map onto two distinct causes:
1. "Hitches roughly once a second" -> a periodic 1 Hz background task (the RSSI poll) doing a
   blocking SDIO round-trip on an unpinned core. Consistency problem (priority 1).
2. "Dial/touch feels laggy" -> input is processed under the LVGL lock, so it queues behind the
   render/observer; on heavy bar screens a frame can take 50-125 ms and the lock is held that long.

---

## Findings (ranked by frame-time impact)

### [P1] Once-per-second SDIO RSSI poll on an unpinned core - affects: consistency
- Where: `main/prop_net.c:510` `rssi_task`; `:515` `esp_wifi_sta_get_ap_info`; `:535`
  `vTaskDelay(pdMS_TO_TICKS(1000))`. Created unpinned at `:240`
  (`xTaskCreate(rssi_task,"rssi",4096,NULL,3,NULL)`).
- Cost/mechanism: Exactly once a second this task makes a blocking SDIO transaction to the C6
  (`esp_wifi_sta_get_ap_info` + `esp_wifi_sta_get_negotiated_phymode`). The 1000 ms cadence is a
  dead-on match for "hitches roughly once a second." Compounding: (a) the task is unpinned
  (`task_affinity=-1` for LVGL too, `bsp_illuminate.c:159`), so the scheduler can run it on the same
  HP core mid-software-render, stalling that frame; (b) the SDIO/C6 link is shared with NimBLE, so
  the round-trip can contend and stretch. A ~10-40 ms stall landing on a render frame is precisely
  the visible 1 Hz hitch.
- Fix: Pin LVGL to one HP core, radios to the other. Set `task_affinity=0` in the `lvgl_port_cfg_t`
  (`bsp_illuminate.c:157-161`); create `rssi_task`, BLE prune (`prop_ble.c:331`), CSI
  (`prop_csi.c:151`), mic (`prop_mic.c:186`) with `xTaskCreatePinnedToCore(...,1)`.
- Win/confidence: Removes the dominant source of the once-a-second hitch. High confidence on the
  cause (cadence match + unpinned + SDIO). The cost model names unpinned radio<->LVGL tasks as "the
  main consistency liability." Highest-value consistency change.
- Risk: Pinning all heavy work to core 1 could saturate it if you also enable a second draw unit
  (P3) - decide together. Do NOT reduce poll cadence to "fix" this: the hitch is the stall on
  render's core, not the poll frequency (cost model lists cadence churn as a dead end). SDIO must
  stay off the LVGL lock (it already is).

### [P1] Full-panel ARGB8888 CRT overlay on lv_layer_top() recomposited every frame - affects: both
- Where: `main/prop_fx.c:178-207`. `s_canvas` is a 1024x600 ARGB8888 canvas on `lv_layer_top()`
  (`:187-189`); `s_band` is an 80 px ARGB8888 band on the same top layer (`:199-207`) moving every
  55 ms (`band_tick` `:159-170`, `FX_BAND_MS=55`).
- Cost/mechanism: The canvas wash/scanlines are ARGB8888 with per-pixel alpha < 255
  (`FX_WASH_OPA=20`, scanline `FX_SCAN_OPA=95`). Translucent content on `lv_layer_top` forces LVGL to
  alpha-blend the overlay over screen content for every invalidated region - and ARGB8888 src-over
  blend is pure CPU here (no PPA). So every widget redraw (each spectrum bar height change, each wave
  segment, the band stripe) is paid twice: draw the widget, then composite the overlay stripe. The
  cost model flags translucent `lv_layer_top` objects as a known trap (it watchdog-hung the FPS HUD);
  the overlay is the deliberately-tolerated version of the same cost.
- Fix: (1) Quantify first - capture spectrum FPS `fx` on vs off. If off recovers a lot, this is the
  biggest throughput lever and the user picks the trade. (2) If keeping it: the band already
  invalidates only its ~82 px stripe (good); the static canvas alpha is the expensive part. Lower
  `FX_WASH_OPA`/scanline density to cut per-pixel blend cost. The cost is inherent to per-pixel
  alpha, so the realistic knob is "how strong / how much area."
- Win/confidence: Plausible-to-strong, needs measurement. Code-evident that it doubles blend work
  over every dirty region; magnitude is an on-device `fx` on/off A/B away.
- Risk: `prop_fx` paints ARGB pixels directly on purpose (v9 canvas layer-draw deadlocks under the
  lock) - don't move it off the LVGL task or re-add a translucent non-canvas top-layer object.

### [P2] Hybrid LVGL allocator (small allocs -> internal SRAM) - affects: throughput
- Where: `main/lv_port_mem.c:39` `lv_malloc_core` routes ALL `lv_malloc` to PSRAM
  (`LV_PSRAM_CAPS = MALLOC_CAP_SPIRAM`).
- Cost/mechanism: LVGL 9 allocates line/shape anti-alias mask buffers via `lv_malloc` during
  drawing. With the heap 100% PSRAM, every line/bar/shape draw pays PSRAM allocation latency, and
  PSRAM contention adds frame-time variance. The cost model names this as THE documented reason
  spectrum is ~8 fps vs ~18 static - the biggest throughput lever not yet done.
- Fix: Make `lv_malloc_core` hybrid: try `MALLOC_CAP_INTERNAL` first for small allocs (the transient
  AA masks), fall back to `MALLOC_CAP_SPIRAM` for large buffers and on internal-OOM.
  `lv_realloc_core`/`lv_free_core` already use `heap_caps_*` (free is cap-agnostic). Keep generous
  internal headroom - ~332 KB internal free at runtime with all radios live; cap internal-routed
  total well under that.
- Win/confidence: Code-evident high-value throughput win for bar/line screens (the firmware's own
  documented lever). Exact fps gain needs measurement.
- Risk: Constraint-sensitive. Do NOT restore the LVGL builtin pool or raise `LV_MEM` - starves
  esp_hosted's SDIO DMA mempool -> "HS_MP: mempool create failed: no mem" boot loop. The hybrid
  allocator must always have a PSRAM fallback (never return NULL) and leave esp_hosted internal RAM
  intact. Test a full `fullclean` boot with all radios on; watch internal-free in VITALS.

### [P2] SPECTRUM / RF BAND / CSI rewrite every bar every frame, no shadow-compare - affects: both
- Where: `main/prop_ui.c` `ui_observer`: SPECTRUM `:2690-2704` (32 bars), RF BAND `:2714-2727`, CSI
  `:2755-2769`. Each bar does, unconditionally every frame, `lv_obj_set_height` + `lv_obj_align` +
  `lv_obj_set_style_bg_color`.
- Cost/mechanism: ~3 LVGL calls x N bars per 50 ms tick (spectrum ~96 calls/frame). Each
  set_height/align invalidates old+new bbox; set_bg_color re-stamps style and invalidates again -
  even when value, position and color band didn't change. With peak-hold decay, many bars are flat in
  steady state, so much of this is wasted invalidation feeding the expensive blend + PSRAM-alloc
  path. The scanner trace already solves this with a 10-segment shadow-compare
  (`prop_ui.c:2538-2557`, `WAVE_SEGS`); these three screens don't.
- Fix: Add the scanner's shadow-compare. Cache last height + last color-band per bar; call `set_*`
  only on change. Bar x never changes - move `lv_obj_align` out of the per-frame loop (set once at
  build). Re-aligning a fixed-x bar every frame is pure waste.
- Win/confidence: Code-evident throughput + consistency win on the three sensor/spectrum screens (the
  ones most likely on-screen when it "hitches"). Magnitude scales with how static bars are; measure
  spectrum with live mic vs a quiet room.
- Risk: Low. Mirror the proven scanner pattern; keep the peak-hold decay math, just gate the calls.

### [P2] BLE contact list full teardown + rebuild every ~400 ms - affects: consistency (periodic hitch)
- Where: `main/prop_ui.c:2731-2751` (`PK_BLE`, `st->tick % 8 == 0` ~2.5 Hz): `lv_obj_clean(s_ble_list)`
  then `ble_add_row()` loop rebuilding the whole list.
- Cost/mechanism: `lv_obj_clean` deletes the entire child hierarchy (each row = 4 labels + a meter
  bar, `ble_add_row` `:1211-1246`) and recreates them - object alloc (PSRAM) + style init + layout
  for every widget, ~2.5x/s. A periodic spike that invalidates the whole list region and churns the
  PSRAM heap. Confined to the CONTACTS screen but a textbook periodic hitch.
- Fix: Diff rows / update in place, or reuse a fixed pool of `PROP_BLE_MAX` pre-built row widgets:
  set labels/meter for rows with data, hide the rest, never delete/recreate. Device set changes
  slowly, so most refreshes touch only text.
- Win/confidence: Code-evident consistency win on CONTACTS.
- Risk: Low; track row widget pointers and guard them in `close_panel` like the others.

### [P3] Single software draw unit - affects: throughput
- Where: `sdkconfig` `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`; two HP cores available.
- Cost/mechanism: All software rendering on one core; the second HP core is idle for draw. A second
  draw unit parallelizes the blend/raster across both cores.
- Fix: Try `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2`. Coupled to P1 affinity - a second draw unit competes
  with radios for the second core, so pin radios to one core and let the draw unit use the spare
  cycles; measure both together.
- Win/confidence: Plausible, needs measurement. Could raise throughput on heavy screens IF core 1
  isn't already saturated by radios - which is why it pairs with P1.
- Risk: If radios + second draw unit both pile on core 1, you trade hitches back in. Measure
  consistency, not just average fps.

### [P3] Partial draw buffers in internal SRAM - affects: throughput (experiment)
- Where: `bsp_illuminate.c:173-189` - full-screen (`H_size*V_size`), double, `buff_spiram=true`.
- Cost/mechanism: Rendering into PSRAM is far slower than into internal SRAM. Full-screen x2
  (~2.4 MB) can't fit internal RAM (hence PSRAM), but a partial N-line buffer in internal SRAM can.
  This UI mostly produces small dirty rects (partial-refresh mode), so partial internal buffers may
  render those faster.
- Fix: A/B a partial-refresh config: smaller per-buffer line count, `buff_spiram=false` (internal),
  keep double-buffer. Watch internal-RAM headroom (esp_hosted budget).
- Win/confidence: Plausible, untried - an experiment with a measurement plan, not a guaranteed win.
  Depends on actual dirty-rect sizes.
- Risk: Internal RAM is contended by esp_hosted; size conservatively. Keep `swap_bytes=false`.

### Verified-OK (do not churn)
- CPU 360 MHz (`ESP_DEFAULT_CPU_FREQ_MHZ_360`), PSRAM 200 MHz HEX (`SPIRAM_SPEED_200M`,
  `SPIRAM_MODE_HEX`), compiler -O2 (`COMPILER_OPTIMIZATION_PERF`), FreeRTOS 1000 Hz - all near
  ceiling; no headroom.
- Board rev pinned (`ESP32P4_REV_MIN_100`, `ESP32P4_SELECTS_REV_LESS_V3`) - don't touch.
- `swap_bytes=false` / `LV_COLOR_FORMAT_RGB565` correct - don't change.
- Scanner waveform already uses segment shadow-compare (`prop_ui.c:2538-2557`) - the model to copy.
- `ui_observer` already skips the scanner block unless `PK_NONE` is visible (`prop_ui.c:2498`);
  HOME/VITALS/ABOUT throttle with `tick % N` - good.
- FPS HUD is an opaque child, not a translucent top-layer object (`prop_ui.c:2917`) - correct.

---

## Suggested order of attack

Lead with consistency (user priority #1), cheapest-first:
1. P1 core affinity - pin LVGL to core 0, radios (rssi/ble/csi/mic) to core 1
   (`bsp_illuminate.c:159` + the `xTaskCreate` sites). Surgical; directly targets the 1 Hz hitch.
2. P2 per-bar shadow-compare on SPECTRUM/RF/CSI (`prop_ui.c:2690/2714/2755`) and move bar
   `lv_obj_align` out of the per-frame loop. Pure win, mirrors the scanner pattern.
3. P2 BLE list diff-in-place (`prop_ui.c:2731`) - kills the CONTACTS periodic hitch.
4. P2 hybrid allocator (`lv_port_mem.c:39`) - biggest throughput lever, constraint-sensitive; verify
   boot + internal-RAM headroom with all radios on.
5. Measure FX overlay cost (`fx` on vs off) to size the P1 overlay finding; decide the creative trade.
6. P3 second draw unit + partial internal buffers - experiments, measured together with the affinity
   choice (they fight for the same second core).

Group #1 + #6 in one measurement pass (they interact). Everything else is independent.

---

## Measurement plan

Hardware was not reachable here; run on the device (mDNS `comm-unit-7.local`). Enable the HUD, then
capture each heavy screen and watch the FPS readout over ~10 s for the SWING, not just the average.

```bash
python tools/prop.py state
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'

python tools/prop.py shot spectrum.png --screen spectrum --wait
python tools/prop.py shot rfband.png   --screen rfband   --wait
python tools/prop.py shot csi.png      --screen csi      --wait
python tools/prop.py shot ble.png      --screen ble      --wait

# A/B the CRT overlay cost
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'

# touch/dial latency on a heavy screen
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"ui","screen":"spectrum"}'
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"input","control":"selector","arg":"cw"}'
```

For each change record steady-state FPS AND the min/max swing per screen, before and after. The
user's bar is consistent frame times first, higher fps second - flattening an 18->6->18 swing to a
steady 12 is a win even if the average barely moves.
