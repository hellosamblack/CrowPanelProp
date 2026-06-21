# Communicator framerate review

## Measured baseline

**Static review — NOT measured on hardware.** The device was unreachable (`tools/prop.py` couldn't run — no `python` on PATH, no board attached), so I could not enable the FPS HUD or capture per-screen FPS / frame-time swings. Findings are tagged code-evident vs. needs-on-device-confirmation; a measurement plan is at the end.

Known baseline (from cost model): full 1024×600 software render ≈ 250 ms (~4 fps); spectrum ~8 fps; static screens ~18 fps; the 8↔18 gap is dominated by per-frame PSRAM allocation during drawing. The user's ~10–20 fps + erratic frame times fits that baseline plus a contention/overlay cost. Priority applied: **(1) consistency, (2) fps.**

## Findings (ranked by frame-time impact)

### [P1] No core affinity — LVGL and all radio/background tasks float across both HP cores
- **Affects:** consistency (prime suspect for the swinging frame times)
- **Where:** `bsp_illuminate.c:159` (`task_affinity = -1`); `prop_net.c:240,229`; `prop_ble.c:331` + NimBLE host; `prop_csi.c:151`; `prop_mic.c:186`; `prop_engine.c:302`; on-demand scans `prop_ui.c:431,950,1085` (blocking 1–5 s)
- **Cost:** every task is unpinned. A ~250 ms render holds a core for the whole frame; when the scheduler migrates a radio task (NimBLE adverts, CSI fold, mic FFT, RSSI SDIO poll, blocking scan) onto LVGL's core mid-render, that frame stalls → 18→6→18 swing. Cost model names this "the main consistency liability."
- **Fix:** set `lvgl_cfg.task_affinity = 0` and convert the radio/sensor `xTaskCreate` calls to `xTaskCreatePinnedToCore(..., 1)`. Decide where `prop_anim`/observer rides by A/B. **Biggest consistency win**; confidence high it helps, fps delta needs measurement.
- **Risk:** don't pile the 2nd draw unit (P3) + radio tasks on the same core. No hard-constraint knobs touched.

### [P1] Full-screen ARGB8888 CRT overlay on `lv_layer_top()` — alpha-composited every frame (when FX on)
- **Affects:** both — large fixed per-frame cost on every screen + periodic stripe-composite from the scroll band
- **Where:** `prop_fx.c:178-193` (`s_canvas` = 1024×600 ARGB8888 on `lv_layer_top()`), `:64-67` (scanline/phosphor/vignette all *partial* alpha), `:159-170`/`:51` (band scrolls every `FX_BAND_MS = 55 ms`)
- **Cost:** object is `LV_OPA_COVER` but its **pixels carry per-pixel alpha < 255**, so LVGL must software alpha-blend this full top-layer canvas (ARGB8888 over RGB565 — the most expensive blend path) over every dirty region of every frame, on every screen, while FX is on. The band also invalidates a full-width ~82 px stripe ~18×/s, forcing overlay recomposite even on static screens. The cost-model "paints directly to dodge recomposite" note addresses the *canvas-update deadlock*, NOT this per-frame *composite* of an alpha layer-top object.
- **Fix:** (1) Confirm FX state first — defaults OFF (`prop_fx.c:258`) but is persisted; A/B `{"cmd":"fx","on":false}`. (2) If it must stay on, bake the static scanline/vignette/wash into content and keep only the moving band as a small object. (3) Interim: raise `FX_BAND_MS` / shrink band width. Turning FX off is a large immediate win (code-evident); restructure quantified by measurement.
- **Risk:** keep visually verified via `/screenshot`. Do NOT make it a *translucent* layer_top object (cost-model dead-end: watchdog hang).

### [P1] LVGL heap entirely in PSRAM — per-draw AA-mask allocation pays PSRAM latency (the named lever)
- **Affects:** both — documented reason spectrum is ~8 fps vs ~18; allocator spikes add variance
- **Where:** `lv_port_mem.c:39-43` — all `lv_malloc`/`lv_realloc` → `MALLOC_CAP_SPIRAM` regardless of size
- **Cost:** LVGL 9 allocates line/shape AA masks via `lv_malloc` *during drawing*; with the whole heap in PSRAM every draw pays PSRAM alloc+access latency, and variable timing adds jitter.
- **Fix:** implement the **hybrid allocator** (CLAUDE.md/cost-model named lever): route small/short-lived allocs (e.g. <1–4 KB) to `MALLOC_CAP_INTERNAL` with **PSRAM fallback**; large buffers stay PSRAM. Keep generous internal headroom (~332 KB free budget). "Usually the single highest-value throughput change."
- **Risk (hard constraint):** never internal-only — must not starve esp_hosted's SDIO mempool ("HS_MP: no mem" boot loop). Don't restore the builtin pool / raise `LV_MEM`. If WiFi RAM tightens, use `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`.

### [P2] Spectrum/RF/CSI bars restyle + realign every bar every frame (no shadow-compare)
- **Affects:** throughput (+ small hitch)
- **Where:** `prop_ui.c` ui_observer — SPECTRUM 24 bars `:2693-2704`, RF 13 bars `:2715-2726`, CSI 32 bars `:2758-2769`
- **Cost:** each bar calls `set_height` + `lv_obj_align` + `set_style_bg_color` *unconditionally* ~20 Hz (~96 ops/frame for CSI), invalidating each bbox even when nothing changed. The X/Y in `lv_obj_align` are constants re-set every frame; bg_color only needs to change on a threshold crossing. The scanner waveform already does this right (`prop_ui.c:2542-2557`, `WAVE_SEGS`); the bar screens don't.
- **Fix:** mirror the scanner shadow-compare. Skip `set_height` when height unchanged; **drop the per-frame `lv_obj_align` entirely** (set bar position once at build time); call `set_bg_color` only when the colour bucket changes. Cuts heavy-bar-screen op count ~2/3 and invalidated area to changed bars only — directly attacks the ~8 fps spectrum number. Pairs with the hybrid allocator (fewer draws ⇒ fewer mask allocs).
- **Risk:** track colour shadow independently of height shadow. Low.

### [P2] BLE CONTACTS — full `lv_obj_clean` + row rebuild every refresh
- **Affects:** consistency (periodic hitch on CONTACTS)
- **Where:** `prop_ui.c:2747-2750` — `lv_obj_clean(s_ble_list)` + `ble_add_row()` for all devices in the `tick % 8` (~2.5 Hz) block
- **Cost:** tears down/recreates the whole row hierarchy every ~400 ms → PSRAM free+alloc churn + full-list redraw spike (recurring stutter). Throttle is fine; the teardown-rebuild shape is wrong.
- **Fix:** reuse a fixed pool of pre-built rows sized to `PROP_BLE_MAX`; update label text/colour in place, hide unused rows. Only changed rows invalidate. Low risk (reset stale rows on shrink).

### [P3] Evaluate a second SW draw unit
- **Where:** `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`. Try `=2` to use both HP cores for rendering — **only after the affinity decision (P1)**, since the 2nd unit competes with radio tasks for the second core. Measurement-dependent, coupled to P1; medium confidence, propose as experiment. Risk: can *worsen* consistency if affinity doesn't separate them — measure the swing, not just average.

### [P3] Partial draw buffers in internal SRAM (experiment)
- **Where:** `bsp_illuminate.c:173-188` — full-screen double buffer, `buff_spiram = true`. Rendering into PSRAM is much slower than internal SRAM; a partial N-line buffer fits internal RAM and may beat full-screen PSRAM for this UI's small dirty regions. A/B: drop `buffer_size` to an N-line strip + `buff_spiram = false`. Keep `swap_bytes=false`, no full_refresh/direct_mode/PPA. Watch internal-RAM budget (esp_hosted). Experimental.

### [P3] (Note) FPS-HUD comment overstates behavior; refresh-period is a known dead end
- `prop_ui.c:2908-2925` claims the HUD drops the refresh period 30→8 ms, but there's **no `set_refr_period` call anywhere** (grep clean); the LVGL timer period is fixed at 5 ms (`bsp_illuminate.c:161`). Comment is stale. Per cost model, chasing the refresh period was already tried and lifted nothing — don't pursue it; optionally fix the comment.

### Already-correct / do-not-churn (verified)
Scanner waveform 10-segment shadow-compare + change-gated style writes (`prop_ui.c:2498-2615`); bare SCANNER skipped when an opaque panel is up (`PK_NONE` guard); HOME/VITALS/ABOUT `tick % N` throttling; change-gated LED I2C + non-publishing sensitivity (`prop_engine.c:235-239,426-429`); RSSI/uplink read from background cache, not under the lock. CPU 360 MHz / PSRAM 200 MHz HEX / `-O2` / FreeRTOS 1000 Hz are at the ceiling — verify only, don't propose raising.

## Suggested order of attack
1. **Free baseline (minutes):** FPS + ~10 s consistency on spectrum/csi/scanner/ble, **FX off vs on** — quantifies the overlay and reveals whether the unit even has FX on.
2. **Core affinity (P1, consistency)** — re-measure the *swing* (user's #1 priority).
3. **Hybrid allocator (P1, throughput)** — re-measure spectrum/csi; watch internal RAM via VITALS.
4. **Bar shadow-compare + drop per-frame align (P2)** — apply to all three bar screens together (synergistic with #3).
5. **BLE row pooling (P2)**.
6. **CRT overlay restructure (P1)** only if FX must stay on.
7. **Experiments together (P3):** 2nd draw unit × affinity, partial internal buffers — keep/revert on data.

## Measurement plan
```bash
# 0. Reachability + HUD/FX toggle (mDNS comm-unit-7.local)
python firmware/communicator/tools/prop.py state
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
# 1. Per-screen baseline
python firmware/communicator/tools/prop.py shot spectrum.png --screen spectrum --wait
python firmware/communicator/tools/prop.py shot csi.png      --screen csi      --wait
python firmware/communicator/tools/prop.py shot scanner.png  --screen scanner  --wait
python firmware/communicator/tools/prop.py shot ble.png      --screen ble      --wait
# 2. P1 overlay A/B (read HUD each time)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'   # then re-read HUD
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'
# 3. After each code change: flash + re-capture + re-read HUD (record steady fps AND the 10 s swing)
./firmware/communicator/tools/dev.sh bf -Port /dev/ttyUSB0     # or dev.ps1 bf -Port COM7
# 4. Input/dial latency (handled under the LVGL lock)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"input","control":"selector","arg":"cw"}'
# 5. RAM headroom guard after hybrid allocator
python firmware/communicator/tools/prop.py shot vitals.png --screen vitals --wait
```

Highest-confidence levers: **affinity (P1 consistency)** and the **hybrid allocator (P1 throughput)**; the **FX overlay** is the highest-confidence *immediate* win if enabled; the **bar shadow-compare (P2)** is a code-evident cleanup. Experiments (2nd draw unit, partial internal buffers) keep/revert strictly on measured numbers.
