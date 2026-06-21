# Communicator framerate review

User priority: **(1) consistent frame times first, (2) higher framerate.** The complaint —
"laggy dial/touch + a hitch roughly once a second" — points squarely at *consistency* (CPU
contention / periodic widget churn), which is where this review leads.

## Measured baseline

**Not measured on hardware — static code review only.** No physical device available, so I could
not run the FPS HUD or time touch latency. Findings are tagged **code-evident** (mechanism visible
in source) vs **needs measurement** (plausible lever, must be A/B'd on-device). Measurement plan at
the end. Known baseline from the cost model (ground truth, not re-derived): whole-screen 1024x600
software render ~250 ms (~4 fps); heavy spectrum ~8 fps; static screens ~18 fps. No GPU/PPA on rev
v1.3 — all compositing is the CPU.

The "hitch roughly once a second" is the most diagnostic clue: there is exactly one ~1 Hz background
actor doing a blocking SDIO round-trip to the C6 (the RSSI poll), and it is **unpinned**. That is P1.

## Findings (ranked by frame-time impact)

### [P1] No core affinity — radio/background tasks share LVGL's core — affects: consistency (the #1 complaint)
- Where:
  - bsp_illuminate.c:159 — lvgl_cfg.task_affinity = -1 (LVGL render+input unpinned)
  - prop_net.c:240 — xTaskCreate(rssi_task,...,3,NULL) unpinned, 1 Hz blocking esp_wifi_sta_get_ap_info() SDIO call (vTaskDelay(1000))
  - prop_engine.c:302 — xTaskCreate(animate_task,...,5,NULL) unpinned, 20 Hz
  - prop_ble.c:331 — xTaskCreate(prune_task,...,3,NULL) unpinned, ~2 Hz
  - prop_csi.c:151 — xTaskCreate(csi_task,...,4,NULL) unpinned, ~15 Hz
  - prop_mic.c:186 — xTaskCreate(mic_task,...,5,NULL) unpinned, ~10 Hz I2S+FFT
  - NimBLE host pinned to core 0 (sdkconfig CONFIG_BT_NIMBLE_PINNED_TO_CORE_0=y) while LVGL floats.
- Cost (mechanism): Software rendering saturates one HP core for a whole draw. An unpinned task
  scheduled onto LVGL's core mid-render preempts and stalls the frame — textbook cause of swinging
  frame times (cost model: "main consistency liability"). The 1 Hz RSSI poll is the smoking gun for
  the once-a-second hitch: a blocking SDIO transaction to the C6 that stalls a frame at ~1 Hz cadence
  when it lands on the render core — matches the user's description.
- Fix: Pin LVGL render+input to core 1 (task_affinity=1 at bsp_illuminate.c:159); move the
  background/radio tasks to core 0 via xTaskCreatePinnedToCore(...,0) (rssi, ble prune, csi, mic;
  NimBLE already on 0). Blocking-SDIO tasks (rssi especially) MUST be off the render core.
  Confidence: high it improves consistency (code-evident + cost model predicts); magnitude needs
  measurement.
- Risk: All radios + render on a 2-core part means core 0 carries WiFi+BLE+CSI+mic; if it saturates,
  heavy-screen throughput could dip even as consistency improves — measure spectrum FPS. No hard
  constraint violated (rssi already caches off the LVGL lock; keep it that way).

### [P2] Hybrid LVGL allocator — kill per-draw PSRAM allocation latency — affects: throughput (also smooths variance)
- Where: lv_port_mem.c:39 — lv_malloc_core() routes EVERY lv_malloc to PSRAM
  (LV_PSRAM_CAPS = MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT). LVGL 9 allocates anti-alias mask buffers via
  lv_malloc during drawing.
- Cost (mechanism): every line/shape draw pays PSRAM allocation latency, and PSRAM contention adds
  frame-time variance. Cost model names this as THE reason spectrum is ~8 vs ~18 fps — the single
  biggest throughput lever; CLAUDE.md "Memory reality" names the exact fix.
- Fix: Make lv_malloc_core/lv_realloc_core a hybrid allocator: try MALLOC_CAP_INTERNAL first for
  small/short-lived allocs (draw masks ~1 KB), fall back to PSRAM for large buffers and on internal
  pressure. Keep generous internal headroom for esp_hosted (budget: stay well under the ~332 KB
  internal free at runtime with all radios live).
- Win: plausibly closes much of the 8->18 fps gap on draw-heavy screens. Confidence: medium-high
  (mechanism/fix documented; threshold + headroom need on-device tuning).
- Risk: the constraint minefield. Do NOT restore the builtin pool or raise LV_MEM -> "HS_MP:
  mempool create failed: no mem" boot loop. If WiFi RAM gets tight, the lever is
  CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y, not shrinking reserves. Verify clean boot with all three
  radio instruments live after the change.

### [P3] Instrument bars update unconditionally every frame — affects: throughput (+ small steady-state hitch)
- Where (no shadow-compare, unlike the scanner trace):
  - SPECTRUM: prop_ui.c:2693-2704 — 32 bars x (set_height + align + set_bg_color) every frame
  - RF BAND: prop_ui.c:2715-2726 — same; data only changes on a (re)scan, so most frames are static yet still re-set/re-aligned
  - SIGNAL ENV (CSI): prop_ui.c:2758-2769 — same pattern
- Cost (mechanism): each set_height/align/bg_color write invalidates the bar's bbox, forcing a
  recomposite (and CRT-overlay alpha-blend) of that region every frame whether or not it moved. The
  scanner trace already solved this with a per-segment shadow-compare (prop_ui.c WAVE_SEGS,
  ~2541-2557); these three don't. RF BAND is worst — its data is static between scans.
- Fix: shadow-compare each bar's computed height/color, only write on change. lv_obj_align to a
  fixed bottom-left X never changes after build — hoist it to build time (only height grows). For
  RF BAND, gate the whole update on "decay still settling OR new scan data".
- Win: cuts steady-state invalidation substantially (RF BAND -> ~zero when idle). Confidence: high
  (direct analogue to the proven scanner-trace optimization).
- Risk: low; pure render-area reduction. Verify bars still animate smoothly.

### [P3] BLE contact list teardown/rebuild every ~400 ms — affects: consistency (periodic hitch) + throughput
- Where: prop_ui.c:2747 — lv_obj_clean(s_ble_list) + full ble_add_row() rebuild inside the
  st->tick % 8 == 0 block (~2.5 Hz). Each row is 4+ labels + a meter bar (ble_add_row, 1211-1246).
- Cost (mechanism): whole widget-hierarchy teardown+recreate ~2.5x/s churns the heap and invalidates
  the entire list bbox at once — a periodic hitch on CONTACTS, amplifying P2's allocation latency.
- Fix: keep a fixed pool of row widgets (PROP_BLE_MAX, built once at panel build); diff+update text/
  RSSI/distance/meter in place, hide unused rows. No clean/rebuild.
- Win: removes the ~2.5 Hz hitch on CONTACTS. Confidence: high (mechanism clear). Scoped to the
  CONTACTS screen.
- Risk: low; mind the close_panel NULLing discipline (prop_ui.c:259) so the pool tears down with the panel.

### [P3 / experiment] Second SW draw unit + partial internal-SRAM draw buffers — affects: throughput — needs measurement
- Where: sdkconfig CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1 (single SW draw unit, ASM_NONE);
  bsp_illuminate.c:188 buff_spiram=true (full-screen x2 double buffer in PSRAM, lines 173-175/187).
- Lever: with two HP cores, a 2nd SW draw unit (=2) can parallelize rendering; rendering INTO
  internal SRAM is far faster than PSRAM, and a partial N-line buffer fits internal RAM where the
  full-screen x2 (~2.4 MB) cannot.
- Fix (experiments, not guaranteed): (a) LV_DRAW_SW_DRAW_UNIT_CNT=2 — competes with radio tasks for
  the 2nd core, so COUPLED to the P1 affinity decision; measure together (if all radios are on core 0
  and LVGL on core 1, a 2nd draw unit has no free core). (b) A/B partial internal-SRAM draw buffers
  vs the full-screen PSRAM buffers for the redraw sizes this UI produces.
- Confidence: low without measurement.
- Risk: keep swap_bytes=false and full_refresh/direct_mode off; watch internal-RAM headroom (interacts with P2).

### [P3 / verify] CRT refresh band is a moving translucent ARGB8888 canvas — affects: throughput on every screen — needs measurement
- Where: prop_fx.c:196-207 — 1024x80 ARGB8888 canvas on lv_layer_top(), FLOATING, lv_obj_set_y
  every 55 ms (band_tick, 159-170).
- Cost (mechanism): it paints pixels directly into its own canvas buffer (correctly dodging the
  layer-recomposite trap the FPS HUD hit — intended per cost model/CLAUDE.md), but it's still a
  translucent 1024x80 stripe alpha-blended over the content ~18x/s. Author already bounds it thin
  (80 px, prop_fx.c:49). Working as designed.
- Fix: none recommended (don't break the look). For a max-FPS shot, turn the CRT overlay off
  (fx off) to remove the per-frame alpha cost.
- Confidence: mechanism code-evident; magnitude needs measurement (fx on vs off).
- Risk: altering it changes the aesthetic — out of scope to "fix".

## Already-maxed knobs — verified, not findings
CPU 360 MHz, PSRAM 200 MHz HEX, -O2, FreeRTOS 1000 Hz all confirmed at ceiling in sdkconfig. Do NOT
raise the CPU clock or drop the refresh period — the latter was tried and lifted nothing while making
the device network-sluggish (the FPS HUD already drops the period to 8 ms; passive counter on
purpose, prop_ui.c:2908-2925). Forcing a full-screen redraw every frame is a known dead end (~4 fps).

## Suggested order of attack
Consistency first (the user's #1), cheapest-high-impact first:
1. P1 core affinity — cheapest, biggest consistency win, directly targets the once-a-second hitch.
   One line in bsp_illuminate.c + swap xTaskCreate->xTaskCreatePinnedToCore in the radio/sensor
   files. Measure consistency before/after on a moving screen.
2. P3 instrument-bar shadow-compare (RF BAND, then spectrum/CSI) + P3 BLE row pool — localized,
   low-risk throughput+jitter wins mirroring the existing scanner-trace pattern. Group them.
3. P2 hybrid allocator — biggest throughput lever, highest constraint risk; do it alone with boot/RAM
   verification. Measure spectrum FPS before/after.
4. P3 experiments (2nd draw unit / partial internal buffers) — only after P1 (shared core decision);
   A/B individually.
Do not bundle P2 with anything else — if it boot-loops, isolate it.

## Measurement plan
NOTE: the skill's example uses {"cmd":"fx","on":true} to toggle the FPS HUD, but in THIS firmware
`fx` toggles the CRT overlay; the FPS HUD is toggled by prop_ui_set_fps / the SETUP "FPS METER"
switch (persisted as fps_on, prop_ui.c:719,2970). Turn the HUD on via SETUP before measuring; use
fx on/off separately to A/B the CRT-overlay cost.

  cd firmware/communicator
  python tools/prop.py state                 # confirm comm-unit-7.local reachable
  # Per-screen baseline: open each, settle, read top-right FPS HUD over ~10 s. Watch the SWING.
  python tools/prop.py shot base_spectrum.png --screen spectrum --wait
  python tools/prop.py shot base_rfband.png   --screen rfband   --wait
  python tools/prop.py shot base_ble.png      --screen ble      --wait
  python tools/prop.py shot base_scanner.png  --screen scanner  --wait
  # Isolate CRT-overlay cost (P3 fx): A/B same screen overlay on/off.
  python tools/prop.py fx on 70
  python tools/prop.py shot spec_fx_on.png  --screen spectrum --wait
  python tools/prop.py fx off
  python tools/prop.py shot spec_fx_off.png --screen spectrum --wait
  # Touch/dial latency proxy:
  curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"input","control":"selector","arg":"cw"}'
  curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"input","control":"action","arg":"press"}'
  # Rebuild+flash after EACH change, re-run the same captures to A/B:
  ./tools/dev.sh bf -Port /dev/ttyUSB0       # (Windows: pwsh tools/dev.ps1 bf -Port COM7)

A/B protocol: P1 -> watch the once-a-second hitch on the SCANNER/SPECTRUM HUD before vs after
pinning (should shrink/disappear). P3 -> compare RF BAND/spectrum/CONTACTS steady-state FPS + swing.
P2 -> compare spectrum FPS (target the 8->18 gap) AND confirm clean boot with all radios live
(/state shows ble:{count} and csi_live).

## Cross-check against hard constraints (all fixes verified safe)
- P2 keeps large buffers in PSRAM + leaves internal headroom — no builtin pool, no LV_MEM raise.
- No fix introduces a WiFi/SDIO call under the LVGL lock; rssi/uplink stay cached off-lock.
- swap_bytes=false, no PPA/GPU path, no CONFIG_ESP32P4_*REV* change, no canvas layer-draw/lv_snapshot
  from a non-LVGL task.
