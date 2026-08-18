# Agent Handoff: Post-Audit Implementation — State, Deviations, and Recovery (2026-08-18)

> **For the incoming agent.** This continues `docs/2026-08-18-agent-handoff-prompt.md` (the
> original execution order). All three phases are **implemented and build clean**, but the
> board went dark after the first OTA and is **waiting on a human power cycle**. Read this
> whole doc before touching anything — especially "Board state" and "Deviations".

## 1. Board state (the blocking issue)

- First OTA (all 3 phases, including a since-removed PPA blend) was pushed at ~2026-08-18
  and acknowledged (`{"ok":true,"rebooting":true}`). The board **never came back**: no mDNS
  (`comm-unit-7.local`), no ping at its last IP (172.17.2.77), full `/24` sweep found nothing,
  for 20+ minutes.
- Two candidate causes, **both recovered by one power cycle**:
  1. **Boot hang in the (now-removed) PPA blend**: `prop_fx_init` runs in `app_main` *before*
     WiFi init and *before* `esp_ota_mark_app_valid()` (main.c:253). A blocking
     `ppa_do_blend` hang there = silent stall → no panic → no rollback → dark board.
     Power cycle → bootloader sees PENDING_VERIFY → **rolls back to the previous working
     firmware** → board online on the OLD image.
  2. **C6 radio wedge on warm reboot** (known esp_hosted quirk): new firmware runs fine,
     radio dead. Power cycle → cold boot → board online on the NEW (risky) image — which
     then having survived one full boot proves the fx path didn't hang (or `fx_on=0`).
- **What the human sees on the screen distinguishes these**: frozen/incomplete UI → hang
  (scenario 1); live SCANNER UI → radio wedge (scenario 2). Either way proceed identically.

### Recovery procedure (run after the power cycle)

The current `build/communicator.bin` is the **fixed** build (PPA removed, see Deviations).
If a prior session's watcher isn't running, do it manually:

```bash
# wait for the board, then push the fixed build
until curl -s -m 3 http://comm-unit-7.local/state | grep -q scene; do sleep 10; done
. ~/.local/esp/esp-idf/export.sh && ./tools/dev.sh ota     # rebuild + push (or curl the bin directly)
# after reboot (~60-90 s incl. WiFi join):
python3 tools/prop.py state && python3 tools/prop.py telemetry
```

If the board comes back but a rebuild is needed first: `idf.py build` from repo root
(IDF 6.0.1 at `~/.local/esp/esp-idf`). **This host has NO serial to the board**
(`/dev/ttyACM0` is an ST-LINK) — OTA is the only flash path. If the board doesn't return
after a power cycle, STOP and ask the human for serial access (see the human handoff doc).

## 2. What is implemented (working tree, UNCOMMITTED)

All edits are in the working tree; **nothing is committed**. `git diff --stat` ≈ 13 files.
Build verified clean (`idf.py build` rc=0, no warnings in touched files).

### Phase 1 (all done as specified)
- **1.1** `components/bsp_io/bsp_io.c`: LED_POWER/LED_SIGNAL (GPIO48/47) and BTN_ACTION
  (GPIO11=CSI_RESET) set to gpio `-1` sentinel; `led_init`/`bsp_io_led_set`/`register_button`
  skip unassigned slots safely (mask API still works).
- **1.2** Orphaned `docs/hardware/schematic/kandle` gitlink removed (`git rm --cached`);
  `git submodule status` works again. (Note: actual submodules here are `docs/referenceDesign`
  and `submodules/lidar-roomscanner`, not `reference/` as CLAUDE.md claims.)
- **1.3** `main/prop_net.c`: `s_uplink_mux` portMUX added; `rssi_task` builds a local struct
  (WiFi/SDIO calls stay OUTSIDE the critical section) and publishes under the lock;
  `prop_net_get_uplink` reads under the lock.
- **1.4** `main/prop_api.c`: `cmd_post_handler` heap-allocates up to `CMD_BODY_MAX` (2048);
  `ws_handler` buffer 256→1024 (httpd stack is 8192, fine).
- **1.5** `main/prop_ble.c`: `s_scan_restart_pending` latch + retry from `prune_task` (~2 Hz);
  `start_scan` treats `BLE_HS_EALREADY` as success; DISC_COMPLETE handler now routes through
  `start_scan` (forward-declared).
- **1.6** `tools/prop.py`: `lidar` added to the docstring screen list (goto name verified
  present in `prop_ui.c`).

### Phase 2 (partially pre-existing — audit was stale in places)
- **2.1** LVGL task was **already** pinned to core 1 (`bsp_illuminate.c:178`) and
  `animate_task` **already** pinned to core 0 — no change needed. What WAS moved to core 0:
  `prop_track` (was unpinned), `telemetry_task` in prop_api (unpinned), `prop_lidar`
  (unpinned), `prop_motion` (was core 1!), both `prop_aux_radar` tasks (were core 1 via
  `TASK_CORE`).
- **2.2** Already implemented upstream (`xTaskDelayUntil` at prop_engine.c:~305) — no change.
- **2.3** `main/lv_port_mem.c`: `LV_SRAM_THRESHOLD` 512→1024. (esp_hosted's mempool is in
  PSRAM via `CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM`, so the internal-RAM starvation risk
  from this is low — see crash-forensics skill.)
- **2.4** **DEVIATION — see §3.**
- **2.5** **DEVIATION — implemented, then REVERTED after review**: LVGL 9.4's
  `lv_obj_set_height` already early-outs on an unchanged value (verified in the vendored
  `lv_obj_pos.c`), so the shadow caches only saved a style-prop lookup while adding a
  lifecycle hazard (a forgotten reset = frozen bars on reopen). The pre-existing color-band
  shadows (`s_spec_band` etc.) remain — style-color sets have NO such early-out.

### Phase 3 (all done as specified)
- **3.1** `main/include/prop_lidar.h`: telemetry struct gains `heading_deg`, `pitch_deg`,
  `roll_deg`, `yaw_rate_dps`, `orientation_valid`, `ir_grid[64]`, `has_ir_grid`;
  `power_mode` + `i3c_airtime_pct` removed (only consumer was the LIDAR panel itself;
  `/telemetry` JSON in prop_api.c never exposed them). `prop_lidar.c::on_telemetry_json`
  parses all new fields (ir_grid: exactly-64-element array, values clamped 0..255).
- **3.2** LIDAR sidebar rebuilt in `build_lidar_panel`: LINK/FPS/POINTS/REC rows;
  `s_lidar_hdg` ("HDG 248.5° WSW" via `compass16()`, COL_AMBER when `orientation_valid`,
  dim "HDG ---" otherwise); `s_lidar_attitude` ("P +x.x° R +y.y°"); `s_lidar_stab` gimbal
  yaw-rate verdict (STEADY <2 dps / SLEWING <20 / FAST, colored); 160×120 IR preview canvas
  (`ir_thermal_rgb565` black→red→yellow→white ramp, 20×15 px cells, repaint only on grid
  change via `s_lidar_ir_last` memcmp); full-width REC button pinned to sidebar bottom.
- **3.3** **Zero-copy** (hardened after review): `prop_lidar_get_frame` (460 KB memcpy)
  replaced by `prop_lidar_peek_frame` returning the PSRAM front-buffer pointer.
  `prop_lidar` now keeps a **triple** buffer (writer fills `(front+1)%3`, so the two most
  recently published frames are never scribbled on — closes the render-race window), and
  the 460 KB rx→back memcpy happens outside the lock. On the UI side the canvas is created
  HIDDEN with no buffer (no 460 KB placeholder at all); the first frame attaches via
  `lv_canvas_set_buffer` ONCE (that call re-runs the whole image-source setup + layout
  dirty — too heavy per-frame, verified in vendored LVGL), and every later frame only swaps
  `lv_canvas_get_draw_buf()->data` + invalidates the canvas. The bezel container owns the
  border and the orbit-drag handler. Sidebar text throttled to `st->tick % 4` (5 Hz), with
  color sets banded behind shadow values. `close_panel` must never free prop_lidar's
  buffers (it doesn't).

## 3. DEVIATION from the original handoff: Task 2.4 (PPA CRT blend)

**Not wired. Replaced with a LUT bake that achieves the same goal safely.**

- The spike (`main/prop_ppa_spike.c`, still intact) verified A8-over-**RGB565**. The fx
  canvas needs **ARGB8888 output with correct alpha** — an unverified mode, with an open
  premultiplication question on the PPA's output alpha semantics.
- `prop_fx_init` sits on the boot path BEFORE `esp_ota_mark_app_valid`. A blocking PPA hang
  there is unrecoverable over OTA (rollback needs a reset; a hang never resets). This exact
  risk is the leading suspect for the current dark-board incident (the first OTA contained
  the PPA version).
- Replacement (in `main/prop_fx.c::paint_canvas`): every overlay pixel is "black @ alpha a
  src-over a constant wash", so the result depends only on `a` → precompute a 256-entry
  alpha→ARGB LUT per bake, per-row/per-column alpha composition, one table lookup per pixel.
  Removes ALL per-pixel src-over math (the old ~40 ms loops) with zero hardware risk.
- **Verified offline** by pixel-exact simulation of old-vs-new at full 1024×600 across 5
  intensity configs (`tools/fx_bake_sim.py`, kept in-repo): bit-exact for wash-only; worst
  deviation elsewhere is 5/255 on alpha, confined to vignette-overlap corners, caused by
  truncation-order differences — imperceptible.
- **If real PPA blending is still wanted later**: verify ARGB8888-out blend semantics with
  `POST /cmd {"cmd":"fx","ppaspike":true}` extended to the ARGB case, on serial, post-boot —
  never on the boot path first. See the `ota-boot-path-rule` memory.

## 4. Environment notes (this Linux host)

- ESP-IDF v6.0.1 installed at `~/.local/esp/esp-idf` (this session). Build from repo root.
- **lvgl managed component needs a local patch after any fresh download**: in
  `managed_components/lvgl__lvgl/env_support/cmake/esp.cmake`, make
  `set(IDF_COMPONENTS esp_driver_ppa esp_mm esp_timer log)` unconditional (the
  `CONFIG_LV_USE_PPA` conditional isn't visible during early requirement expansion →
  `fatal error: driver/ppa.h`). `managed_components/` is gitignored, so this does not
  persist through re-downloads.
- No serial to the board from this host. OTA only (`./tools/dev.sh ota`, token in dev.sh).

## 5. Remaining verification (blocked on the board returning)

Run the original handoff's §5 smoke plan, plus these change-specific checks:

1. `python3 tools/prop.py state` / `telemetry` — confirm boot, uptime, module health.
2. **CRT overlay visual parity**: enable fx (`/cmd {"cmd":"fx","on":true}` or SETUP→DISPLAY),
   `python3 tools/prop.py shot fx.png --wait` — scanlines/vignette/wash must look unchanged.
   Exercise the sliders (scan/phosphor/vignette) to force LUT re-bakes.
3. **LIDAR panel**: `python3 tools/prop.py shot lidar.png --screen lidar --wait` — check the
   new sidebar layout (4 telemetry rows, HDG block, GIMBAL row, IR PREVIEW, full-width REC).
   With the roomscanner rig live, confirm frames render (zero-copy path) and HDG/IR populate.
   With it offline, panel must show SEARCHING + dim placeholders.
4. **Task 1.4**: POST a >256-byte JSON to `/cmd` (was 400 before) and confirm 200.
5. **Spectrum/scanner screens**: `shot spectrum.png --screen spectrum`, `shot scanner.png
   --screen scanner` — bars animate, no regressions from the shadow-compare.
6. Watch for LD2450/aux-radar regressions from the core-0 move (`prop.py watch --only radar`).
7. **Mock-rig exercise** (works without the real roomscanner): run
   `python3 tools/mock_ws_thin.py` on a LAN host (or set NVS `lidar_host`) — it now emits
   the full new telemetry (sweeping heading, breathing pitch/roll, `orientation_valid`,
   an orbiting warm blob in `ir_grid`) so HDG/GIMBAL/IR PREVIEW visibly animate.
8. With `LV_SRAM_HEADROOM` in play, log `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)`
   once WiFi+BLE+LIDAR are live to confirm the 128 KB floor is comfortably above water.

## 5b. Code-review outcomes (already applied this session)

A `/code-review high` pass ran over the diff; all 10 findings were addressed and the build
re-verified clean:

1. `/cmd` body now drained in a recv loop (multi-segment 2 KB POSTs no longer truncate).
2. Per-frame `lv_canvas_set_buffer` replaced with one-time attach + data-pointer swap.
3. LIDAR frame race closed with a triple buffer in `prop_lidar.c`.
4. `docs/gpio_registry.yml` GPIO11 entry updated to reserved/CSI_RESET; stale
   `bsp_io.h` LED_POWER comment fixed. (GPIO47/48 registry entries were already correct.)
5. `lv_malloc_core` SRAM fast path gated on 128 KB internal-RAM headroom
   (`LV_SRAM_HEADROOM`) so persistent 513–1024 B allocations can't erode esp_hosted's room.
6. 460 KB placeholder canvas buffer eliminated (hidden canvas until first frame).
7. LIDAR sidebar color sets banded behind shadow values (no same-value early-out in LVGL
   style props).
8. IR raster tied to `PROP_LIDAR_IR_CELLS` via `LIDAR_IR_GRID` + `_Static_assert`.
9. Task 2.5 height caches REVERTED (LVGL already early-outs unchanged heights; see §2).
10. `tools/mock_ws_thin.py` updated to the new telemetry schema with animated
    orientation + IR grid.

## 6. When done

Commit (working tree currently dirty, includes the `kandle` index removal), then consider
updating the module-map line for `prop_lidar` in CLAUDE.md/AGENTS.md (get_frame → peek_frame,
new telemetry fields). Original audit + plan docs remain untracked in `docs/`.

---

## 7. RESOLVED (2026-08-18, later session) — recovery complete

The board was never dead: NVS had no STA creds and the Linux-built OTA image had no baked
defaults (`wifi_secret.env` was missing on this host), so it sat in AP-only mode, invisible
on the LAN. Serial access was gained (CH340 passed into the LXC container as `/dev/ttyUSB0`),
the fixed build was serial-flashed (boots from **factory**; otadata reset), and creds were
baked (`idf.py reconfigure` required — see the `linux-build-host` memory).

**The "graphical glitches" (missing rail, grey/fragmented buttons, garbled chrome, flicker)
were NOT from this change set.** Root cause: stock LVGL 9.4.0 downloaded fresh on this host
still contains the 3 PPA draw-unit bugs fixed in commit `b68e7cf9`; the repo reorg
(`0ad5e09b`) had untracked `managed_components/`, silently discarding those patches. Fixed by
restoring the patched files from git history — now automated as `tools/apply_lvgl_patches.sh`
(run after ANY fresh component download). §5 verification passed: all screens render
correctly, LIDAR sidebar as designed (SEARCHING state), >256 B `/cmd` returns 200, fx overlay
visually intact, LD2450/IMU telemetry live after the core-0 moves. Working tree (audit
changes + this script) still awaits commit per §6.
