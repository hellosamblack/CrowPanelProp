# PPA Phase B — Live CRT-Overlay Integration: Resume Handoff

**Date paused:** 2026-06-27
**Branch:** `feat/ppa-reenablement-framerate` (NOT merged, NOT pushed)
**Status:** Phases 0, A, C1, C2, B-alt + the Phase-B *spike* are DONE, verified on hardware, and committed. Only the **live Phase B overlay integration** (the risky part) remains. The spike proved it's worth doing (7.29× faster blend). This doc is a self-contained briefing so a fresh agent can resume with no prior context.

Parent plan: `docs/superpowers/specs/2026-06-27-ppa-reenablement-framerate-plan.md`.
Live progress ledger (terse): `.superpowers/sdd/progress.md`.

---

## 0. TL;DR of what to do next

Integrate the PPA **A8 blend** into the live CRT overlay so the per-frame full-screen alpha blend runs on the PPA (≈12 ms) instead of the CPU (≈89 ms). The offline spike already proved correctness + 7.29× speedup. The hard part is **where/when** to run the blend on this board's display pipeline without tearing or deadlock. Expect this to require moving the DPI panel from `num_fbs=1` to `num_fbs=2` and hooking the blend at frame completion. Build/flash/eyeball loop is controller-driven (see §6). The overlay is **OFF by default**, so nothing regresses for normal use while you iterate.

---

## 1. Board / environment (read once)

- Hardware: **CrowPanel Advance ESP32-P4 7" HMI**, **chip rev v1.3**, 1024×600 RGB565 IPS, ESP-IDF **6.0.1**, LVGL **9.4**, esp_lvgl_port 2.8.
- CPU is at **360 MHz** (v1.3 rating; `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=360`). NOT overclocked. Do not bump to 400 (v3.x only).
- Board serial port: **COM4** on this host (the CLAUDE.md says it varies COM4/COM7 — confirm with `[System.IO.Ports.SerialPort]::GetPortNames()` before flashing).
- mDNS host: **comm-unit-7.local** (STA IP was 172.17.2.168). `/screenshot`, `/cmd`, `/state` HTTP endpoints all work.
- IDF on this host: `C:\esp\v6.0.1\esp-idf`; PPA driver header: `C:\esp\v6.0.1\esp-idf\components\esp_driver_ppa\include\driver\ppa.h`.
- **No automated test suite.** Verification = build clean + live `/screenshot` + on-device FPS HUD + user eyeball.
- The **"broken v1.3 silicon" belief is FALSE** — confirmed. PPA works once LVGL bugs are fixed (see §3). Errata show no PPA/DSI/L2/2D-DMA defect on v1.3.

---

## 2. What is already DONE + committed (do NOT redo)

Branch commits (newest first), all hardware-verified:

| Commit | Phase | What |
|---|---|---|
| `a5a86ee` | B step 1 | **PPA A8 blend spike** (offline harness, `main/prop_ppa_spike.c/.h`, `/cmd fx ppaspike`). Proved 7.29×. |
| `c7b0a9e` | B-alt | `esp_lcd_dpi_panel_enable_dma2d(panel_handle)` in `bsp_illuminate.c` (log-and-continue; may be a no-op in direct-fb mode). |
| `1fb9b7b` | C2 | Removed redundant per-frame `lv_obj_align` + gated bar recolor behind band shadows on spectrum/rfband/csi (`prop_ui.c`). |
| `be7d8e3` | C1 | Pinned imu/radar/csi/calib/traffic tasks to core 0. |
| `b68e7cf` | 0 | Enabled `CONFIG_LV_USE_PPA` + **fixed 3 real upstream LVGL PPA bugs** + 64B draw-buf alignment + esp.cmake dep fix. |

**FPS measured (FPS HUD, `/cmd {"cmd":"fx","fps":true}`):**
- Baseline (PPA off, overlay off): spectrum 14, csi 8, rfband 21.
- After all of the above (PPA on + C-stack): spectrum **18**, csi ~6–9 (data-dependent), rfband idles to ~1 when static (C2 stopped it wastefully redrawing identical bars — this is a WIN, not a regression).
- Note: PPA *fill* acceleration alone is ~neutral on this text/vector UI (as the parent plan predicted). The real wins were C2 (churn) and the (pending) overlay blend.

**Everything renders 100% correctly** on the physical panel (user-confirmed after the §3 fixes).

---

## 3. The 3 LVGL PPA bugs already fixed (context — these are upstream defects in LVGL 9.4's *experimental* Espressif PPA draw unit)

All patched in `managed_components/lvgl__lvgl/...` which is **git-tracked inline in this repo**, so the patches are durable. They are documented with `LOCAL PATCH (CrowPanelProp...)` comments. **If the lvgl managed component is ever re-resolved (`idf.py update-dependencies`), these must be re-applied.**

1. **`env_support/cmake/esp.cmake`**: `esp_driver_ppa`/`esp_mm` were gated behind `if(CONFIG_LV_USE_PPA)`, which isn't evaluated during IDF's dependency-graph expansion pass → `driver/ppa.h` not found. Made the dependency **unconditional**.
2. **`src/draw/espressif/ppa/lv_draw_ppa_fill.c`**: `out.pic_w/pic_h` used the *fill* size instead of the draw-buffer size (wrong row stride) and `block_offset` was hardcoded 0 (ignored the fill's position within the layer) → garbled every sub-region fill (the always-present rail/top/footer chrome) while full-area fills looked fine. Fixed to `header.w/h` + `block_offset = coords - buf_area` + cache-line-aligned `buffer_size` (mirrors the sibling `lv_draw_ppa_img.c`). **Also**: op mode was hardcoded `PPA_TRANS_MODE_NON_BLOCKING` while the dispatch's blocking path (`#if !LV_PPA_NONBLOCKING_OPS`) marks the task FINISHED without waiting for the ISR → async fill DMA raced the SW border/text draw → fragmented small fills (the BACK button). Made mode follow `LV_PPA_NONBLOCKING_OPS` (BLOCKING for our config, which is the default; `LV_PPA_NONBLOCKING_OPS=0`).
3. **`src/draw/espressif/ppa/lv_draw_ppa.c`** (`ppa_execute_drawing`): `draw_area` was used **uninitialized** before `lv_area_move()` → pre-op cache invalidation hit a bogus region. Seed `draw_area = area` before moving.

**Build config added** (in `sdkconfig.defaults`, commit `b68e7cf`): `CONFIG_LV_USE_PPA=y`, `CONFIG_LV_DRAW_BUF_ALIGN=64`, `CONFIG_LV_ATTRIBUTE_MEM_ALIGN_SIZE=64` (the PPA draw unit hard-`#error`s unless these equal the 64 B L1 cache line). `LV_DRAW_BUF_STRIDE_ALIGN` stays 1 (prop_fx's hand-packed ARGB canvas needs a `w*4` stride).

---

## 4. The spike (already built/committed) and its result — your evidence base

- File: `main/prop_ppa_spike.c` (+ `.h`). Entry: `void prop_ppa_spike_run(void)`. Triggered by HTTP `POST /cmd {"cmd":"fx","ppaspike":true}` (wired in `prop_api.c`; `esp_driver_ppa` added to `main/CMakeLists.txt` REQUIRES).
- It is **offline only**: allocates 64B-aligned PSRAM scratch buffers (RGB565 bg + A8 mask + RGB565 out), bakes the scanline+vignette geometry from `prop_fx.c` as an **A8 alpha mask** with **fixed black fg** (`fg_fix_rgb_val={0,0,0}`, `in_fg.blend_cm=PPA_BLEND_COLOR_MODE_A8`, `fg_alpha_update_mode=PPA_ALPHA_NO_CHANGE`), blends 100× (blocking) via `ppa_do_blend`, vs 100× SW reference, with `esp_cache_msync` (C2M on inputs before, M2C on output after).
- **Result @ rev v1.3 / 360 MHz:** `PPA = 12214 us/op, SW = 89096 us/op, ratio = 7.29x`. Checksum differs from SW by expected rounding / RGB888-internal colorspace conversion (visually equivalent for an overlay — not a bug).
- **The spike's blend config in `prop_ppa_spike.c` is your known-good template** for the live blend (correct ppa.h field names, A8 setup, cache sync). Reuse it.
- The spike can stay in the tree (handy regression check) or be removed once the live integration lands — your call; if removed, also revert the `/cmd ppaspike` branch + the `esp_driver_ppa` REQUIRES if nothing else needs it.

---

## 5. The remaining task: LIVE overlay integration (what to actually build)

**Goal:** Replace the current CPU-software CRT overlay composite with a PPA A8 blend onto the framebuffer at frame completion, so enabling the CRT effect no longer craters FPS.

### 5.1 How the overlay works today (`main/prop_fx.c`, ~484 lines — read it fully)
- The static overlay = scanlines + amber phosphor wash + edge vignette, baked into a **full-screen ARGB8888 `lv_canvas`** on **`lv_layer_top()`** (composited above every screen by LVGL's SW renderer every frame). Plus a thin scrolling "refresh band" (separate small floating canvas + slow timer).
- It paints pixels **by hand** (`fx_fill`, src-over into the ARGB buffer) because LVGL v9 canvas layer-draw deadlocks under the port lock from a non-LVGL task.
- Lazily allocated when enabled; off-by-default (`fx_on` NVS default 0).
- Per-effect levels: `fx_scan`, `fx_phosphor`, `fx_vignette`, `fx_refresh` (0–100, NVS).

### 5.2 The plan's intended approach (parent plan §B, steps 2–3)
1. **Rework the overlay for A8** (bandwidth lever — this is what made it win):
   - Bake the **amber phosphor wash into the screen background** (flat tint — free; do it where the screen bg color is set).
   - Store **scanlines + vignette as an A8 alpha mask** (1 B/px) with **fixed RGB = black** (the spike already builds this exact mask — reuse `bake_a8_mask` logic).
   - Keep the **amber refresh band** as a small separate ARGB pass (it's small/cheap), or fold it in later.
2. **Blend on the flush path**: after LVGL renders a frame and before the panel scans it out, `ppa_do_blend(in_bg=framebuffer RGB565, in_fg=A8 mask + fixed black, out=framebuffer)`. Drop the `lv_layer_top` SW composite. Keep the static-vs-animated split.

### 5.3 THE HARD PROBLEM (why this was paused for a decision) — the display pipeline
- **`num_fbs = 1`** today (`bsp_illuminate.c` `dpi_config.num_fbs = 1`). The DPI peripheral **continuously scans out that single FB**. `/screenshot` reads it directly (`prop_api.c` `esp_lcd_dpi_panel_get_frame_buffer(panel,1,&fb)`), which is why `/screenshot` == what's on the panel.
- Render mode is **partial double-buffer** (`double_buffer=true`, `full_refresh=false`, `direct_mode=false`; the `CONFIG_DISPLAY_LVGL_*` Kconfig symbols are NOT defined in this project — they only exist in `reference/` examples, so those `#if` branches are always false). LVGL has 2 *draw* buffers but the panel has 1 *scanout* FB; flush copies dirty regions into it via `esp_lcd_panel_draw_bitmap`.
- **Blending into a single continuously-scanned FB causes tearing** (you'd be writing pixels the DPI is actively reading). There is no vblank/back-buffer to hide the blend.
- **`full_refresh=true` ASSERTS at boot** in this config: `assert failed: xQueueSemaphoreTake queue.c:1709` right after "Starting LVGL task" → boot loop. esp_lvgl_port's full_refresh path waits on a vsync/swap semaphore that only exists with `num_fbs>=2` / `avoid_tearing`. (Confirmed experimentally — don't repeat it without also setting num_fbs=2.)

### 5.4 Recommended integration path (de-risked, incremental — do in this order, build/flash/eyeball each)
1. **Move to `num_fbs = 2`** (`bsp_illuminate.c` dpi_config) and enable the tear-avoidance path: set `avoid_tearing = true` in `lvgl_dpi_cfg.flags` and consider `direct_mode = true` (forces draw buffers == the 2 panel FBs). This gives a back buffer + the vsync semaphore the port needs. **Cost:** ~1.2 MB more PSRAM (fine — 32 MB PSRAM, but re-check internal-RAM headroom; the sanctioned lever if tight is `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`, never shrink internal). **Verify:** boots clean (no `HS_MP` mempool loop, no queue assert), renders correct, no tearing, measure FPS. This is the "Smaller step first: num_fbs=2" option the user was offered — it's the prerequisite. NOTE `/screenshot` reads FB index from `prop_api.c` — with 2 FBs it may capture the non-displayed buffer; check/adjust `prop_api.c` capture if screenshots look stale (read the *currently-scanned* fb index).
2. **Hook the blend at frame completion**: register a PPA blend client once at init; on `LV_EVENT_RENDER_READY` (or the esp_lvgl_port flush-ready callback — runs IN the LVGL task, so no lock re-entry / no deadlock), after LVGL has rendered the back buffer and before it's swapped to scanout, run the A8 PPA blend (mask onto the back buffer). Then let the normal swap present it. Blocking PPA op (~12 ms) inside the LVGL task is acceptable since it replaces the ~89 ms SW composite; revisit non-blocking only if 12 ms blocking hurts.
3. **Rework `prop_fx.c`**: bake amber wash into the screen bg; build the A8 mask once (re-bake on level change, like today's `rebake_canvas_locked`); drop the `lv_layer_top` static canvas composite; keep the refresh band (small ARGB pass or fold into mask later). Allocate the A8 mask 64B-aligned in PSRAM; `esp_cache_msync` C2M after (re)baking, and the framebuffer needs the right coherency around the blend (the spike shows the pattern).
4. **Verify thoroughly**: overlay ON vs OFF FPS on a moving screen (e.g. spectrum) via the HUD; eyeball the physical panel for tearing/color/alpha correctness (`/screenshot` will now ALSO capture the overlay once it's blended into the FB — a verification bonus vs today where the lv_layer_top overlay is invisible to `/screenshot`). Confirm no boot regressions and BLE/WiFi still up.

### 5.5 HARD CONSTRAINTS (a change that breaks one is a regression — from CLAUDE.md + this session)
- Don't run the draw pipeline / canvas layer-draw / `lv_snapshot` under the LVGL lock from a non-LVGL task → deadlock. Run the blend from the LVGL task's flush/render-ready callback, or a path that doesn't hold the lock incorrectly.
- Don't restore the LVGL builtin pool / raise `LV_MEM` → `HS_MP` mempool boot loop. LVGL heap stays in PSRAM (`lv_port_mem.c`).
- Don't touch `CONFIG_ESP32P4_*REV*` (board is v1.3).
- Keep `swap_bytes=false` (native RGB565; true → magenta background).
- Never call WiFi/SDIO APIs under the LVGL lock.
- PPA buffers (mask + FB) must be **64 B aligned** (address and size) — `heap_caps_aligned_alloc(64,...)`, `PPA_ALIGN_UP(size, CONFIG_CACHE_L1_CACHE_LINE_SIZE)`.
- `main/CMakeLists.txt` GLOBs `main/*.c` → run `idf.py reconfigure` after adding any new `main/*.c` file (else `undefined reference`).

---

## 6. Build / flash / measure recipe (controller-driven; PowerShell on this host)

```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure   # only after adding/removing main/*.c or config changes
idf.py -C "f:\git\personal\CrowPanelProp" build
idf.py -C "f:\git\personal\CrowPanelProp" -p COM4 flash  # confirm port first
```

Screenshots / FPS / spike trigger (bash tool; board must be up):
```bash
python tools/prop.py state                                              # readiness / version
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'   # FPS HUD on (top-right of panel)
python tools/prop.py shot out.png --screen spectrum --wait              # navigate + capture
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'    # CRT overlay ON (to test the blend)
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","ppaspike":true}'  # re-run the offline spike
```
- Read FPS off the captured PNG (HUD renders "FPS N" top-right). `/screenshot` reads the FB raw (no vsync) so fast-animated elements look doubled — that's a known capture artifact, NOT a render bug (see `screenshot-fast-animation-ghosting` memory).
- To read serial (boot logs / spike banner / asserts), open COM4 via `System.IO.Ports.SerialPort` in PowerShell (115200) and read for ~6–15 s; filter for `PPA|assert|Guru|abort|E (|rst:`.
- Decode a crash PC: `python tools/prop.py decode` (or trace) against `build/communicator.elf`.

---

## 7. Decision log / why paused
- User authorized "everything incl Phase B" at the start, then (after the spike succeeded and the single-FB/num_fbs=2 complexity surfaced) chose to **save a handoff and resume later** rather than push the invasive flush-path rework in the same session. The durable wins (Phase 0 correctness + 3 bug fixes, C1, C2, B-alt, spike) are all committed. The live integration is the only remaining item and is cleanly separable.
- Recommended resume entry point: **§5.4 step 1 (num_fbs=2 + avoid_tearing) as a standalone, verified change first**, then the blend hook, then the prop_fx rework.

## 8. Finishing up (when the live integration is done)
- Use `superpowers:finishing-a-development-branch` to decide merge/PR. Branch `feat/ppa-reenablement-framerate` is not pushed.
- Consider whether to keep `main/prop_ppa_spike.c` + the `/cmd ppaspike` hook (useful regression probe) or remove it.
- Worth a memory: the v1.3 PPA premise is disproven + the 3 LVGL PPA bugs (so a future agent doesn't re-litigate "broken silicon"). Possibly pin `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_360=y` in `sdkconfig.defaults` as a guardrail (offered to user; not yet done).
