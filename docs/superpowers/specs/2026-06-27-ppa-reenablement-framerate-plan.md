# PPA Re-enablement & Framerate Implementation Plan

**Date:** 2026-06-27
**Status:** Implementation plan — ready to execute
**Supersedes:** `2026-06-20-ppa-acceleration-research.md` (written under LVGL 8.4, before the v9 migration; its core "PPA can't reach full-screen 60 fps" point stands, but its blocking objection — "needs a v8→v9 migration" — is now resolved)
**Board:** CrowPanel Advance ESP32-P4 7" HMI, **chip rev v1.3**, 1024×600 RGB565, LVGL 9.4, ESP-IDF 6.0.1

---

## 1. Why this plan exists (the premise correction)

The project disabled the PPA on the belief that v1.3 silicon is "flawed." **The official Espressif errata do not support that belief.** This plan re-enables the PPA *intentionally*, scoped to the operations that are genuinely useful and safe, and pairs it with complementary non-PPA framerate work.

### 1.1 Evidence (three Espressif sources, cross-checked)

| Claimed v1.3 PPA hazard | Errata reality |
|---|---|
| PPA monopolizes AXI bus → starves MIPI-DSI → tearing | **No such erratum on any revision.** |
| 64 B vs 128 B L2 cache-line alignment conflict | **No such erratum.** Our L2 line is already 64 B (`sdkconfig` `CONFIG_CACHE_L2_CACHE_LINE_64B=y`, line 2655). |
| RGB888 PPA color-space logic failure | **No such erratum.** |
| Sub-cache-line dirty / PSRAM stale DMA reads | **MSPI-750 exists but affects v3.0 ONLY (fixed v3.1). Does NOT affect v1.3.** |

For **v1.3 specifically**, the documented errata are RMT-176 (RMT idle), I2C-308 (I2C slave), and APM-560 (unauthorized-AHB access stalls — only on a permission violation, not normal rendering). **None touch the PPA, MIPI-DSI, L2 cache, or 2D-DMA path.**

What v3.x actually changed for the PPA — **32×32 block processing and YUV422/420 support** — is a *feature addition for video fluidity*, **not** a v1.3 defect repair. A text/vector communicator UI uses neither; v1.x PPA's RGB565 fill/blend/SRM is all this UI needs.

The accurate PPA reference for this board is `docs/ESP32-P4 v1.3 PPA Optimization & Debugging Guide.md` (rewritten to match the errata above).

### 1.2 The real reason PPA was abandoned

The `2026-06-20` spike evaluated PPA under **LVGL 8.4, which had no ESP32-P4 PPA draw backend at all**. "PPA didn't help" then meant "there was no PPA draw path." That blocker is gone: the project is on **LVGL 9.4**, and the v9 PPA draw unit (`managed_components/lvgl__lvgl/src/draw/espressif/ppa/lv_draw_ppa*.c`) is present and auto-wired (`lv_init.c:272` calls `lv_draw_ppa_init()` when `LV_USE_PPA`). It is simply gated off (`sdkconfig:4953` `# CONFIG_LV_USE_PPA is not set`).

**Conclusion: re-enabling PPA on v1.3 is a software-integration task, not a silicon-workaround task.**

---

## 2. What `LV_USE_PPA` actually accelerates (verified from source)

`lv_draw_ppa.c` registers three PPA clients (SRM, Fill, Blend) and its `ppa_evaluate` (lines 113–167) bids (preference score 70; the SW draw unit handles everything else) on exactly two task types:

- **Opaque, square, non-gradient FILL** — requires `opa == 255` (`opa <= LV_OPA_MAX → reject`, line 124), `radius == 0`, `grad.dir == NONE` (line 123). Destination format RGB565/RGB888/ARGB8888/XRGB8888 (`ppa_dest_cf_supported`, `lv_draw_ppa_private.h:108`).
- **Opaque, unscaled, unrotated RGB565/RGB888 IMAGE** blit — only when `LV_USE_PPA_IMG` is set; rejects any scale (`scale != 256`), rotation, skew, mask, recolor, or non-normal blend mode (lines 134–155).

It does **not** bid on: text, lines, arcs, gradients, rounded corners, or **layer-composite / blend** tasks. Those stay on the CPU.

Two consequences that shape this plan:

1. **General fps:** PPA offloads opaque fills — the bar rects (spectrum/RF/CSI), panel/screen background clears, button/card bodies. Bounded win: the UI is text/vector-dominated, so the full-screen redraw stays CPU-bound. Tiny bar fills may not beat a SW memset once PPA dispatch overhead is counted; large fills clearly win. **Must measure.**
2. **The CRT overlay is NOT touched by `LV_USE_PPA`.** It is an ARGB8888 `lv_layer_top` composite (`prop_fx.c:181–193`), and the draw unit does not bid on layer blends. Accelerating it requires a **manual `ppa_do_blend`** (Track B).

The hardcoded burst lengths in `lv_draw_ppa.c` are already conservative: SRM=128 (line 62), Fill=128 (line 69), **Blend=32** (line 75). No erratum requires changing them. The framework already invalidates the destination cache before each op (`lv_draw_ppa.c:221`).

---

## 3. Current baseline (already-done levers — do NOT re-do)

The firmware is already well-optimized; the older `communicator-perf` cost-model doc lists several of these as "TODO" but they are implemented:

| Lever | State | Location |
|---|---|---|
| Hybrid LVGL allocator (small→internal SRAM, large→PSRAM) | ✅ done | `lv_port_mem.c:46–53` (≤512 B threshold) |
| LVGL task pinned to core 1; most radio/engine tasks pinned to core 0 | ✅ done | `bsp_illuminate.c:159`; `*PinnedToCore(…,0)` across modules |
| Engine tick uses `xTaskDelayUntil` (no drift) | ✅ done | `prop_engine.c:304` |
| Shadow-compare on scanner wave, labels, BLE row pool | ✅ done | `prop_ui.c` (`s_wave_shadow`, `label_set_text_cached`) |
| RGB565, `swap_bytes=false`, full-screen double buffer in PSRAM | ✅ done | `bsp_illuminate.c:112,173–188` |

---

## 4. Implementation phases

Execute in order. Each phase has a go/no-go gate; do not start the next until the current one is measured.

### Phase 0 — Empirical PPA spike (1 build/flash cycle) — **DO FIRST**

The whole "is PPA broken on *our* board" question is answerable in one cycle and gates everything else.

**Change**
- Add to `sdkconfig.defaults` (so it survives a clean reconfigure):
  ```
  CONFIG_LV_USE_PPA=y
  # CONFIG_LV_USE_PPA_IMG left unset for the first spike (fills only)
  ```
  (Equivalently set via `idf.py menuconfig` → Component config → LVGL → … → "Use Espressif's PPA accelerator".)

**Build / flash**
```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash monitor   # confirm port first (varies COM4/COM7)
```

**Observe (the gate)**
1. Does it boot cleanly (no PPA assert in `lv_draw_ppa_init`, no `HS_MP` mempool loop)?
2. Does the panel render correctly — **no garbage, no torn fills, no color corruption** — on a fill-heavy screen (spectrum bars, a panel with large solid backgrounds)?
3. FPS HUD A/B on the bar screens (see §5).

**Go/No-Go**
- **Clean render + boots** → the "broken silicon" premise is disproven; proceed to Track A.
- **Visible tearing during fills** (no erratum predicts this, but verify) → contingency: lower the Fill client burst at `lv_draw_ppa.c:69` from `PPA_DATA_BURST_LENGTH_128` to `_64`/`_32` (managed-component patch — see §6) and re-test.
- **Boot assert / garbage that won't clear** → capture the exact log, `decode` the PC against `build/communicator.elf`, and stop. That would be a real integration bug worth root-causing before going further.

**Expected gain:** definitive answer + a first fps number. Confidence: this is a measurement, not a guess.

---

### Phase A — Keep `LV_USE_PPA` on for opaque fills/blits (low risk)

If Phase 0 is clean, this *is* the deliverable for "re-enable PPA intentionally." No code beyond the Kconfig flag; the evaluate callback guarantees only the safe opaque-fill subset goes to PPA, so it cannot force a defective op through hardware — it is inherently hybrid with the SW draw unit.

**Decisions**
- **Blocking vs non-blocking** (`LV_PPA_NONBLOCKING_OPS`): start **blocking** (default). Non-blocking spawns a `ppa_thread` (`lv_draw_ppa.c:84`) that waits on the ISR — more parallelism but it competes for core 0 with the radio tasks. Only revisit non-blocking if blocking shows the PPA op serialization is a bottleneck.
- **`LV_USE_PPA_IMG`**: enable only if a measured image-blit screen benefits (the transition noise tile is *scaled*, so it will NOT qualify; a future unscaled RGB565 bitmap, e.g. an ARCHIVE map, would).

**Measure:** bar screens (spectrum/rfband/csi), panel opens with large background fills, with the FPS HUD; A/B against PPA off.

**Risk:** low. Worst case is no gain (tiny-fill dispatch overhead) or marginal tearing (contingency in §6). Revert = flip the Kconfig.

**Expected gain:** modest average-fps lift on fill-heavy screens; little on text-heavy screens. **Confidence: medium — must measure.**

---

### Phase B — Manual `ppa_do_blend` for the CRT overlay (highest value for the stated goal; higher effort)

This is the actual fix for *"the CRT overlay shouldn't crush framerates."* `LV_USE_PPA` does not help here (§2). The overlay (`prop_fx.c`) is a full-screen ARGB8888 layer with per-pixel alpha that LVGL software-alpha-blends over every dirty region every frame.

**Approach:** composite the overlay onto the framebuffer via the PPA **blend** engine (`ppa_do_blend` — `in_bg` = framebuffer RGB565, `in_fg` = overlay ARGB8888, `out` = framebuffer, per-pixel alpha via `fg_alpha_update_mode`) instead of leaving it as a SW-blended `lv_layer_top` canvas. No erratum blocks this on v1.3. PSRAM output buffers must be **64 B-aligned** (`buffer` and `buffer_size`), per `driver/ppa.h`.

> **Honest caveat — blend is memory-bandwidth-bound.** TRM Ch 37 shows BLEND moves **three PSRAM streams through one 2D-DMA** (read bg + read fg + write out), which is why LVGL reports blend shows *no significant gain* over software while fill (one stream) gets up to 9×. So a naive ARGB8888 overlay blend offloads the CPU but **may not raise the overlay-on frame rate**.
>
> **The lever that can make it win — A8 foreground** (TRM Ch 37): the BLEND foreground may be **A8 (alpha-only, 1 B/px) + fixed RGB** (`PPA_BLEND1_RX_R/G/B`). The overlay's scanlines+vignette are a black alpha-mask over one color, so A8 cuts foreground traffic 4 B→1 B/px (~8→5 B/px total, ~37% less). Plan: bake the amber wash into the background (flat tint, free); feed scanlines+vignette as an **A8 mask, fixed RGB=black** (one cheap pass); keep the amber refresh band as a small separate ARGB pass. With A8 the bandwidth math may actually favor the PPA — without it, it likely won't. Still a spike: measure. Note Phase C2 (shrink dirty area) may do as much for overlay-on fps and is far cheaper.

**Why it is more involved**
- It intersects the LVGL flush / compositing timing — the blend must land after LVGL renders a frame and before the panel DMA reads it.
- Cache sync: the foreground (overlay) and destination (framebuffer) are in PSRAM; must `esp_cache_msync` correctly around the PPA DMA.
- Must respect the existing hard constraint: **do not run the draw pipeline / canvas layer-draw under the LVGL lock from a non-LVGL task** (that deadlock is why `prop_fx` paints pixels directly today and `/screenshot` reads the FB).
- `/screenshot` currently reads the DPI FB and skips `layer_top`, so today the overlay must be judged on the physical panel. **If we blend into the FB, `/screenshot` would start capturing the overlay** — a verification bonus.

**Sub-steps**
1. Spike: a standalone `ppa_do_blend` of the baked overlay onto a test framebuffer region; inspect on the physical panel for correctness (alpha, color, no tearing). Measure fps vs. the SW overlay — if no gain, try the A8 path before investing further.
2. Rework the overlay for A8: bake the amber wash into the screen background; store scanlines+vignette as an **A8 alpha mask** (1 B/px) with fixed RGB=black; keep the amber refresh band as a small separate ARGB pass.
3. Integrate into the flush path: register a PPA blend client in `prop_fx`, blend the A8 mask onto the active draw buffer at frame-completion, drop the `lv_layer_top` SW composite. Keep the static-vs-animated split (`prop_fx.c` design).

**Risk:** medium — touches flush timing and cache coherency. Gate on the Phase-0/Track-A result and on its own hardware spike.

**Expected gain:** offloads the per-frame full-screen alpha blend from the CPU; whether that *raises fps* is uncertain (bandwidth-bound — see caveat). **Confidence: low–medium — the spike decides it. Do Phase C2 first; it may help overlay-on fps more.**

### Phase B-alt — 2D-DMA on the DPI panel (cheap experiment)

Independent of the PPA blend: `esp_lcd_dpi_panel_enable_dma2d()` lets the MIPI-DSI panel use the 2D-DMA engine for its framebuffer transfers, offloading copy work from the CPU. This is a small, bounded experiment (one API call at panel init in `bsp_illuminate.c`) worth A/B-ing for a general CPU-time saving on the flush path. Low risk, measure with the HUD.

---

### Phase C — Complementary non-PPA wins (independent; compound with A/B)

These stand on their own and reduce the work both the CPU and the PPA must do.

**C1 — Pin the last unpinned tasks to core 0 (consistency, near-zero risk).** Plain `xTaskCreate` = no affinity, so the scheduler may place them on core 1 (LVGL's core) mid-render and stall a frame:

| Task | Prio / rate | Location |
|---|---|---|
| `imu_dmp` | 5, every 40 ms ← worst offender | `prop_imu.c:239` |
| `prop_radar` | 5 | `prop_radar.c:199` |
| `csi_push` | 4 | `prop_coproc.c:128` |
| `prop_calib` | 4 | `prop_calib.c:145` |
| `prop_traffic` | 3 | `prop_traffic.c:90` |

Change each `xTaskCreate(...)` → `xTaskCreatePinnedToCore(..., 0)`. Verify placement by logging `xTaskGetCoreID(NULL)` at task entry during one run.

**C2 — Remove per-frame `lv_obj_align` + blind restyle on the bar screens (throughput).** Spectrum (`prop_ui.c:4368–4372`), RF band (`:4390–4394`), CSI (`:4446–4450`) call `set_height` + `lv_obj_align` + `set_bg_color` for every bar every frame. The bar **X never moves** → set position once at build, drop the per-frame `align` (it forces re-layout + invalidation). Shadow-compare the color; only `set_bg_color` on a threshold-band change. Keep `set_height` (tracks decay), optionally skip when unchanged at rest.

**Expected gain:** C1 removes sporadic hitches (consistency, priority-1); C2 cuts spectrum from ~96 LVGL calls/frame to ~32 height-sets + rare color/align and shrinks dirty area. **Confidence: high it helps; magnitude needs the HUD.**

---

## 5. Measurement protocol (A/B every change with the on-device HUD)

```bash
python tools/prop.py state                                                   # confirm comm-unit-7.local reachable
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","fps":true}'    # FPS HUD on  (separate from the CRT overlay)
# capture each heavy screen
python tools/prop.py shot spectrum.png --screen spectrum --wait
python tools/prop.py shot csi.png      --screen csi      --wait
python tools/prop.py shot rfband.png   --screen rfband   --wait
# CRT overlay A/B (Phase B target) — toggle the overlay, watch the HUD on the same screen
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":false}'    # CRT overlay OFF
curl -s -X POST http://comm-unit-7.local/cmd -d '{"cmd":"fx","on":true}'     # CRT overlay ON
```

Record per screen: **steady-state FPS** and **frame-time consistency** (watch the HUD ~10 s on a moving screen — a tightening swing matters more than +2 avg fps). For Phase 0/A, also note any visual artifact (tearing, color corruption) on the physical panel — `/screenshot` will catch content artifacts but not the `lv_layer_top` overlay until Phase B blends into the FB.

---

## 6. Contingencies & rollback

- **Tearing during PPA fills (Phase 0/A):** patch the Fill client burst at `lv_draw_ppa.c:69` (`PPA_DATA_BURST_LENGTH_128` → `_64` or `_32`). This edits a managed component — pin the version and document the patch (e.g. a component-override under `components/` or a tracked patch), since `idf.py update-dependencies` would otherwise overwrite it.
- **Internal-RAM pressure from anything:** the sanctioned lever is `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` — never shrink internal reserves (esp_hosted SDIO mempool needs them; ~332 KB internal free is the budget).
- **Full rollback:** Phase 0/A revert = remove `CONFIG_LV_USE_PPA=y`, reconfigure. Phase B revert = restore the `lv_layer_top` overlay path. Phase C is independent and individually revertable.

## 7. Hard constraints (a change that breaks one is a regression)

- Don't restore the LVGL builtin pool / raise `LV_MEM` (→ `HS_MP` mempool boot loop).
- Don't touch `CONFIG_ESP32P4_*REV*` (board is rev v1.3).
- Keep `swap_bytes=false` (native RGB565; `true` → magenta background).
- Never call WiFi/SDIO under the LVGL lock; never run canvas layer-draw / `lv_snapshot` from a non-LVGL task under the lock (deadlock).

## 8. Sequencing summary

1. **Phase 0** — flip `CONFIG_LV_USE_PPA=y`, build/flash, inspect + HUD. (1 cycle, disproves or confirms the "broken" premise.)
2. **Phase C1** — pin 5 tasks (cheap consistency win, parallel to anything).
3. **Phase A** — keep PPA on for fills; measure fill-heavy screens.
4. **Phase C2** — bar-screen churn cleanup; re-measure (compounds with A; this is also the most likely *overlay-on* fps win).
5. **Phase B-alt** — `esp_lcd_dpi_panel_enable_dma2d()` one-liner; A/B the flush-path CPU saving.
6. **Phase B** — manual `ppa_do_blend` for the CRT overlay; spike first. Lower priority than it first appeared — the blend is bandwidth-bound, so it may free CPU without raising fps. Only integrate if the spike shows a real fps gain.

## 9. References

- ESP32-P4 chip errata: https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32p4/03-errata-description/index.html (no PPA/DSI/L2/2D-DMA erratum; MSPI-750/751 are v3.0-only)
- ESP32-P4 v3.x chip-revision user guide: https://documentation.espressif.com/esp32-p4-chip-revision-v3.x_user_guide_en.html
- LVGL v9 PPA draw unit: `managed_components/lvgl__lvgl/src/draw/espressif/ppa/lv_draw_ppa.c`, auto-init at `managed_components/lvgl__lvgl/src/lv_init.c:272`
- LVGL PPA docs: `managed_components/lvgl__lvgl/docs/src/details/integration/chip_vendors/espressif/hardware_accelerator_ppa.rst` (notes: ~30% avg / up to 9× on fills; **blend bandwidth-bound, no significant gain**)
- IDF PPA driver API: `C:/esp/v6.0.1/esp-idf/components/esp_driver_ppa/include/driver/ppa.h` (`ppa_do_fill/blend/scale_rotate_mirror`; output buffer needs 64 B / L2-line alignment; `data_burst_length` per-client)
- 2D-DMA on DPI panel: `esp_lcd_dpi_panel_enable_dma2d()` (IDF `esp_lcd`)
- TRM chapters (broken out locally): `datasheets/ESP32-P4/technicalReference/37 Pixel-Processing Accelerator (PPA).pdf` (BLEND = 3 PSRAM streams → bandwidth-bound; A8 fg = 1 B/px + fixed RGB; source-over formula; 32×32 default block), `6 2D-DMA Controller (2D-DMA).pdf`, `4 GDMA Controller (GDMA-AHB, GDMA-AXI).pdf`, `35 JPEG Codec.pdf`
- Prior spike (superseded): `docs/superpowers/specs/2026-06-20-ppa-acceleration-research.md`
- PPA reference (errata-accurate): `docs/ESP32-P4 v1.3 PPA Optimization & Debugging Guide.md`
- Perf skill + cost model: `.claude/skills/communicator-perf/`
