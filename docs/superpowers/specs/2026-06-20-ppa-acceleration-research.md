# PPA / 2D-DMA Acceleration — Research Spike

**Date:** 2026-06-20
**Status:** Research finding (no implementation) — decision needed
**Question:** Can the ESP32-P4 PPA (Pixel Processing Accelerator) get the 1024×600
panel to a sustained 60fps, as wanted for the FPS-meter "performance mode"?

## What exists today

- **IDF 6.0.1 ships the PPA driver** (`driver/ppa.h`). It exposes exactly three
  operations:
  - `ppa_do_fill` — solid-color rectangle fill
  - `ppa_do_blend` — alpha-blend two layers (incl. A8/A4 foreground)
  - `ppa_do_scale_rotate_mirror` (SRM) — scale / rotate / mirror a buffer
- **esp_lvgl_port already uses PPA — but only for screen rotation**
  (`src/common/ppa/lcd_ppa.c` → `lvgl_port_ppa_rotate`, gated by
  `CONFIG_LVGL_PORT_ENABLE_PPA` = "Enable PPA for screen rotation", and the
  `sw_rotate` display-config flag). There is **no general draw offload**.
- **LVGL 8.4 has GPU draw backends** for ARM-2D, NXP PXP/VGLite, STM32 DMA2D —
  **but none for ESP32-P4 PPA**. (`src/draw/arm2d`, `src/draw/nxp`, `src/draw/sw`
  only.) So there is no turnkey PPA draw context to switch on.

## Why PPA does NOT reach 60fps for this UI

1. **PPA accelerates fills, blends, and blits — not text, lines, arcs, or
   gradients.** The communicator UI is overwhelmingly **text (Eurostile labels)
   and vector primitives** (the rail glyphs, waveform line, gauges, borders).
   That work stays on the CPU regardless of PPA. A full-screen redraw is slow
   (~250ms ≈ 4fps measured) because of the *object/text count*, which PPA cannot
   touch.
2. **The flush is already DMA.** MIPI-DSI output is hardware-driven; copying the
   rendered framebuffer to the panel is not the bottleneck — CPU rasterization is.
3. **Using PPA for general drawing requires a custom LVGL v8 draw context**
   (mirroring the PXP/DMA2D backends) that routes fill/blend/blit to PPA. Even
   done well, it only offloads the minority of our draw ops, so the full-screen
   render stays CPU-bound on text/lines. **Low ROI for a 60fps full-screen goal.**
4. The cleaner long-term route — **LVGL v9's draw-unit architecture + a PPA draw
   unit** — is a **major migration** (v8→v9 API touches all of `prop_ui.c`, fonts,
   styles) and still leaves text on the CPU.

**Conclusion:** PPA is the wrong lever for "60fps full-screen" on a text/vector
UI. Full-screen 60fps is not attainable here by acceleration; it isn't attainable
at all with CPU rasterization at this resolution, and PPA doesn't change that.

## Where PPA *would* pay off (real, bounded wins)

- **Transition effects** (`prop_fx` snow/roll/collapse): these are big fills and
  image blits of the noise tile — exactly PPA's `fill`/`blend`/`SRM`. PPA could
  make transitions cheap and let them cover the full screen smoothly.
- **Large background fills** and any future **image-heavy screens** (e.g. the
  desert map bitmap in the ARCHIVE) — `ppa_do_blend`/SRM for fast scaled blits.
- **Screen rotation** — already supported via `sw_rotate` if ever needed.

## Recommendation

"Never below 60fps" is achievable for **smoothness of motion**, not full-screen
redraws: keep renders partial and raise the **animation source rate** (engine
tick 10Hz → ~60Hz) so the things that move update 60×/s and the cheap partial
renders keep up. Pursue PPA **only** as a targeted accelerator for the
image/fill-heavy transition effects, not as a path to global 60fps.

### Options

1. **Engine-rate for smooth 60fps motion** (recommended) — raise the engine/anim
   tick toward 60Hz; verify responsiveness + no WDT. Directly improves perceived
   framerate; no new architecture.
2. **PPA for transitions only** — register a PPA client in `prop_fx`, do the
   snow/collapse via `ppa_do_fill`/`ppa_do_blend`. Bounded, concrete win for the
   channel-change effects; does not affect general UI fps.
3. **LVGL v9 + PPA draw unit** — large migration; broad but expensive, text still
   CPU-bound. Only worth it as a separate major project.

## Verification notes

- Full-screen redraw measured at ≈4fps (FPS meter showed `FPS 4` when forcing a
  full `lv_obj_invalidate(lv_scr_act())` every frame).
- Engine-driven animation (scanner waveform, ~10Hz) measured ≈14fps with the
  refresh period already lowered to 8ms — confirming the limiter is content-change
  rate, not the refresh cap.
- esp_lvgl_port task enforces a ≥5ms `vTaskDelay` per loop, so higher refresh
  rates cannot starve the task watchdog (the earlier crash was a single
  >5s `lv_timer_handler`, caused by a translucent `lv_layer_top` object).
