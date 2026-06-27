# ESP32-P4 Silicon v1.3: PPA, DSI, and Cache — Accurate Reference

This document describes how to run a 1024×600 UI on **ESP32-P4 chip rev v1.3** with the
Pixel Processing Accelerator (PPA), based on Espressif's published errata and the LVGL 9.4
PPA draw unit. It replaces an earlier draft that was premised on v1.3 silicon defects which
the official errata do **not** document.

For the staged implementation plan that uses this reference, see
`docs/superpowers/specs/2026-06-27-ppa-reenablement-framerate-plan.md`.

## 0. What is actually wrong (and not wrong) on v1.3

Per the official ESP32-P4 chip errata
(<https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32p4/03-errata-description/index.html>),
the documented errata for **v1.3** are RMT-176 (RMT idle level), I2C-308 (I2C slave
multi-read), and APM-560 (an unauthorized-AHB access can stall PSRAM/flash — triggers only on
a permission violation, not normal rendering). **None of these touch the PPA, MIPI-DSI, L2
cache, or 2D-DMA.**

Key points to design around:

- **There is no documented PPA logic defect on v1.3.** The fill, blend, and SRM engines work
  on RGB565/RGB888/ARGB8888 as specified. Earlier trouble with PPA on this project was a
  software/timing issue: it was first evaluated under LVGL 8.4, which had **no ESP32-P4 PPA
  draw backend at all**. LVGL 9.4 (current) ships one.
- **The PSRAM unaligned-DMA-read bug (MSPI-750) and the overlap-timing bug (MSPI-751) affect
  v3.0 only and are fixed in v3.1.** They do **not** affect v1.3, so the "stale PSRAM reads /
  corrupted vertical strips" failure mode does not apply to this board.
- **What v3.x changed for the PPA** — 32×32 block processing and YUV422/420 support — is a
  feature addition for video fluidity, **not** a v1.3 defect repair. A text/vector UI uses
  neither; v1.x's RGB565 fill/blend is sufficient.

### Configuration that is already correct on this board

- **L2 cache line is 64 B** (`CONFIG_CACHE_L2_CACHE_LINE_64B=y`). No 128-vs-64 alignment
  conflict exists here.
- **The framebuffer/draw path is RGB565, `swap_bytes=false`** (`bsp_illuminate.c`). This is the
  stable color path; do not switch the framebuffer to RGB888/ARGB8888.
- **LVGL heap is routed to PSRAM with a hybrid allocator** (`lv_port_mem.c`): small transient
  draw-mask allocations go to internal SRAM, large buffers to PSRAM.

## 1. Enabling the PPA (LVGL 9.4)

The v9 PPA draw unit lives in
`managed_components/lvgl__lvgl/src/draw/espressif/ppa/lv_draw_ppa*.c` and is auto-registered
from `lv_init.c` (`lv_draw_ppa_init()`) when `LV_USE_PPA` is set. To enable:

```
# sdkconfig.defaults
CONFIG_LV_USE_PPA=y
# CONFIG_LV_USE_PPA_IMG=y   # optional: opaque unscaled RGB565/888 image blits
```

Then `idf.py reconfigure build flash`.

### What the PPA draw unit accelerates

`ppa_evaluate` (in `lv_draw_ppa.c`) bids on exactly two task types; the software draw unit
handles everything else (text, lines, arcs, gradients, rounded corners, and **layer
composites**):

- **Opaque, square, non-gradient fills** — `opa == 255`, `radius == 0`, no gradient.
  Destination RGB565/RGB888/ARGB8888/XRGB8888.
- **Opaque, unscaled, unrotated RGB565/RGB888 image blits** — only with `LV_USE_PPA_IMG`;
  rejects any scale, rotation, skew, mask, recolor, or non-normal blend mode.

Because the evaluator only claims this safe subset, the PPA draw unit is inherently a *hybrid*
with the software renderer — it cannot route an unsupported op through the hardware.

### Burst lengths and cache handling

Burst lengths are set in `lv_draw_ppa.c` at client registration: SRM = 128, Fill = 128,
**Blend = 32**. These are conservative and need no change for v1.3. The draw unit already
invalidates the destination cache before each op (`lv_draw_buf_invalidate_cache`), so no
manual `esp_cache_msync` is required for the LVGL-managed path.

`data_burst_length` is a **per-client** config field (`ppa_client_config_t`, default
`PPA_DATA_BURST_LENGTH_128`). Lowering it reduces PPA throughput but frees AXI burst bandwidth
for other masters (e.g. the MIPI-DSI readout) — this is the real lever if PPA activity ever
contends with the display, and it is set in code at client registration, **not** via any
Kconfig symbol.

### Buffer alignment requirement (authoritative)

Per `driver/ppa.h` (`ppa_out_pic_blk_config_t`), the PPA **output** buffer and its
`buffer_size` must be aligned to the cache line: **internal memory → L1 line; external (PSRAM)
memory → L1 *and* L2 line**. On this board the L2 line is 64 B, so PSRAM output buffers
(framebuffers) align to **64 B**. This is the genuine alignment constraint — 64 B, matching our
config. Use `heap_caps_aligned_alloc(64, size_aligned_to_64, MALLOC_CAP_SPIRAM)` for any
manual PPA destination.

## 2. Accelerating the CRT overlay (manual blend)

`LV_USE_PPA` does **not** accelerate the CRT overlay (`prop_fx.c`): the overlay is a full-screen
ARGB8888 `lv_layer_top` composite, and the draw unit does not bid on layer-blend tasks. To
offload it, composite the overlay onto the framebuffer with the PPA **blend** engine
(`ppa_do_blend` — `in_bg` = framebuffer RGB565, `in_fg` = overlay ARGB8888, `out` = framebuffer;
per-pixel alpha via `fg_alpha_update_mode`). No erratum blocks this on v1.3.

**Honest caveat — blend is memory-bandwidth-bound.** The TRM (Ch 37) shows BLEND moves **three
PSRAM streams through one 2D-DMA**: read background + read foreground + write output. That is why
the LVGL PPA docs report blend shows *no significant gain* over software, while fill (one stream)
gets ~30% avg and up to 9×. PSRAM bandwidth is the wall whether the CPU or PPA does the blend.

**The lever that can make it win — A8 foreground.** Per TRM Ch 37, the BLEND foreground may be
**A8 (alpha-only, 1 byte/pixel) with a fixed RGB color** (`PPA_BLEND1_RX_R/G/B`). The CRT overlay's
scanlines + vignette are essentially a **black alpha-mask over one color**, so feeding them as A8
instead of ARGB8888 cuts foreground traffic from 4 B/px to 1 B/px (≈8→5 B/px total, ~37 % less).
The blend math is standard source-over: `Aout = Ab + Af − Ab·Af`, `Cout = (Cb·Ab·(1−Af) + Cf·Af)/Aout`.
Practical plan for the overlay:
- Bake the uniform amber **phosphor wash** into the screen background (it's a flat tint — free).
- Feed **scanlines + vignette as an A8 mask** with fixed RGB = black → one cheap blend pass.
- The amber refresh band, being a different color, stays a small separate ARGB pass (it's already
  a thin bounded stripe).

This keeps Track B a **spike** (measure first), but A8 is the difference between "offloads CPU,
no fps gain" and a real win, given the bandwidth bound.

When doing a manual `ppa_do_blend`:

- Both foreground (overlay canvas) and destination (framebuffer) are in PSRAM. Synchronize the
  cache around the PPA DMA: write back the source/destination before the op and invalidate the
  destination after, so the panel's DMA reads coherent pixels.
- Sequence the blend after LVGL renders a frame and before the panel reads it.
- Respect the existing constraint: do not run canvas layer-draw / `lv_snapshot` from a non-LVGL
  task under the LVGL lock (that path deadlocks — this is why `prop_fx` paints pixels directly
  today).

See Phase B of the implementation plan for the staged approach (spike → integrate).

## 3. Serial-monitor debugging

Enable verbose logging while bringing up the PPA:

```
CONFIG_LOG_DEFAULT_LEVEL_VERBOSE=y
CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE=y
```

| Observed | Likely cause | Action |
| :--- | :--- | :--- |
| `ppa: ... not aligned` / transaction aborted | a manual `ppa_do_*` call passed an unaligned buffer/stride | align the buffer to 64 B and ensure stride matches the PPA's expectation; the LVGL draw unit handles this for managed ops |
| Visible tearing during PPA fills | PPA bursts contending with the DSI readout (not an erratum — measure if it occurs) | lower the Fill client burst in `lv_draw_ppa.c` (`PPA_DATA_BURST_LENGTH_128` → `_64`/`_32`); this edits a managed component, so pin and track the patch |
| Corrupted pixels with no log error | cache incoherency around a manual PPA DMA | add the `esp_cache_msync` write-back/invalidate around the op (§2) |

### Stress test

Drag an overlay panel continuously across the 1024×600 plane for ~5 minutes while watching the
serial log. A clean run (no GDMA/cache errors) plus a clean physical-panel image confirms the
PPA integration is stable.

## 4. Other hardware acceleration on this SoC

Beyond the PPA, the ESP32-P4 has several DMA/accelerator blocks. Ranked by relevance to *this*
text/vector communicator UI:

| Block | What it offloads | Relevance here |
| :--- | :--- | :--- |
| **PPA fill** | opaque RGB565 rectangle fills | **High** — bar rects, panel/background clears. The clearest PPA win (~30% avg, up to 9× on integer-multiple sizes). Enabled by `LV_USE_PPA`. |
| **PPA blend** | ARGB8888-over-RGB565 layer composite (CRT overlay) | **Medium/uncertain** — bandwidth-bound (§2); offloads CPU but may not raise fps. Manual `ppa_do_blend`. |
| **2D-DMA** | strided block memory moves (the engine the PPA sits on) | **Medium** — `esp_lcd_dpi_panel_enable_dma2d()` lets the DPI panel use 2D-DMA for its framebuffer transfers, freeing the CPU from copy work. Worth A/B-ing on the flush path. |
| **PPA SRM** | scale / rotate / mirror blits | **Low/situational** — only if a screen scales an image (e.g. an ARCHIVE map zoom). The transition noise tile is scaled, so SRM *could* accelerate it; today it is a CPU image blit. |
| **GDMA (mem-to-mem)** | async large `memcpy` (asset loads, buffer moves) | **Low** — not per-frame; useful for moving large assets without stalling the CPU. |
| **JPEG codec** | hardware JPEG decode/encode | **Low** — only if assets are stored as JPEG; decode is one-shot at load, not per-frame, so it affects load time, not framerate. |
| **H.264 encoder, ISP, MIPI-CSI** | video encode / camera pipeline | **None** for this UI (no camera/video path in the prop today). |

For the framerate goal the actionable set is **PPA fill** (clear win), **2D-DMA on the DPI panel**
(`esp_lcd_dpi_panel_enable_dma2d`, frees the CPU from framebuffer copies), and **PPA blend for the
overlay** (spike — uncertain). The rest are situational or load-time only.
