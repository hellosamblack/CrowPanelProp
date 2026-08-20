# LIDAR thin-client: motion-compensated video delivery — feasibility spike

**Date:** 2026-08-19
**Status:** Proposed — research spike only. No implementation, no protocol changes yet.
**Question:** Can `/ws-thin` frames be delivered as inter-frame-coded H.264 instead of
independent JPEGs, to trade the rig's spare render/encode compute for less bandwidth on
the panel's constrained SDIO link — and does the CrowPanel's ESP32-P4 have anywhere near
the decode budget to make that worthwhile?
**Related:** `2026-08-18-lidar-thin-frame-bandwidth-protocol.md` (the shipped v2 JPEG +
credit-flow protocol this would supersede) and its **"Rejected alternatives → Delta/tile
updates instead of JPEG"** entry — this proposal is best understood as a retry of that
idea using a real motion-compensated codec instead of naive pixel/tile diffing,
specifically because motion compensation does not share tile-diff's failure mode under
camera orbit (see below). Do not approve this spec without reading that rejection first.

## Baseline to beat

Current shipped state (`2026-08-18-...md` §Measured, 10 min hardware soak):

| | v2 JPEG (current) |
|---|---|
| Encoding | Independent baseline JPEG per frame, 4:2:0, quality negotiated (`lidar_q`) |
| Resolution | 480×480 |
| Frame size | ~9,100 B (q75) |
| Sustained delivered rate | **6.52 fps** |
| Bytes/s to panel | ~57 KB/s = 456 kbit/s (link budget ~1.5 Mbit/s — lots of headroom left on paper) |
| Frame independence | Every frame is a full self-contained image — the server can drop any queued frame and just send the newest one; the client can lose a frame with zero decode-state consequence |

That last row is the fact this whole spike has to reckon with — see §The flow-control
problem below. Note the link is *not* saturated at 456 kbit/s of a ~1.5 Mbit/s budget;
the fps ceiling here is the ~100 ms poll/credit-pump cadence and per-frame JPEG decode
path, not raw bandwidth. That changes the ROI calculus: video coding would mostly buy
*headroom*, not fix an active bottleneck.

## The proposal

Replace (or offer as a third `tag`) independent JPEG frames with an H.264 (or similar
motion-compensated) bitstream: encode on the rig, decode on the panel. The intuition —
"most of the time it's a few pixels moving a little" — is correct, and unlike naive
tile-diffing, motion-compensated inter-frame coding is *built* for a moving/orbiting
camera: each block is predicted via a motion vector rather than assumed static, so
`thin_orbit` drags (`main/prop_lidar.c:299-326`, spec `2026-08-17-...md:123-128`) are a
well-handled case, not a pathological one the way they were for tile-diff. A target
bitrate is also decoupled from resolution×fps the way raw/JPEG isn't, so "0.6 Mbit/s at
30 fps" is a real, achievable encoder setting in principle.

## Hardware reality check: the P4's H.264 block is encode-only

Verified against the ESP-IDF's own example, the only H.264 support that ships for this
chip (`~/.local/esp/esp-idf/examples/peripherals/h264/`):

```c
#if CONFIG_H264_ENCODER_HARDWARE
#include "esp_h264_enc_single_hw.h"
#else
#include "esp_h264_enc_single_sw.h"
#endif
#include "esp_h264_dec_sw.h"        /* no hardware branch exists, for either target */
```

README, verbatim: *"Encode video frames using H.264 codec (hardware on ESP32-P4, software
on ESP32-S3) — Decode the encoded frames back to original format using **software**
decoder."* There is no `esp_h264_dec_hw` anywhere in `espressif/esp_h264` (pinned
`^1.0.4` in the example's `idf_component.yml`; not currently a dependency of this project
— checked `main/idf_component.yml`). The P4's hardware acceleration targets
camera→network encode (video doorbell / conferencing use cases), the opposite direction
from this system.

**Consequence: the CrowPanel is the receiver here, so this is entirely a software-decode
feasibility question.** The rig-side encoder is a non-issue — any Linux box runs libx264
comfortably, hardware or not; the rig already renders faster than it can currently send
(measured 29.9 fps internal render vs 6.52 fps delivered, per the v2 spec's telemetry
example). All the risk is on the P4 finding spare cycles for `tinyh264`-class software
decode (motion compensation + deblocking + CABAC/CAVLC entropy decode) at 480×480, on a
core that is already running the LVGL flush loop, `esp_hosted` WiFi/BT, and the sensor
task set — this is categorically heavier per pixel than the JPEG path it would replace,
which offloads entirely to the P4's dedicated `esp_driver_jpeg` hardware block and costs
the CPU almost nothing (`prop_lidar.c` decodes tag-2 JPEG straight into the PSRAM triple
buffer with no software decode loop at all).

## The flow-control problem (the part that isn't just a CPU question)

This is the objection that matters most, independent of whether decode turns out to be
cheap enough — it's a structural mismatch with the protocol architecture that made v2
work at all.

The v2 credit model (`2026-08-18-...md` §1) exists because JPEG frames are independent:
*"On render completion with zero credit, the server discards that frame for this client.
It never buffers it. When credit arrives it sends the newest frame it has, never a
backlog entry."* That is only safe because dropping frame N has no effect on the
client's ability to decode frame N+1 — there is no reference chain. The 10 min soak
measured `dropped: 1287` alongside a healthy `tx_fps` and called that *"normal and
correct"* (`2026-08-18-...md:143-144`).

Inter-frame video coding breaks that assumption outright. A P-frame is only decodable if
the client already has the exact reference frame the encoder predicted it against; a
B-frame needs frames on both sides. If the server (or the credit/backpressure logic)
discards a P-frame the way it discards stale JPEGs today, every subsequent frame that
references it decodes into corrupted macroblocks until the next I-frame — visible,
ugly artifacts, not a graceful skip. WebSocket-over-TCP does guarantee in-order,
lossless *delivery* (no network-level packet loss reaches the app layer the way it would
over RTP/UDP), which removes one class of desync risk — but it does nothing about the
server *deliberately* dropping a frame under backpressure, which is exactly the
mechanism v2 relies on to survive a link this constrained, and exactly what a client
with only 2 credits deep will keep needing.

Two ways to reconcile this, both real machinery beyond "add a decoder":

1. **Encoder-side frame selection driven by client credit**, i.e. the rig never encodes
   a P-frame referencing a base the client hasn't confirmed decoding — the credit signal
   has to reach into the *encoder loop*, not just gate transmission. This is roughly what
   adaptive real-time video (WebRTC simulcast/SVC, temporal layering) does, and it is
   substantially more machinery than "JPEG whatever's newest."
2. **Forced I-frame on every credit-starved gap / reconnect**, which is closer to what
   this link actually needs (SEARCHING backoff already exists, `prop_lidar.c`) but
   spends exactly the bytes a keyframe costs at exactly the moments bandwidth is worst —
   working against the reason video coding was attractive in the first place.

Either path needs designing and testing before a single decode-cost number matters. This
is called out explicitly because it's easy to greenlight a decode microbenchmark, see a
good CPU number, and only discover the reference-frame problem after wiring the whole
protocol — the decode benchmark below is deliberately scoped to not require solving this
yet, and the recommendation does not skip past it.

## Open questions (in priority order — each is a potential kill criterion)

1. **Software H.264 decode cost on the P4, under realistic load.** CPU %, wall-clock
   decode latency, and peak RAM for `tinyh264` (or equivalent) at 480×480 (match the
   current raster, not the 640×480 floated in discussion — apples-to-apples against the
   JPEG baseline), across a small fps/bitrate grid, **with LVGL's 20 Hz observer tick,
   `esp_hosted`, and the sensor tasks running at their normal load** — not an idle bench.
   No public number exists for this chip at this resolution; the encode-side numbers in
   the ESP-IDF example (320×240 @ 30 fps hardware-encode) say nothing about software
   decode cost.
2. **The flow-control redesign above** — is encoder-side credit-aware frame selection
   feasible against this rig's render pipeline, or does every credit-starved moment force
   a keyframe expensive enough to erase the bandwidth win?
3. **GOP/keyframe strategy for reconnect.** Every `SEARCHING` recovery needs a fresh
   I-frame; on a link this marginal, how often does that actually happen in practice
   (worth pulling from the existing `link_fps`/staleness telemetry), and what does it cost
   in the worst case (a flappy link forcing frequent I-frames is worse than steady JPEG).
4. **Wire framing.** Annex-B NAL units over the existing WS transport — a new `tag = 3`
   alongside the existing `tag = 1` (raw) / `tag = 2` (JPEG), with the same
   unfragmented-WS-message contract v1/v2 already require. Not hard, but not free either
   (NAL unit boundaries, SPS/PPS delivery on connect or per-keyframe).
5. **Is the rig's own encode path actually a non-issue?** Assumed yes (libx264 on
   whatever the rig runs on is trivial), but not verified against the rig's actual
   render/encode loop and its own real-time budget — the rig currently renders at ~30 fps
   *un*-encoded; adding real-time H.264 encode into that loop is a rig-side (separate
   repo, `submodules/lidar-roomscanner`, not checked out in this workspace) change this
   spec does not scope.

## Recommended next step — decode-only microbenchmark, nothing else

Do **not** start on protocol/wire changes yet. The single highest-value, lowest-cost
action is a standalone test app that answers open question 1 in isolation:

1. Add `espressif/esp_h264` (`^1.0.4`) to a throwaway test target (not `main/`).
2. Feed it a representative motion-compensated 480×480 stream (can be synthetic — the
   IDF example's pattern generator is a fine starting point, but a real point-cloud/SLAM
   capture from the rig would be more representative of the "flat black, sparse moving
   points, occasional orbit" content this actually needs to handle).
3. Measure decode CPU%, latency, and RSS **while the rest of the firmware's normal task
   set is running** (LVGL @ 20 Hz, `esp_hosted`, IMU/radar polling) — a bench with nothing
   else running will systematically overstate available headroom.
4. Kill criterion: if sustained decode at a usable fps/resolution costs more than a small
   fraction of a core, or meaningfully starves the LVGL flush loop (frame-time regression
   on any other panel), stop here — the JPEG+hardware-decode path stays as-is, and the
   answer to this spec's Question is "no, not on this chip."
5. If it clears that bar, only *then* is it worth scoping the flow-control redesign
   (open question 2) as its own spec — it's the more expensive design problem and
   shouldn't be started until decode is known to be affordable at all.

## Non-goals for this spike

- No changes to `main/prop_lidar.c`, `main/prop_ui.c`, or the `/ws-thin` wire protocol.
- No rig-side (`lidar-roomscanner`) changes or scoping — this spec covers only the
  client-decode feasibility question, which gates everything else.
- Not re-litigating plain tile/delta diffing — that was correctly rejected in
  `2026-08-18-...md` for reasons (orbit, compositor machinery) that don't apply here, but
  also don't need re-arguing here.

## Rejected framing

- **"Just try it end-to-end and see."** Skips both the reference-frame flow-control
  problem and the decode-cost unknown at once — if it fails, there's no way to tell which
  of the two killed it, and the flow-control redesign (open question 2) is expensive
  enough that it shouldn't be built speculatively before question 1 has an answer.
- **640×480 as floated in discussion.** The rig currently negotiates and renders 480×480
  (`thin_hello`/`thin_hello_ack`, `2026-08-18-...md` §Handshake); benchmarking a
  different resolution than the JPEG baseline uses would make the comparison apples-to-
  oranges. 480×480 is the correct benchmark target; a resolution *reduction* is already
  an available fallback lever in the existing handshake (`2026-08-18-...md`, "Rejected
  alternatives → Client-side downscale request only") independent of codec choice.
