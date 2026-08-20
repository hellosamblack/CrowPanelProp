# LiDAR thin-client `/ws-thin` v2 — flow control + compressed frames

**Date:** 2026-08-18
**Status:** implemented both sides 2026-08-19. Server: `lidar-roomscanner` issue #202 —
seq'd 16-byte tag-2 header, baseline 4:2:0 JPEG, credit flow control with `thin_ready`,
per-client `tx_fps`/`tx_bytes_per_s`/`dropped` telemetry. Client: `main/prop_lidar.c` —
`thin_hello`/`thin_hello_ack`, `thin_ready` on UI consumption, `esp_driver_jpeg` hardware
decode of tag 2. **All four acceptance bars pass on hardware** (see §Measured): 0.4 -> 6.52
fps, zero SEARCHING over 10 min, ping 892 -> 39.8 ms, and both compatibility directions
verified. Closing the latency bar also required gating `prop_ftm`'s background WiFi scan,
which was an unrelated pre-existing tax on the shared C6 radio.
**Supersedes parts of:** `2026-08-17-lidar-thin-client-crowpanel-design.md` ("Protocol
contract", and the "Resolution/rate negotiation" + "JPEG + P4 hardware decode" open questions)

## Problem

The v1 protocol streams uncompressed 480x480 RGB565 at a fixed cadence with no flow
control. On a wired client that works fine. On the CrowPanel it does not, because the
CrowPanel's link is roughly **20-40x too small for the stream**.

Measured 2026-08-18, rig at `roomscan.local:8000`, panel at `comm-unit-7.local`:

| Path | Result |
|---|---|
| Rig -> wired host (`/ws-thin`) | **10.05 fps** x 460,808 B = **37.1 Mbit/s** |
| Rig -> CrowPanel, sustained | **0.37-0.51 fps** |
| CrowPanel link capacity (`/screenshot`, 1.23 MB) | **0.96 / 1.03 / 1.54 / 2.01 Mbit/s** |
| Ping to panel while streaming, 1400 B | min 6 ms, **avg 892 ms, max 4690 ms**, 0% loss |
| Ping to panel with the stream idle | **avg 24 ms, 0% loss** |

One `THIN_FRAME` is 460,808 B = **3.69 Mbit**. At the ~1-2 Mbit/s the panel's ESP32-C6
link actually delivers, a single frame takes **1.8-3.7 s**, so **0.3-0.5 fps is the
protocol's hard ceiling on this client** — exactly where it sits. The server pushing 10
fps into that pipe does not raise the frame rate; it only fills TCP buffers, which is what
the 4.7 s ping RTT is. Frames then arrive late enough to trip the client's
`network_timeout_ms`, the session drops, and the panel cycles OK -> STALE -> SEARCHING
through an exponential backoff.

Note the ceiling is not the server's fault and not fixable server-side alone: the panel's
P4 talks to its C6 radio over **1-bit SDIO** (`CONFIG_ESP_HOSTED_SDIO_1_BIT_BUS=y`; only
D0/D1 are pinned in `sdkconfig.defaults`, so 4-bit is not wired on this board). Treat
~1.5 Mbit/s as the design budget and do not assume it will improve.

## What v2 must change

Two independent problems, two independent fixes. **Flow control fixes the stability;
compression fixes the frame rate.** Either is useful alone; both are wanted.

### 1. Credit-based flow control (fixes STALE/disconnect flapping)

The server must never have more than a negotiated number of frames outstanding to a given
client, and must **drop rather than queue** when it has none.

- Client grants credit with `{"type":"thin_ready","seq":<last seq it finished with>}`.
- Server starts each connection with the credit count from the handshake (default **2** —
  one frame on the wire while the client works on the previous one; deeper just recreates
  the queue).
- On render completion with zero credit, the server **discards that frame for this
  client**. It never buffers it. When credit arrives it sends the *newest* frame it has,
  never a backlog entry.
- No timeout-based client eviction for slowness. A client at 0.4 fps is healthy on this
  link; with credits it can no longer cause buffer growth, so slowness is not a fault.

This alone removes the multi-second queueing delay, the client-side timeouts, and the
knock-on latency the stream inflicts on everything else the board is doing.

### 2. Compressed frames (fixes the frame rate)

Add a JPEG frame type. The point-cloud/SLAM/IR views are mostly flat black with sparse
coloured points — extremely JPEG-friendly. Expect **~15-30 KB/frame**, a 15-30x
reduction, which turns the ~1.5 Mbit/s budget into a realistic **5-10 fps**.

The client decodes in hardware: the P4 has a JPEG decode block
(`CONFIG_SOC_JPEG_DECODE_SUPPORTED=y`, `esp_driver_jpeg` in IDF 6) that outputs RGB565
straight into the existing PSRAM triple buffer, so the zero-copy canvas path in
`prop_ui.c` is unchanged.

Quality is negotiated, not hardcoded — the rig should be free to trade quality for rate.

## Wire format

`tag = 1` (`THIN_FRAME`, raw RGB565) keeps its exact v1 layout and meaning, so a v1 client
against a v2 server still works. New:

```
THIN_FRAME_JPEG
  u32 tag         = 2
  u16 width                     // 480
  u16 height                    // 480
  u32 seq                       // monotonic per connection, echoed in thin_ready
  u32 payload_len               // bytes of JPEG data following
  u8  jpeg[payload_len]         // baseline JPEG, YCbCr 4:2:0
```

Header is 16 bytes (v1's is 8). `seq` is what makes credit accounting unambiguous.

**Unchanged from v1 and still mandatory:** a frame MUST be one unfragmented WebSocket
message (FIN=1, no continuation frames). The embedded client reassembles from
`payload_len`/`payload_offset` across `WEBSOCKET_EVENT_DATA` callbacks, and WS-level
fragmentation restarts that accounting. TCP segmentation is fine and expected.

### Handshake

Client speaks first, immediately after the WS upgrade:

```json
{"type": "thin_hello", "proto": 2, "client": "crowpanel-p4",
 "accept": ["jpeg", "rgb565"], "width": 480, "height": 480,
 "credits": 2, "max_frame_bytes": 262144}
```

Server replies:

```json
{"type": "thin_hello_ack", "proto": 2, "encoding": "jpeg",
 "width": 480, "height": 480, "quality": 75, "credits": 2}
```

- A server that never sees `thin_hello` MUST fall back to exact v1 behaviour (raw
  `tag = 1` frames, free-running cadence) so existing clients are unaffected.
- A client that gets no `thin_hello_ack` MUST assume v1 and cope with free-running
  frames — it cannot rely on credits being honoured.
- `encoding` in the ack is authoritative; the client renders whatever tag arrives and
  drops tags it did not negotiate.
- `width`/`height` in the ack are authoritative too, which is the seam for later
  resolution negotiation (e.g. dropping to 320x320 on a bad link) with no new messages.

### Telemetry additions

`thin_telemetry`'s existing `fps` field is the rig's internal render rate. On this link
that reads ~29.9 while the panel receives ~0.4, and the panel used to display it as if it
were the frame rate — the single most misleading number in the whole system. It stays
(it is genuinely useful) but the per-client truth must be alongside it:

```json
{"type": "thin_telemetry", "fps": 29.9,
 "tx_fps": 0.4, "tx_bytes_per_s": 184320, "dropped": 1287, ...}
```

- `tx_fps` — frames actually **sent to this client** over the last few seconds.
- `tx_bytes_per_s` — bytes on the wire to this client; the number to watch against the
  ~1.5 Mbit/s budget.
- `dropped` — frames skipped for this client since connect, i.e. how hard flow control is
  working. A large and growing `dropped` with a healthy `tx_fps` is normal and correct.

## Client-side follow-on (this repo)

Already landed 2026-08-18 (`main/prop_lidar.c`, independent of the server work):

- Delivered-rate measurement (`link_fps`) from a windowed count of frame arrivals, shown
  on the panel as **RX FPS** and exposed as `lidar.link_fps` in `/telemetry`. The rig's
  `fps` is still reported in `/telemetry` but is no longer displayed as the panel's rate.
- Adaptive staleness — stale is now 3x the session's own measured cadence (clamped
  3-15 s) instead of a fixed 2 s that sat *below* the healthy interframe time.
- Panel gating (`prop_lidar_set_active`): the socket only lives while `PK_LIDAR` is on
  screen, plus a 20 s grace window. Verified: ping to the board went from avg 892 ms
  (stream always on) to avg 24 ms with the panel closed.

Landed 2026-08-19, completing this spec:

1. `thin_hello` on connect (`send_hello`), `thin_hello_ack` honoured (`on_hello_ack` →
   `s_proto`/`s_encoding_jpeg`), v1 kept as the fallback for a server that never acks.
   The hello goes out on a dedicated `LIDAR_EVT_CONNECTED` wake rather than the next
   100 ms poll, so the server does not free-run a 460 KB v1 frame first.
2. `thin_ready` from `prop_lidar_frame_consumed(seq)`, called by `ui_observer` right
   after the canvas retarget. Grants are counted in `s_frame_seq` units so a UI that
   skips a frame still returns BOTH credits — granting per-consumption without that
   would drain a 2-deep window to a permanent stall. A failed send is deliberately not
   marked acked, so the next poll retries it.
3. `tag = 2` decoded by `esp_driver_jpeg` straight into the PSRAM triple buffer
   (`decode_jpeg_into`). The buffers moved to `jpeg_alloc_decoder_mem` because the
   engine's DMA output needs a cache-line-aligned address and size.
   **The RX buffer did NOT shrink**: "a v2 client still renders against a v1 server" is
   also an acceptance bar, and a v1 server's raw frame is still 460,808 B. It is PSRAM.
4. `credits` left at 2 — the link is not idling between frames (see §Measured).

Also fixed here because it sits directly on the handshake path: WS **text** messages were
dropped unless they arrived in a single callback, so any JSON straddling a TCP segment
boundary was lost. Measured on hardware as `payload_len=655 off=0 len=307`. That silently
ate `thin_telemetry` and, on the first session tried, the one `thin_hello_ack` that decides
v1-vs-v2 — the client sat on the uncompressed path with no way to tell. Text now reassembles
into its own buffer exactly as frames do.

## Measured (2026-08-19, on hardware, live rig, 10 min soak)

| | v1 baseline | v2 measured | bar |
|---|---|---|---|
| Frame size | 460,808 B | **~9,100 B** (q75) | — |
| Bytes/s to the panel | ~1.5 Mbit/s | **~57 KB/s = 456 kbit/s** | — |
| Sustained frame rate | 0.37-0.51 fps | **6.52 fps** | >= 4 |
| `SEARCHING` transitions | one per ~20-60 s | **0** (and zero STALE — one unbroken OK) | 0 |
| Ping avg while streaming | 892 ms | **39.8 ms**, 0% loss, 1400 B | < ~150 ms |

All four bars pass. Compression bought ~50x smaller frames and ~15x the frame rate;
flow control plus the scan fix below bought a link that is now *quieter under load than
the old one was idle*. Compatibility was checked in both directions on hardware: this v2
client renders correctly against the unmodified v1 rig (raw tag-1 frames, 8x8 `ir_grid`),
and the v2 rig serves a v1 client unchanged.

Colour correctness was checked against the rig's own render of the same frame rather than
by eye: the panel's hue histogram matches it bin for bin, where a wrong `rgb_order` would
smear flat across all hues. `esp_driver_jpeg`'s `JPEG_DEC_RGB_ELEMENT_ORDER_BGR` is
"small endian" for RGB565 — a byte order, not a channel swap — and is the correct value
for LVGL-native RGB565 on this panel.

### The first run failed, and the cause was not this protocol

The first soak measured 3.79 fps, zero SEARCHING (so flow control was already working)
but a **1058 ms** average ping — worse than the v1 baseline it was meant to fix. The
control measurement is what identified it: with the panel closed and no stream at all,
the board still pinged at **861 ms avg, 4.7 s max, 6.7% loss**, against 24 ms the day
before. The stream's own contribution had in fact fallen from ~868 ms to ~197 ms; it was
sitting on a floor 35x higher than it used to be.

That floor was `main/prop_ftm.c`. `ftm_task` ran `prop_net_scan_raw()` — a full WiFi scan,
which takes the STA radio off-channel for seconds — **every 8 s, unconditionally, for the
life of the boot**, whether or not the RANGE panel was open. It was the same failure
`prop_lidar_set_active` exists to fix, in a different instrument, and it accounted for
every remaining symptom: bursty delivery (`link_fps` reading ~1.2 against a 3.79 average,
i.e. ~4.3 s since the last frame at any sample), the OK<->STALE flapping, and the frame
rate shortfall. Gating that scan on `PK_RANGE` the same way (`prop_ftm_set_active`, 20 s
grace) produced the table above — 3.79 -> 6.52 fps and 1058 -> 39.8 ms — with no change
to the protocol code at all.

The lesson generalises: **on this board, a background instrument that touches the C6 is a
tax on every other instrument, and it has to be gated on its panel.** Two of them have now
been caught this way. Any third should be gated when it is written, not after.

## Follow-on landed alongside (2026-08-19)

- **Full-resolution IR preview.** The sidebar thumbnail was 8x8 because
  `thin_render.extract_ir_grid` block-mean downsampled to 64 cells before sending — a ~97%
  throwaway of a sensor whose native reflectance grid is 54x42 = 2268 zones. The client now
  advertises `ir_cells` in `thin_hello` and the rig answers with `ir_grid_b64` (base64
  uint8 zones) plus `ir_grid_w`/`ir_grid_h`, which are read per message because the pane
  rotates with gravity and transposes them. A client that does not ask, or whose budget is
  too small, still gets the exact v1 8x8. Verified on hardware both ways — and the live rig
  answered `ir_grid_w x ir_grid_h = 42 x 54`, i.e. already transposed, so the per-message
  read is not a theoretical precaution: a client that cached 54x42 would render it wrong
  today. 2268 zones cost 3024 base64 chars, ~3 KB per telemetry message at 2 Hz.
- **`quality` and `host` are runtime-settable** — `POST /cmd {"cmd":"lidar","quality":N}`
  and `{"cmd":"lidar","host":"h:p"}`, persisted to NVS as `lidar_q`/`lidar_host`, both
  applied to the live session (quality re-runs `thin_hello`; host drops and reconnects).
  `lidar_host` had been the documented mDNS escape hatch since v1 with nothing in firmware
  able to write it.
- **Text-message reassembly** (see above) — the defect that made the very first session
  negotiate v1 without any way to tell.

## Rejected alternatives

- **Just lower the server's send rate to ~0.5 fps.** Fixes the queueing but locks in the
  bad frame rate; compression is what actually buys frames.
- **Just compress, no flow control.** 20 KB frames at 10 fps is 1.6 Mbit/s — still at or
  over the budget, so the same queue grows, just more slowly. Both are needed.
- **Delta/tile updates instead of JPEG.** Better in theory for a mostly-static scene, but
  it needs damage tracking server-side and a compositor client-side, and it degrades badly
  under orbit (every pixel changes). JPEG + hardware decode is far less machinery.
- **Client-side downscale request only (320x320 raw).** 205 KB/frame is still ~2x the
  per-second budget for one frame, and it costs resolution the operator can see. The
  handshake leaves this available as a fallback lever without designing around it.
