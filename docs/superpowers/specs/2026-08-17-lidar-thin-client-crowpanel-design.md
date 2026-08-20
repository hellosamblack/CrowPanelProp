# LiDAR thin-client render — CrowPanel side

**Status:** Approved for planning
**Date:** 2026-08-17
**Repo:** `CrowPanelProp` (this repo)
**Companion spec:** `lidar-roomscanner` repo,
`docs/superpowers/specs/2026-08-17-thin-client-render-design.md` (server side). The
**Protocol contract** section below is duplicated verbatim in both specs — it's the shared
interface, not a cross-reference — keep them in sync if it changes.

## Goal

Show a live render of the `lidar-roomscanner` rig's point cloud / SLAM / IR view on the
CrowPanel communicator's 1024×600 panel, with minimal on-device controls (orbit the
camera, switch view mode, start/stop recording) and a compact sensor-telemetry readout —
without building a 3D engine or an image codec into the ESP32-P4 firmware.

## Non-goals

- No true 3D point-cloud/mesh rendering *on* the CrowPanel. The P4 has no GPU and LVGL 9
  has no 3D pipeline; building one is out of scope.
- No image compression (JPEG/PNG) in v1 — raw RGB565 only. See "Open questions" for the
  deferred codec path.
- No ranging-profile/manual-parameter device control from the CrowPanel in v1 (dropped
  during brainstorming — the DEVICE panel's exposure/FPS/power controls stay browser-only
  for now).
- No mesh/point-cloud data ever lands on the CrowPanel raw — it only ever receives an
  already-rendered raster frame. The heavy 3D math stays server-side.

## Architecture

**Thin-client model:** the CrowPanel is a remote display + input relay, nothing more. All
3D rendering happens server-side in `lidar-roomscanner` (see the companion spec); the
CrowPanel connects out to it, receives raw RGB565 frames, blits them into a canvas, and
sends back small JSON control messages.

**`main/prop_lidar.c`** (new module) — adds `esp_websocket_client` as a new managed
component dependency (none of `esp_http_client`/`esp_websocket_client` exist in this
firmware today; this is a new outbound-network capability). Owns:

- mDNS resolution of the roomscanner host (the server advertises itself — see companion
  spec; no hardcoded IP).
- The WebSocket connection lifecycle (connect, reconnect w/ backoff, teardown).
- A PSRAM-backed RGB565 frame buffer (`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`,
  480×480×2 = 460,800 B), double-buffered so the UI never reads a partially-written frame.
- A mutex-protected cached telemetry struct.

This mirrors `prop_track.c`'s established pattern: background FreeRTOS task owns live
state behind a mutex in PSRAM; `ui_observer` reads a copy.

**New `PK_LIDAR` panel** in `main/prop_ui.c` — full-screen hero panel (no header/BACK, own
bordered layout, like `PK_MOTION`), reachable via rail deep-link `goto lidar` (added to
the `POST /cmd {"cmd":"ui","screen":"lidar"}` screen list). Contains:

- A 480×480 `lv_canvas` in the panel's native RGB565 format (`swap_bytes=false`, matching
  the display's existing convention — no ARGB, no per-pixel alpha needed here since it's a
  straight frame blit, not hand-painted graphics).
- A curated telemetry strip: measured FPS, power mode, I3C bus airtime %, point count,
  record status, link status.
- Controls: touch-drag on the render region (and/or the physical dial) sends orbit deltas;
  the tab/action button cycles view mode; a themed toggle button (styled like the SCANNER
  speaker toggle) starts/stops recording.

Add the panel following the repo's standard 6-step checklist (`PK_LIDAR` enum value,
`build_lidar_panel`, `case` in `open_panel`, deep-link name, NULL widget pointers +
`s_cur_kind` guard in `close_panel`).

## Protocol contract (`/ws-thin`)

New WebSocket endpoint served by `lidar-roomscanner`: **`GET /ws-thin`**. Independent of
the existing `/ws` and `/ws-mesh` protocol (see that repo's `docs/web-protocol.md`) — this
client never receives `POINT_CLOUD`/`MESH`/`IR_IMAGE` binary tags or the full JSON message
surface. This keeps the embedded client's parsing trivial.

### Binary frame (server → client): `THIN_FRAME`

Fixed 480×480 for v1 (no resolution negotiation — see "Open questions").

```
u32 tag        = 1
u16 width      = 480
u16 height     = 480
u8  pixels[width * height * 2]   // RGB565, little-endian, row-major
```

Target cadence: **~10 fps** (~460 KB/frame → ~4.6 MB/s). The server drops to the freshest
frame if this client falls behind rather than queueing.

**A `THIN_FRAME` MUST be sent as a single, unfragmented WebSocket message** (one binary
frame with FIN=1 — no continuation frames). The embedded client reassembles each frame
from the several `WEBSOCKET_EVENT_DATA` callbacks the transport hands it, keyed on
`payload_len` (total message size) + `payload_offset`; WS-level fragmentation restarts
that accounting per fragment and the frame is rejected as a size mismatch. TCP-level
segmentation is fine and expected — this is about WS framing only.
*(Shared contract: the server-side spec in the `lidar-roomscanner` repo needs the same
note; it could not be edited from this repo.)*

### JSON out: `thin_telemetry`

Sent at a lower cadence (~2 Hz):

```json
{
  "type": "thin_telemetry",
  "fps": 9.8,
  "power_mode": "ULP",
  "i3c_airtime_pct": 35.6,
  "point_count": 2268,
  "recording": false,
  "mode": "point_cloud",
  "link": "ok"
}
```

`mode` and `recording` here are the **authoritative** state — this panel renders whatever
this says, not what it last requested. State convergence is self-healing: a dropped
command frame corrects itself at the next telemetry tick, so no ack/retry protocol is
needed on the client.

### JSON in (client → server commands)

```json
{"type": "thin_orbit", "dyaw": 3.5, "dpitch": -1.0, "dzoom": 0.0}
{"type": "thin_mode", "mode": "point_cloud" | "slam" | "ir"}
{"type": "thin_record", "on": true}
```

`thin_orbit` deltas are relative, sent at whatever rate touch-drag/dial events occur
(best-effort, no ack expected — lossy-tolerant by design). `thin_mode`/`thin_record` are
fire-and-forget from this side too, since the telemetry tick is what confirms them.

### mDNS discovery

The server advertises itself (service type `_roomscan._tcp.local.`, instance name
`roomscan`, port `8000` — see companion spec). `prop_lidar.c` resolves this the same way
the CrowPanel itself is resolved externally as `comm-unit-7.local` (`PROP_HOSTNAME`), just
in the client role instead of the server role.

## Data flow

1. Operator navigates to the `PK_LIDAR` panel.
2. `prop_lidar.c`'s background task resolves the roomscanner host via mDNS, opens
   `ws://<host>:8000/ws-thin`.
3. `prop_lidar.c` writes each incoming frame into the PSRAM double buffer; `ui_observer`
   (already running at ~20 Hz, only when `s_cur_kind == PK_LIDAR`) blits the newest
   complete frame into the canvas — a cheap `lv_canvas` buffer copy, no per-pixel LVGL
   draw calls.
4. Touch-drag on the render region and dial rotation compute `dyaw`/`dpitch`/`dzoom` and
   send `thin_orbit`; the tab/action button sends `thin_mode`; the record toggle button
   sends `thin_record`. All three go out over the same open socket.
5. Telemetry strip updates from the cached `thin_telemetry` struct, same
   cached-state-then-observer-reads-it pattern as every other instrument.

## Error handling

- **mDNS resolution / WS connect failure:** retry with exponential backoff (cap ~30s).
  Panel shows `LINK: SEARCHING` in `COL_MUTE`/`COL_ALERT` per the camera-legibility rule
  (never `COL_DIM` for text that must be read).
- **No new frame for >2s while connected:** keep displaying the last good frame, but flag
  it `LINK: STALE` in the telemetry strip rather than freezing silently.
- **Frame size/format mismatch:** drop the malformed frame, log, keep last good frame —
  never crash the panel on a bad payload.
- **Command send failure (socket closed mid-send):** no retry needed — `thin_orbit` is
  lossy-tolerant, and `thin_mode`/`thin_record` self-correct at the next `thin_telemetry`
  tick per the "server telemetry is authoritative" design above.

## Testing / validation plan

- `tools/prop.py shot` against the `PK_LIDAR` panel to visually confirm the render region
  is populating.
- `tools/prop.py watch` filtered to the new telemetry fields to confirm link/fps/mode
  populate and update.
- Manual on-hardware check of touch-drag/dial orbit responsiveness and the record-toggle
  round-trip (toggle on CrowPanel → confirm recording state change reflected in the
  browser UI too, since both are driven by the same underlying server-side `record`
  handler).
- Requires a running `lidar-roomscanner` server with `/ws-thin` implemented (companion
  spec) — can be run with `--replay recordings/scan.bin` on that side, no physical LiDAR
  hardware needed for end-to-end validation.

## Open questions / deferred work

- **Resolution/rate negotiation:** v1 hardcodes 480×480 @ ~10 fps on both sides. A
  `thin_hello` handshake to negotiate size/rate is plausible future work but adds protocol
  surface not needed for a single known client class.
- **JPEG + P4 hardware decode:** the P4 has an unused onboard JPEG decode block
  (`CONFIG_SOC_JPEG_DECODE_SUPPORTED=y`) and no LVGL image decoder is currently wired up.
  If bandwidth becomes a real constraint later, this is the documented upgrade path — not
  needed for v1's raw-RGB565 approach.

> **RESOLVED 2026-08-18 — bandwidth is the constraint, and both deferred items above are
> now required.** Measured on hardware: the rig pushes 10 fps / 37 Mbit/s happily to a
> wired client, but the CrowPanel's link carries ~1-2 Mbit/s, so a 460 KB frame takes
> 1.8-3.7 s and the panel tops out at **0.4 fps** with the queue backing up until the
> session drops. See **`2026-08-18-lidar-thin-frame-bandwidth-protocol.md`** for the v2
> protocol (credit-based flow control + a JPEG frame type) that supersedes the "Protocol
> contract" and "Error handling" sections here.
- **Ranging-profile control from the CrowPanel:** explicitly dropped from this design;
  device parameter control (exposure/FPS/power mode) stays a browser-only capability for
  now.
