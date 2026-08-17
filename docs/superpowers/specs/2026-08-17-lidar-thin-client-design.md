# LiDAR thin-client render: CrowPanel ↔ lidar-roomscanner

**Status:** Approved for planning
**Date:** 2026-08-17
**Repos touched:** `CrowPanelProp` (this repo, submodule path `submodules/lidar-roomscanner`) and `lidar-roomscanner` (`hellosamblack/lidar-roomscanner`)

## Goal

Show a live render of the `lidar-roomscanner` rig's point cloud / SLAM / IR view on the
CrowPanel communicator's 1024×600 panel, with minimal on-device controls (orbit the
camera, switch view mode, start/stop recording) and a compact sensor-telemetry readout —
without building a 3D engine or an image codec into the ESP32-P4 firmware.

## Non-goals

- No true 3D point-cloud/mesh rendering *on* the CrowPanel. The P4 has no GPU and LVGL 9
  has no 3D pipeline; building one is out of scope.
- No image compression (JPEG/PNG) in v1. See "Open questions" for the deferred codec path.
- No ranging-profile/manual-parameter device control from the CrowPanel in v1 (dropped
  during brainstorming — the DEVICE panel's exposure/FPS/power controls stay browser-only
  for now).
- No mesh/point-cloud data ever lands on the CrowPanel raw — it only ever receives an
  already-rendered raster frame. The heavy 3D math stays server-side.

## Architecture

**Thin-client model:** the CrowPanel is a remote display + input relay, nothing more. All
3D rendering (point cloud / SLAM mesh / IR view, camera orbit) happens server-side in
`lidar-roomscanner`, via a new **Open3D offscreen renderer** (Open3D is already a
roomscanner dependency for SLAM meshing, and has a real offscreen-rendering API — chosen
over driving a headless browser against the existing Three.js UI, which would need a full
Chromium instance per stream and is too slow for a real-time control loop). The server
rasterizes the current view to a fixed 480×480 frame and streams raw RGB565 bytes — no
codec, matching the panel's native pixel format so the P4 does zero decode work.

Two new components, one per repo:

### `lidar-roomscanner` (server side)

- **`host/src/roomscan/thin_render.py`** (new module) — owns an Open3D
  `OffscreenRenderer`, a camera-orbit state (yaw/pitch/zoom), and a "current mode"
  (`point_cloud` / `slam` / `ir`). On each render tick it pulls the latest data already
  computed by the existing pipeline (the same `pts`/`colors` arrays that feed
  `pack_point_cloud`, the latest SLAM mesh, or the latest IR frame — no new sensor-side
  work), applies the orbit state, renders to 480×480 RGBA, and converts to RGB565.
- **New `/ws-thin` WebSocket endpoint** in `web.py`, with its own broadcast loop/task
  (mirroring the existing point-cloud broadcaster's task structure) so it never blocks the
  main reader thread or the existing `/ws`/`/ws-mesh` clients.
- **New mDNS advertisement.** The server does not advertise itself via mDNS today (the
  existing `zeroconf` dependency in `sources.py` is used for discovering the MCU/travel
  router, not for advertising the web server). Add a `ServiceInfo` registration for the
  web server itself so the CrowPanel can resolve it without a hardcoded IP.

### CrowPanel (client side, this repo)

- **`main/prop_lidar.c`** (new module) — adds `esp_websocket_client` as a new managed
  component dependency (none of `esp_http_client`/`esp_websocket_client` exist in this
  firmware today; this is a new outbound-network capability). Owns:
  - mDNS resolution of the roomscanner host.
  - The WebSocket connection lifecycle (connect, reconnect w/ backoff, teardown).
  - A PSRAM-backed RGB565 frame buffer (`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`,
    480×480×2 = 460,800 B), double-buffered so the UI never reads a partially-written
    frame.
  - A mutex-protected cached telemetry struct.
  - This mirrors `prop_track.c`'s established pattern: background FreeRTOS task owns
    live state behind a mutex in PSRAM; `ui_observer` reads a copy.
- **New `PK_LIDAR` panel** in `main/prop_ui.c` — full-screen hero panel (no header/BACK,
  own bordered layout, like `PK_MOTION`), reachable via rail deep-link `goto lidar`
  (added to the `POST /cmd {"cmd":"ui","screen":"lidar"}` screen list). Contains:
  - A 480×480 `lv_canvas` in the panel's native RGB565 format (`swap_bytes=false`,
    matching the display's existing convention — no ARGB, no per-pixel alpha needed here
    since it's a straight frame blit, not hand-painted graphics).
  - A curated telemetry strip: measured FPS, power mode, I3C bus airtime %, point count,
    record status, link status.
  - Controls: touch-drag on the render region (and/or the physical dial) sends orbit
    deltas; the tab/action button cycles view mode; a themed toggle button (styled like
    the SCANNER speaker toggle) starts/stops recording.

## Protocol contract (`/ws-thin`) — hand this section to `lidar-roomscanner`

New WebSocket endpoint: **`GET /ws-thin`**. Independent of the existing `/ws` and
`/ws-mesh` protocol (see `docs/web-protocol.md`) — a thin client never receives
`POINT_CLOUD`/`MESH`/`IR_IMAGE` binary tags or the full JSON message surface. This keeps
the embedded client's parsing trivial and avoids adding a subscribe/filter layer to the
existing sockets.

### Binary frame (server → client): `THIN_FRAME`

Fixed 480×480 for v1 (no resolution negotiation — see "Open questions").

```
u32 tag        = 1
u16 width      = 480
u16 height     = 480
u8  pixels[width * height * 2]   // RGB565, little-endian, row-major
```

Target cadence: **~10 fps** (~460 KB/frame → ~4.6 MB/s). If the client falls behind
(WS send buffer growing), the server drops to the freshest frame rather than queueing —
same decimation policy as the existing point-cloud broadcaster.

### JSON out: `thin_telemetry`

Sent at a lower cadence (~2 Hz) — deliberately not sent per-frame:

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

`mode` and `recording` here are the **authoritative** state — the client renders whatever
this says, not what it last requested. This makes state convergence self-healing: a
dropped command frame corrects itself at the next telemetry tick instead of needing an
ack/retry protocol.

### JSON in (client → server commands)

```json
{"type": "thin_orbit", "dyaw": 3.5, "dpitch": -1.0, "dzoom": 0.0}
{"type": "thin_mode", "mode": "point_cloud" | "slam" | "ir"}
{"type": "thin_record", "on": true}
```

- `thin_orbit` deltas are relative and applied cumulatively server-side each time one
  arrives; the client can send these at whatever rate touch-drag/dial events occur
  (best-effort, no ack — lossy-tolerant by design).
- `thin_mode` switches the render source; `thin_record` reuses the existing record
  start/stop logic already wired to the browser's record control (`web.py`'s existing
  `record` handler) — no new recording logic, just a second entry point into it.

### mDNS advertisement (new requirement on the server)

Register a `ServiceInfo` for the web server on startup (e.g. service type
`_roomscan._tcp.local.`, instance name `roomscan`, port `8000`). The CrowPanel resolves
this by service type the same way it already advertises itself as `comm-unit-7.local`
(`PROP_HOSTNAME`) — this is new work in `lidar-roomscanner`, not a reuse of existing
advertisement.

## Data flow

1. Operator navigates to the `PK_LIDAR` panel on the CrowPanel.
2. `prop_lidar.c`'s background task resolves the roomscanner host via mDNS, opens
   `ws://<host>:8000/ws-thin`.
3. Server's `/ws-thin` handler registers the new connection with the thin-render broadcast
   loop; that loop renders at ~10 fps regardless of how many thin clients are attached
   (mirrors the existing point-cloud broadcaster's "decimate, don't queue" behavior) and
   sends `THIN_FRAME` to each, plus `thin_telemetry` at ~2 Hz.
4. `prop_lidar.c` writes each incoming frame into the PSRAM double buffer; `ui_observer`
   (already running at ~20 Hz, only when `s_cur_kind == PK_LIDAR`) blits the newest
   complete frame into the canvas — a cheap `lv_canvas` buffer copy, no per-pixel LVGL
   draw calls.
5. Touch-drag on the render region and dial rotation compute `dyaw`/`dpitch`/`dzoom` and
   send `thin_orbit`; the tab/action button sends `thin_mode`; the record toggle button
   sends `thin_record`. All three go out over the same open socket.
6. Telemetry strip updates from the cached `thin_telemetry` struct, same
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
- **Server-side renderer init failure** (e.g. Open3D can't get a rendering context):
  `/ws-thin` should reject the connection with a JSON error message and close, not crash
  the `roomscan.web` process — the existing `/ws`/`/ws-mesh` clients must be unaffected.
- **Backpressure:** freshest-frame-wins, same policy as the existing broadcaster; a thin
  client that can't keep up gets frames dropped for it specifically, not a global slowdown.

## Testing / validation plan

**`lidar-roomscanner` side:**
- Unit test the `THIN_FRAME` packer (byte layout, dimensions, RGB565 conversion
  correctness).
- A small standalone script (same shape as `tools/query_mdns.py` / the MCP server's
  `RigSession`) that connects to `/ws-thin` as a fake client, saves received frames as
  PNGs for a visual sanity check, and round-trips `thin_orbit`/`thin_mode`/`thin_record`.
- End-to-end validation using the existing `--replay recordings/scan.bin` capability — no
  physical LiDAR hardware needed to exercise the thin-client path.

**CrowPanel side:**
- `tools/prop.py shot` against the `PK_LIDAR` panel to visually confirm the render region
  is populating.
- `tools/prop.py watch` filtered to the new telemetry fields to confirm link/fps/mode
  populate and update.
- Manual on-hardware check of touch-drag/dial orbit responsiveness and the record-toggle
  round-trip (toggle on CrowPanel → confirm recording state change reflected in the
  browser UI too, since both are driven by the same underlying `record` handler).

## Open questions / deferred work

- **Resolution/rate negotiation:** v1 hardcodes 480×480 @ ~10 fps on both sides. A
  `thin_hello` handshake to negotiate size/rate is plausible future work but adds protocol
  surface not needed for a single known client class.
- **JPEG + P4 hardware decode:** the P4 has an unused onboard JPEG decode block
  (`CONFIG_SOC_JPEG_DECODE_SUPPORTED=y`) and no LVGL image decoder is currently wired up.
  If bandwidth becomes a real constraint later, this is the documented upgrade path — not
  needed for v1's raw-RGB565 approach.
- **Ranging-profile control from the CrowPanel:** explicitly dropped from this design;
  device parameter control (exposure/FPS/power mode) stays a browser-only capability for
  now.
