# LiDAR Thin-Client Render (CrowPanel side) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Show a live render of the `lidar-roomscanner` rig's point cloud / SLAM / IR view
on the CrowPanel's 1024×600 panel, with minimal on-device controls (orbit, mode-select,
record toggle) and a compact telemetry readout, by connecting out as a thin WebSocket
client to a new `/ws-thin` endpoint on that server.

**Architecture:** A new `main/prop_lidar.c` module (mirroring `prop_track.c`'s
background-task-owns-state-behind-a-mutex pattern) resolves the roomscanner host via
mDNS, maintains an `esp_websocket_client` connection, decodes raw RGB565 frames into a
PSRAM double buffer, and caches a small telemetry struct. A new full-screen `PK_LIDAR`
panel in `main/prop_ui.c` blits the newest frame into a native-RGB565 `lv_canvas` each UI
tick and exposes touch-drag/dial orbit, tab mode-cycle, and a record-toggle button. A new
stdlib-only mock `/ws-thin` server (`tools/mock_ws_thin.py`) lets every task be verified
on real hardware without depending on the (separately planned) server-side work in the
`lidar-roomscanner` repo.

**Tech Stack:** ESP-IDF 6.0.1, LVGL 9.4 (`esp_lvgl_port` 2.8), FreeRTOS, `esp_websocket_client`
(new managed dependency), `espressif/mdns` (already a dependency), cJSON (already a
dependency), Python 3 stdlib (mock server + verification).

**Spec:** `docs/superpowers/specs/2026-08-17-lidar-thin-client-crowpanel-design.md`

## Global Constraints

- Protocol: `GET /ws-thin`, binary `THIN_FRAME` = `u32 tag=1, u16 w=480, u16 h=480, u8
  pixels[w*h*2]` (RGB565, little-endian, row-major); JSON out `thin_telemetry` at ~2 Hz
  (`fps`, `power_mode`, `i3c_airtime_pct`, `point_count`, `recording`, `mode`, `link` —
  `mode`/`recording` are server-authoritative); JSON in `thin_orbit {dyaw,dpitch,dzoom}`,
  `thin_mode {mode}`, `thin_record {on}`. Frame size is fixed 480×480 in v1 — no
  negotiation.
- No JPEG/image codec in v1 — raw RGB565 only, matching the panel's native pixel format.
- No ranging-profile/manual-parameter device control from the CrowPanel in v1.
- Any large buffer (frame double-buffer, canvas backing) must be allocated with
  `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)` — never raise `LV_MEM` / switch off the
  custom PSRAM allocator (starves esp_hosted's SDIO DMA mempool → boot loop).
- Never call networking APIs under the LVGL lock (`lvgl_port_lock()`/`unlock()`) — cache
  values from the background task; the UI only ever reads the cache.
- Camera-legibility rule: never use `COL_DIM` for text the operator must read — use
  `COL_MUTE` + `FONT_BODY`/`FONT_HEAD` (bold Eurostile).
- New panels follow the repo's standard 6-step checklist: `PK_*` enum value,
  `build_<x>_panel`, `case` in `open_panel`, menu/deep-link entry, `prop_ui_goto` name,
  NULL widget pointers + `s_cur_kind` guard in `close_panel`.
- After adding `main/prop_lidar.c`, run `idf.py reconfigure` before the next build (the
  `main/CMakeLists.txt` GLOB only picks up new `.c` files at configure time).

---

## Task 1: Add the `esp_websocket_client` managed dependency

**Files:**
- Modify: `main/idf_component.yml`

**Interfaces:**
- Produces: the `esp_websocket_client.h` API (`esp_websocket_client_init`,
  `esp_websocket_client_start/stop/destroy`, `esp_websocket_register_events`,
  `esp_websocket_client_send_text`) available to later tasks.

- [ ] **Step 1: Add the dependency**

Open `main/idf_component.yml` and add a new entry after the existing `espressif/mdns: '*'`
line (matching the file's existing style of an inline comment + a loose `'*'` version for
non-fussy deps like `cjson`/`mdns`):

```yaml
  # LiDAR thin-client render stream: outbound WS client to lidar-roomscanner's
  # /ws-thin endpoint (docs/superpowers/specs/2026-08-17-lidar-thin-client-crowpanel-design.md)
  espressif/esp_websocket_client: '*'
```

- [ ] **Step 2: Reconfigure and build**

Run (Windows PowerShell, from repo root, after loading the ESP-IDF environment per
`CLAUDE.md`):

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Expected: the component manager resolves and downloads `esp_websocket_client` into
`managed_components/`, and the build completes with no new errors (nothing references
the new API yet, so this is purely a dependency-resolution check).

- [ ] **Step 3: Commit**

```bash
git add main/idf_component.yml dependencies.lock
git commit -m "build: add esp_websocket_client managed dependency"
```

---

## Task 2: Local mock `/ws-thin` server for hardware testing

**Files:**
- Create: `tools/mock_ws_thin.py`

**Interfaces:**
- Produces: a standalone script serving `ws://<host>:8000/ws-thin` that speaks exactly
  the protocol in Global Constraints — every later task on-device check connects to this
  instead of needing the real `lidar-roomscanner` server.

- [ ] **Step 1: Write the mock server**

Create `tools/mock_ws_thin.py`:

```python
#!/usr/bin/env python3
"""mock_ws_thin.py — stdlib-only stand-in for lidar-roomscanner's /ws-thin endpoint.

Speaks the exact protocol from
docs/superpowers/specs/2026-08-17-lidar-thin-client-crowpanel-design.md: sends synthetic
THIN_FRAME binary frames (480x480 RGB565, ~10 fps) and thin_telemetry JSON (~2 Hz), and
receives/logs thin_orbit/thin_mode/thin_record JSON commands from the client, feeding
thin_mode/thin_record back into telemetry so a real CrowPanel can be exercised end-to-end
without the real Open3D-backed server.

Usage: python tools/mock_ws_thin.py [--port 8000]
"""
import argparse
import base64
import hashlib
import json
import socket
import struct
import threading
import time

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
FRAME_W = 480
FRAME_H = 480
FRAME_INTERVAL = 1.0 / 10.0
TELEMETRY_INTERVAL = 0.5


def _accept_key(key: str) -> str:
    sha1 = hashlib.sha1((key + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(sha1).decode("ascii")


def _read_http_headers(conn: socket.socket) -> dict:
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = conn.recv(4096)
        if not chunk:
            raise ConnectionError("client closed during handshake")
        data += chunk
    head, _, _ = data.partition(b"\r\n\r\n")
    lines = head.decode("iso-8859-1").split("\r\n")
    headers = {}
    for line in lines[1:]:
        if ":" in line:
            k, v = line.split(":", 1)
            headers[k.strip().lower()] = v.strip()
    return headers


def _do_handshake(conn: socket.socket) -> None:
    headers = _read_http_headers(conn)
    key = headers.get("sec-websocket-key")
    if not key:
        raise ConnectionError("not a websocket upgrade request")
    accept = _accept_key(key)
    resp = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
    )
    conn.sendall(resp.encode("ascii"))


def _send_ws_frame(conn: socket.socket, payload: bytes, opcode: int) -> None:
    fin_op = 0x80 | opcode
    length = len(payload)
    if length < 126:
        header = struct.pack("!BB", fin_op, length)
    elif length < 65536:
        header = struct.pack("!BBH", fin_op, 126, length)
    else:
        header = struct.pack("!BBQ", fin_op, 127, length)
    conn.sendall(header + payload)


def _send_binary(conn: socket.socket, payload: bytes) -> None:
    _send_ws_frame(conn, payload, opcode=0x2)


def _send_text(conn: socket.socket, text: str) -> None:
    _send_ws_frame(conn, text.encode("utf-8"), opcode=0x1)


def _recv_ws_frame(conn: socket.socket):
    """Read one client (masked) frame. Returns (opcode, payload) or None on close."""
    hdr = conn.recv(2)
    if len(hdr) < 2:
        return None
    b0, b1 = hdr[0], hdr[1]
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    length = b1 & 0x7F
    if length == 126:
        length = struct.unpack("!H", conn.recv(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", conn.recv(8))[0]
    mask = conn.recv(4) if masked else b"\x00\x00\x00\x00"
    payload = b""
    while len(payload) < length:
        chunk = conn.recv(length - len(payload))
        if not chunk:
            return None
        payload += chunk
    if masked:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return opcode, payload


def _make_frame(tick: int) -> bytes:
    """Synthetic 480x480 RGB565LE pattern that visibly animates: diagonal bands that
    shift with `tick`, cheap enough to regenerate every call."""
    row = bytearray(FRAME_W * 2)
    band = (tick * 4) % 64
    for x in range(FRAME_W):
        v = (x + band) % 64
        color = ((v & 0x1F) << 11) | ((v * 2 & 0x3F) << 5) | (v & 0x1F)
        row[x * 2:x * 2 + 2] = struct.pack("<H", color)
    pixels = bytearray()
    for y in range(FRAME_H):
        shift = (y + band) % FRAME_W
        pixels += row[shift * 2:] + row[:shift * 2]
    return struct.pack("<IHH", 1, FRAME_W, FRAME_H) + bytes(pixels)


class ClientState:
    def __init__(self):
        self.mode = "point_cloud"
        self.recording = False
        self.lock = threading.Lock()


def _reader_loop(conn: socket.socket, state: ClientState, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            frame = _recv_ws_frame(conn)
        except OSError:
            break
        if frame is None:
            break
        opcode, payload = frame
        if opcode == 0x8:      # close
            break
        if opcode != 0x1:      # only text commands expected inbound
            continue
        try:
            msg = json.loads(payload.decode("utf-8"))
        except ValueError:
            continue
        print(f"[mock_ws_thin] recv: {msg}")
        with state.lock:
            if msg.get("type") == "thin_mode":
                state.mode = msg.get("mode", state.mode)
            elif msg.get("type") == "thin_record":
                state.recording = bool(msg.get("on", state.recording))
            # thin_orbit: logged above; the mock has no camera to actually move.
    stop.set()


def _writer_loop(conn: socket.socket, state: ClientState, stop: threading.Event) -> None:
    tick = 0
    next_frame = time.monotonic()
    next_telem = time.monotonic()
    while not stop.is_set():
        now = time.monotonic()
        if now >= next_frame:
            try:
                _send_binary(conn, _make_frame(tick))
            except OSError:
                stop.set()
                break
            tick += 1
            next_frame = now + FRAME_INTERVAL
        if now >= next_telem:
            with state.lock:
                telem = {
                    "type": "thin_telemetry",
                    "fps": 10.0,
                    "power_mode": "ULP",
                    "i3c_airtime_pct": 35.6,
                    "point_count": 2268,
                    "recording": state.recording,
                    "mode": state.mode,
                    "link": "ok",
                }
            try:
                _send_text(conn, json.dumps(telem))
            except OSError:
                stop.set()
                break
            next_telem = now + TELEMETRY_INTERVAL
        time.sleep(0.005)


def _handle_client(conn: socket.socket, addr) -> None:
    print(f"[mock_ws_thin] client connected: {addr}")
    try:
        _do_handshake(conn)
    except (ConnectionError, OSError) as exc:
        print(f"[mock_ws_thin] handshake failed: {exc}")
        conn.close()
        return
    state = ClientState()
    stop = threading.Event()
    reader = threading.Thread(target=_reader_loop, args=(conn, state, stop), daemon=True)
    reader.start()
    _writer_loop(conn, state, stop)
    stop.set()
    conn.close()
    print(f"[mock_ws_thin] client disconnected: {addr}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"[mock_ws_thin] serving ws://0.0.0.0:{args.port}/ws-thin (Ctrl-C to stop)")
    try:
        while True:
            conn, addr = srv.accept()
            _handle_client(conn, addr)   # one client at a time is enough for this mock
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify the handshake and stream against the existing `prop.py` WS client helpers**

Start the mock server in the background, then reuse `tools/prop.py`'s already-working
stdlib WS client (`_ws_connect`/`_ws_frames`, `tools/prop.py:92-147`) to sanity-check it
without writing a second client:

```bash
python tools/mock_ws_thin.py --port 8765 &
python - <<'EOF'
import sys, time
sys.path.insert(0, "tools")
import prop
sock, leftover = prop._ws_connect("127.0.0.1:8765", "/ws-thin")
frames = prop._ws_frames(sock, leftover)
opcode, payload = next(frames)
print("first frame opcode:", opcode, "len:", len(payload))
assert opcode == 0x2 and len(payload) == 8 + 480 * 480 * 2
opcode, payload = next(frames)
print("second frame opcode:", opcode)
sock.close()
EOF
kill %1
```

Expected: `first frame opcode: 2 len: 460808`, confirming the header + 480×480×2 pixel
payload size the CrowPanel firmware will need to parse.

- [ ] **Step 3: Commit**

```bash
git add tools/mock_ws_thin.py
git commit -m "test: add stdlib mock /ws-thin server for CrowPanel-side testing"
```

---

## Task 3: `prop_lidar` module skeleton — state, mutex, background task shell

**Files:**
- Create: `main/include/prop_lidar.h`
- Create: `main/prop_lidar.c`

**Interfaces:**
- Produces: `prop_lidar_init() -> esp_err_t`, `prop_lidar_get_telemetry(prop_lidar_telemetry_t *out) -> void`,
  `prop_lidar_get_frame(uint16_t *dst, uint32_t *out_seq) -> bool`,
  `prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom) -> void`,
  `prop_lidar_send_mode(prop_lidar_mode_t mode) -> void`,
  `prop_lidar_send_record(bool on) -> void`; the types `prop_lidar_mode_t`,
  `prop_lidar_link_t`, `prop_lidar_telemetry_t`, and constants `PROP_LIDAR_FRAME_W`,
  `PROP_LIDAR_FRAME_H`, `PROP_LIDAR_FRAME_PIXELS`. Later tasks fill in the task body;
  this task only wires the skeleton so it builds and the task logs its (idle) state.

- [ ] **Step 1: Write the header**

Create `main/include/prop_lidar.h`:

```c
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROP_LIDAR_FRAME_W      480
#define PROP_LIDAR_FRAME_H      480
#define PROP_LIDAR_FRAME_PIXELS (PROP_LIDAR_FRAME_W * PROP_LIDAR_FRAME_H)

typedef enum {
    PROP_LIDAR_MODE_POINT_CLOUD = 0,
    PROP_LIDAR_MODE_SLAM,
    PROP_LIDAR_MODE_IR,
} prop_lidar_mode_t;

typedef enum {
    PROP_LIDAR_LINK_SEARCHING = 0,  /* resolving mDNS / connecting */
    PROP_LIDAR_LINK_OK,             /* connected, frames arriving within budget */
    PROP_LIDAR_LINK_STALE,          /* connected, but no frame in >2s */
} prop_lidar_link_t;

typedef struct {
    prop_lidar_link_t link;
    float             fps;
    char              power_mode[16];
    float             i3c_airtime_pct;
    int               point_count;
    bool              recording;
    prop_lidar_mode_t mode;
} prop_lidar_telemetry_t;

/* Starts the background connection task. Call once, after prop_net_init() has brought
 * up WiFi + mdns_init(). Non-fatal to call even if WiFi is down: the task just keeps
 * retrying discovery in the background. */
esp_err_t prop_lidar_init(void);

/* Copies the newest complete RGB565 frame (PROP_LIDAR_FRAME_PIXELS uint16_t's,
 * little-endian) into dst, which must be at least PROP_LIDAR_FRAME_PIXELS * 2 bytes.
 * Returns true and sets *out_seq to the frame's sequence number if a frame was
 * available; returns false (leaving dst untouched) before the first frame ever arrives.
 * Cheap to call every UI tick — skip the canvas blit when *out_seq hasn't changed. */
bool prop_lidar_get_frame(uint16_t *dst, uint32_t *out_seq);

/* Copies the current cached telemetry. Always succeeds; before the first thin_telemetry
 * message arrives it reports link=PROP_LIDAR_LINK_SEARCHING and zeroed fields. */
void prop_lidar_get_telemetry(prop_lidar_telemetry_t *out);

/* Best-effort outbound commands; no-ops quickly if not currently connected. */
void prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom);
void prop_lidar_send_mode(prop_lidar_mode_t mode);
void prop_lidar_send_record(bool on);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Write the module skeleton**

Create `main/prop_lidar.c`:

```c
/* prop_lidar — thin-client render link to lidar-roomscanner's /ws-thin endpoint.
 * See prop_lidar.h for the public API and docs/superpowers/specs/
 * 2026-08-17-lidar-thin-client-crowpanel-design.md for the protocol contract. */
#include "prop_lidar.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"

#define TAG "PROP_LIDAR"

#define FRAME_BYTES        (PROP_LIDAR_FRAME_PIXELS * 2)
#define STALE_MS           2000
#define RECONNECT_BACKOFF_MIN_MS 1000
#define RECONNECT_BACKOFF_MAX_MS 30000

/* ---- Cached state (written by the task, read by getters under s_lock) ---------- */
static SemaphoreHandle_t s_lock;

static uint16_t *s_frame_buf[2];   /* PSRAM double buffer, FRAME_BYTES each */
static int       s_frame_front;    /* index of the buffer readers should copy from */
static uint32_t  s_frame_seq;      /* increments each time a new frame lands */
static uint32_t  s_last_frame_ms;  /* esp_timer ms at the last complete frame */

static prop_lidar_telemetry_t s_telemetry;
static bool s_have_telemetry;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ---- Public getters -------------------------------------------------------------- */

bool prop_lidar_get_frame(uint16_t *dst, uint32_t *out_seq)
{
    if (!s_lock || !dst) return false;
    bool ok = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_frame_seq > 0) {
        memcpy(dst, s_frame_buf[s_frame_front], FRAME_BYTES);
        if (out_seq) *out_seq = s_frame_seq;
        ok = true;
    }
    xSemaphoreGive(s_lock);
    return ok;
}

void prop_lidar_get_telemetry(prop_lidar_telemetry_t *out)
{
    if (!out) return;
    if (!s_lock) {
        memset(out, 0, sizeof(*out));
        out->link = PROP_LIDAR_LINK_SEARCHING;
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_telemetry;
    if (out->link == PROP_LIDAR_LINK_OK && (now_ms() - s_last_frame_ms) > STALE_MS) {
        out->link = PROP_LIDAR_LINK_STALE;   /* computed live, not stored */
    }
    xSemaphoreGive(s_lock);
}

/* ---- Outbound commands (filled in by Task 6) -------------------------------------- */

void prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom)
{
    (void)dyaw; (void)dpitch; (void)dzoom;
    /* implemented in Task 6 */
}

void prop_lidar_send_mode(prop_lidar_mode_t mode)
{
    (void)mode;
    /* implemented in Task 6 */
}

void prop_lidar_send_record(bool on)
{
    (void)on;
    /* implemented in Task 6 */
}

/* ---- Background task (connect/reconnect logic filled in by Task 4) ---------------- */

static void lidar_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "task started (mDNS/WS connect logic pending — Task 4)");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_BACKOFF_MAX_MS));
    }
}

esp_err_t prop_lidar_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    for (int i = 0; i < 2; i++) {
        s_frame_buf[i] = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
        if (!s_frame_buf[i]) {
            ESP_LOGE(TAG, "PSRAM alloc failed for frame buffer %d (%u bytes)", i, (unsigned)FRAME_BYTES);
            return ESP_ERR_NO_MEM;
        }
        memset(s_frame_buf[i], 0, FRAME_BYTES);
    }
    s_frame_front = 0;
    s_frame_seq = 0;
    s_last_frame_ms = 0;

    memset(&s_telemetry, 0, sizeof(s_telemetry));
    s_telemetry.link = PROP_LIDAR_LINK_SEARCHING;
    s_have_telemetry = false;

    BaseType_t ok = xTaskCreate(lidar_task, "prop_lidar", 6144, NULL, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}
```

- [ ] **Step 3: Reconfigure (new `.c` file) and build**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Expected: clean build. `prop_lidar_init`/`_get_frame`/`_get_telemetry`/`_send_*` are not
called from anywhere yet, so no link errors; this step only proves the new module
compiles against the toolchain and headers.

- [ ] **Step 4: Commit**

```bash
git add main/include/prop_lidar.h main/prop_lidar.c
git commit -m "feat(lidar): add prop_lidar module skeleton"
```

---

## Task 4: mDNS resolution + WebSocket connect/reconnect lifecycle

**Files:**
- Modify: `main/prop_lidar.c`

**Interfaces:**
- Consumes: `mdns_query_ptr()` (from the already-present `espressif/mdns` dependency),
  `esp_websocket_client_init/start/stop/destroy/register_events()`.
- Produces: a running `lidar_task` that discovers the roomscanner host, connects, and
  retries with exponential backoff on failure — the foundation Tasks 5–6 hang their
  event-handler logic off of.

- [ ] **Step 1: Add mDNS resolution**

In `main/prop_lidar.c`, add near the top (after existing includes):

```c
#include "mdns.h"
#include "esp_websocket_client.h"
```

Add a resolver function above `lidar_task`:

```c
/* Resolve the roomscanner's advertised _roomscan._tcp service to "host:port". Returns
 * true and fills uri_out (e.g. "ws://192.168.4.55:8000/ws-thin") on success. */
static bool resolve_uri(char *uri_out, size_t uri_out_sz)
{
    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_ptr("_roomscan", "_tcp", 3000, 5, &results);
    if (err != ESP_OK || !results) {
        if (results) mdns_query_results_free(results);
        return false;
    }
    bool found = false;
    for (mdns_result_t *r = results; r && !found; r = r->next) {
        for (mdns_ip_addr_t *a = r->addr; a; a = a->next) {
            if (a->addr.type == ESP_IPADDR_TYPE_V4) {
                char ip[16];
                esp_ip4addr_ntoa(&a->addr.u_addr.ip4, ip, sizeof(ip));
                snprintf(uri_out, uri_out_sz, "ws://%s:%u/ws-thin", ip, (unsigned)r->port);
                found = true;
                break;
            }
        }
    }
    mdns_query_results_free(results);
    return found;
}
```

- [ ] **Step 2: Add the WebSocket client, event handler, and connect/reconnect loop**

Add module-scope state and the event handler above `lidar_task`:

```c
static esp_websocket_client_handle_t s_ws;
static EventGroupHandle_t s_evt;
#define LIDAR_EVT_DISCONNECTED (1 << 0)

static void set_link_state(prop_lidar_link_t link)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_telemetry.link = link;
    xSemaphoreGive(s_lock);
}

static void ws_event_handler(void *handler_args, esp_event_base_t base,
                              int32_t event_id, void *event_data)
{
    (void)handler_args; (void)base;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "ws connected");
            set_link_state(PROP_LIDAR_LINK_OK);
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGW(TAG, "ws disconnected/error (event %d)", (int)event_id);
            xEventGroupSetBits(s_evt, LIDAR_EVT_DISCONNECTED);
            break;
        case WEBSOCKET_EVENT_DATA:
            /* frame/telemetry parsing added in Tasks 5-6 */
            break;
        default:
            break;
    }
}
```

Replace the placeholder `lidar_task` body from Task 3 with:

```c
static void lidar_task(void *arg)
{
    (void)arg;
    s_evt = xEventGroupCreate();
    uint32_t backoff_ms = RECONNECT_BACKOFF_MIN_MS;

    for (;;) {
        char uri[96];
        set_link_state(PROP_LIDAR_LINK_SEARCHING);
        if (!resolve_uri(uri, sizeof(uri))) {
            ESP_LOGW(TAG, "mDNS: _roomscan._tcp not found, retrying in %u ms", (unsigned)backoff_ms);
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = backoff_ms * 2 < RECONNECT_BACKOFF_MAX_MS ? backoff_ms * 2 : RECONNECT_BACKOFF_MAX_MS;
            continue;
        }
        ESP_LOGI(TAG, "resolved %s", uri);

        esp_websocket_client_config_t cfg = {
            .uri = uri,
            .buffer_size = 4096,
            .network_timeout_ms = 8000,
        };
        s_ws = esp_websocket_client_init(&cfg);
        if (!s_ws) {
            ESP_LOGE(TAG, "esp_websocket_client_init failed");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            continue;
        }
        esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
        xEventGroupClearBits(s_evt, LIDAR_EVT_DISCONNECTED);
        esp_websocket_client_start(s_ws);

        backoff_ms = RECONNECT_BACKOFF_MIN_MS;   /* reset once a connect attempt is made */
        xEventGroupWaitBits(s_evt, LIDAR_EVT_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);

        esp_websocket_client_stop(s_ws);
        esp_websocket_client_destroy(s_ws);
        s_ws = NULL;
        set_link_state(PROP_LIDAR_LINK_SEARCHING);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```

Add the two needed FreeRTOS includes at the top alongside the existing ones:

```c
#include "freertos/event_groups.h"
```

- [ ] **Step 3: Reconfigure and build**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Expected: clean build.

- [ ] **Step 4: On-device verification against the mock server**

Wire the (not-yet-called) `prop_lidar_init()` in temporarily via a one-line call from
`app_main` for this manual check only if it isn't wired yet (Task 7 wires it for real via
`main.c`) — or, simpler, since Task 7 does the permanent wiring, skip flashing here and
instead confirm via log inspection once Task 7 lands. Note in the task-tracking system
that this step's real verification happens as part of Task 7 Step 4/5's on-device check
(mDNS resolve + WS connect logs), to avoid a throwaway temporary `main.c` edit.

- [ ] **Step 5: Commit**

```bash
git add main/prop_lidar.c
git commit -m "feat(lidar): mDNS resolution + WS connect/reconnect lifecycle"
```

---

## Task 5: `THIN_FRAME` reception into the PSRAM double buffer

**Files:**
- Modify: `main/prop_lidar.c`

**Interfaces:**
- Consumes: `WEBSOCKET_EVENT_DATA`'s `esp_websocket_event_data_t` (`op_code`, `data_ptr`,
  `data_len`, `payload_offset`, `payload_len`).
- Produces: `s_frame_seq`/`s_frame_front` now actually advance, making
  `prop_lidar_get_frame()` (from Task 3) return real data.

- [ ] **Step 1: Add a reassembly buffer and binary-frame handling**

Add module-scope state (near the other frame state):

```c
#define RX_HEADER_BYTES 8   /* u32 tag + u16 w + u16 h */
static uint8_t *s_rx_buf;      /* PSRAM, RX_HEADER_BYTES + FRAME_BYTES */
```

Allocate it in `prop_lidar_init()`, alongside the existing `s_frame_buf` allocation:

```c
    s_rx_buf = heap_caps_malloc(RX_HEADER_BYTES + FRAME_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_rx_buf) {
        ESP_LOGE(TAG, "PSRAM alloc failed for rx buffer");
        return ESP_ERR_NO_MEM;
    }
```

Extend `ws_event_handler`'s `WEBSOCKET_EVENT_DATA` case:

```c
        case WEBSOCKET_EVENT_DATA: {
            esp_websocket_event_data_t *d = (esp_websocket_event_data_t *)event_data;
            if (d->op_code == 0x2 /* binary */ &&
                d->data_len == (int)(RX_HEADER_BYTES + FRAME_BYTES) &&
                d->payload_offset + d->payload_len <= (int)(RX_HEADER_BYTES + FRAME_BYTES)) {
                memcpy(s_rx_buf + d->payload_offset, d->data_ptr, d->payload_len);
                if (d->payload_offset + d->payload_len == d->data_len) {
                    on_frame_complete(s_rx_buf, d->data_len);
                }
            } else if (d->op_code == 0x1 /* text */) {
                /* telemetry parsing added in Task 6 */
            }
            break;
        }
```

Add `on_frame_complete` above `ws_event_handler`:

```c
static void on_frame_complete(const uint8_t *buf, int len)
{
    if (len != (int)(RX_HEADER_BYTES + FRAME_BYTES)) return;
    uint32_t tag; uint16_t w, h;
    memcpy(&tag, buf + 0, 4);
    memcpy(&w,   buf + 4, 2);
    memcpy(&h,   buf + 6, 2);
    if (tag != 1 || w != PROP_LIDAR_FRAME_W || h != PROP_LIDAR_FRAME_H) {
        ESP_LOGW(TAG, "bad THIN_FRAME header tag=%u w=%u h=%u", (unsigned)tag, w, h);
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int back = 1 - s_frame_front;
    memcpy(s_frame_buf[back], buf + RX_HEADER_BYTES, FRAME_BYTES);
    s_frame_front = back;
    s_frame_seq++;
    s_last_frame_ms = now_ms();
    xSemaphoreGive(s_lock);
}
```

Free `s_rx_buf` is not needed at runtime (it lives for the program's lifetime, like the
frame buffers).

- [ ] **Step 2: Reconfigure and build**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add main/prop_lidar.c
git commit -m "feat(lidar): reassemble and cache THIN_FRAME binary frames"
```

---

## Task 6: `thin_telemetry` parsing + outbound commands

**Files:**
- Modify: `main/prop_lidar.c`

**Interfaces:**
- Consumes: cJSON (`cJSON_ParseWithLength`, `cJSON_GetObjectItem`,
  `cJSON_IsNumber/IsBool/IsString`), `esp_websocket_client_send_text()`.
- Produces: `prop_lidar_get_telemetry()` now returns live data; `prop_lidar_send_orbit/mode/record()`
  now actually send JSON commands.

- [ ] **Step 1: Parse `thin_telemetry`**

Add `#include "cJSON.h"` to the includes. Add a helper above `ws_event_handler`:

```c
static prop_lidar_mode_t mode_from_str(const char *s)
{
    if (!s) return PROP_LIDAR_MODE_POINT_CLOUD;
    if (strcmp(s, "slam") == 0) return PROP_LIDAR_MODE_SLAM;
    if (strcmp(s, "ir") == 0)   return PROP_LIDAR_MODE_IR;
    return PROP_LIDAR_MODE_POINT_CLOUD;
}

static void on_telemetry_json(const char *text, int len)
{
    cJSON *root = cJSON_ParseWithLength(text, len);
    if (!root) return;
    const cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(type) || strcmp(type->valuestring, "thin_telemetry") != 0) {
        cJSON_Delete(root);
        return;
    }
    prop_lidar_telemetry_t t = {0};
    t.link = PROP_LIDAR_LINK_OK;
    const cJSON *fps   = cJSON_GetObjectItem(root, "fps");
    const cJSON *pw    = cJSON_GetObjectItem(root, "power_mode");
    const cJSON *i3c   = cJSON_GetObjectItem(root, "i3c_airtime_pct");
    const cJSON *pts   = cJSON_GetObjectItem(root, "point_count");
    const cJSON *rec   = cJSON_GetObjectItem(root, "recording");
    const cJSON *mode  = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsNumber(fps)) t.fps = (float)fps->valuedouble;
    if (cJSON_IsString(pw))  snprintf(t.power_mode, sizeof(t.power_mode), "%s", pw->valuestring);
    if (cJSON_IsNumber(i3c)) t.i3c_airtime_pct = (float)i3c->valuedouble;
    if (cJSON_IsNumber(pts)) t.point_count = pts->valueint;
    if (cJSON_IsBool(rec))   t.recording = cJSON_IsTrue(rec);
    if (cJSON_IsString(mode)) t.mode = mode_from_str(mode->valuestring);
    cJSON_Delete(root);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_telemetry = t;
    s_have_telemetry = true;
    xSemaphoreGive(s_lock);
}
```

Wire it into the `WEBSOCKET_EVENT_DATA` text branch from Task 5:

```c
            } else if (d->op_code == 0x1 /* text */ &&
                       d->payload_offset + d->payload_len == d->data_len) {
                on_telemetry_json(d->data_ptr, d->payload_len);
```

(Small JSON messages arrive as a single event in practice, so this doesn't need the
reassembly buffer Task 5 added for binary frames — it only acts once the final chunk of a
given text frame is seen.)

- [ ] **Step 2: Implement the outbound command senders**

Replace the three placeholder functions from Task 3:

```c
static void send_text_if_connected(const char *json)
{
    if (!s_ws || !esp_websocket_client_is_connected(s_ws)) return;
    esp_websocket_client_send_text(s_ws, json, (int)strlen(json), pdMS_TO_TICKS(200));
}

void prop_lidar_send_orbit(float dyaw, float dpitch, float dzoom)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"thin_orbit\",\"dyaw\":%.3f,\"dpitch\":%.3f,\"dzoom\":%.3f}",
             dyaw, dpitch, dzoom);
    send_text_if_connected(buf);
}

static const char *mode_to_str(prop_lidar_mode_t m)
{
    switch (m) {
        case PROP_LIDAR_MODE_SLAM: return "slam";
        case PROP_LIDAR_MODE_IR:   return "ir";
        default:                   return "point_cloud";
    }
}

void prop_lidar_send_mode(prop_lidar_mode_t mode)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"type\":\"thin_mode\",\"mode\":\"%s\"}", mode_to_str(mode));
    send_text_if_connected(buf);
}

void prop_lidar_send_record(bool on)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"type\":\"thin_record\",\"on\":%s}", on ? "true" : "false");
    send_text_if_connected(buf);
}
```

- [ ] **Step 3: Reconfigure and build**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add main/prop_lidar.c
git commit -m "feat(lidar): parse thin_telemetry and implement outbound commands"
```

---

## Task 7: Wire `prop_lidar_init()` into boot, add `BOOT_STAGE_LIDAR`

**Files:**
- Modify: `main/include/prop_bootlog.h`
- Modify: `main/prop_bootlog.c`
- Modify: `main/main.c`

**Interfaces:**
- Consumes: `prop_lidar_init()` (Task 3).
- Produces: the module is actually running on real hardware from here on, letting every
  remaining task be checked against the mock server from Task 2.

- [ ] **Step 1: Add the boot stage**

In `main/include/prop_bootlog.h`, add `BOOT_STAGE_LIDAR,` to the enum right after
`BOOT_STAGE_FTM,` (before `BOOT_STAGE_READY,`):

```c
    BOOT_STAGE_FTM,
    BOOT_STAGE_LIDAR,
    BOOT_STAGE_READY,
```

In `main/prop_bootlog.c`, add the matching name entry after `[BOOT_STAGE_FTM] = "ftm",`:

```c
    [BOOT_STAGE_FTM]         = "ftm",
    [BOOT_STAGE_LIDAR]       = "lidar",
```

- [ ] **Step 2: Call `prop_lidar_init()` from `main.c`**

In `main/main.c`, add `#include "prop_lidar.h"` to the includes, and after the existing
`prop_ftm_init()` block (inside the `if (net_err == ESP_OK) { ... }` block, since this
needs WiFi + `mdns_init()` already up from `prop_net_init()`):

```c
        /* LiDAR thin-client render link (LIDAR panel) — connects out to the
         * lidar-roomscanner rig's /ws-thin endpoint via mDNS. NON-fatal: if the task
         * can't start, the panel just shows LINK: SEARCHING forever. */
        prop_bootlog_mark(BOOT_STAGE_LIDAR);
        esp_err_t lidar_err = prop_lidar_init();
        if (lidar_err != ESP_OK) {
            MAIN_ERROR("LiDAR link unavailable (%s) — LIDAR panel will show no data",
                       esp_err_to_name(lidar_err));
        }
```

- [ ] **Step 3: Reconfigure and build**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Expected: clean build.

- [ ] **Step 4: Flash and check the boot log**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash monitor
```

(Confirm the actual port first with `[System.IO.Ports.SerialPort]::GetPortNames()` per
`CLAUDE.md`.) With the mock server from Task 2 **not yet running** on the network,
expect repeated `PROP_LIDAR: mDNS: _roomscan._tcp not found, retrying in ... ms` log
lines — confirms the task is alive and backing off correctly. Ctrl-] to exit monitor.

- [ ] **Step 5: Verify a real connection against the mock server**

The mock server from Task 2 doesn't advertise itself over mDNS (that's out of scope for
this repo — it's the companion `lidar-roomscanner` spec's job). For this on-device check,
temporarily point the resolver at a fixed host instead of mDNS by running the mock server
and confirming manually that a plain TCP client (any machine on the same LAN as the
board) can reach `ws://<mock-server-ip>:8000/ws-thin` — this validates the mock server
itself is reachable, but full mDNS-driven auto-connect from the CrowPanel can only be
proven once the real `lidar-roomscanner` `/ws-thin` + mDNS advertisement work (companion
spec) lands. Note this explicitly as a known integration gap to close in Task 9's
end-to-end check, and re-verify then.

- [ ] **Step 6: Commit**

```bash
git add main/include/prop_bootlog.h main/prop_bootlog.c main/main.c
git commit -m "feat(lidar): wire prop_lidar_init into boot sequence"
```

---

## Task 8: `PK_LIDAR` panel — canvas render + telemetry strip

**Files:**
- Modify: `main/prop_ui.c`

**Interfaces:**
- Consumes: `prop_lidar_get_frame()`, `prop_lidar_get_telemetry()` (Tasks 3–6).
- Produces: a visible, populated `PK_LIDAR` panel reachable via `goto lidar` — no
  controls yet (Task 9 adds those).

- [ ] **Step 1: Add the `PK_LIDAR` enum value and forward declaration**

In the `panel_kind_t` enum (near `PK_RANGE`), add:

```c
    PK_RANGE,         /* WiFi FTM ranging table (per-BSSID distance-to-AP) */
    PK_LIDAR,         /* thin-client render of the lidar-roomscanner rig's live view */
```

Near the other `build_*_panel` forward declarations (alongside
`static lv_obj_t *build_minimap_panel(lv_obj_t *parent);`), add:

```c
static lv_obj_t *build_lidar_panel(lv_obj_t *parent);
```

Add `#include "prop_lidar.h"` to `prop_ui.c`'s includes.

- [ ] **Step 2: Add module-scope widget statics**

Near the other panel-specific statics (e.g. alongside the MINIMAP block:
`s_map_canvas`, etc.), add:

```c
/* PK_LIDAR: thin-client render + telemetry strip. */
static lv_obj_t *s_lidar_canvas;
static void     *s_lidar_canvas_buf;    /* PSRAM, LV_CANVAS_BUF_SIZE(480,480,16,...) */
static uint16_t *s_lidar_frame_scratch; /* PSRAM, PROP_LIDAR_FRAME_PIXELS uint16_t's */
static uint32_t  s_lidar_last_seq;      /* last frame seq blitted, to skip redundant copies */
static lv_obj_t *s_lidar_fps;
static lv_obj_t *s_lidar_power;
static lv_obj_t *s_lidar_i3c;
static lv_obj_t *s_lidar_pts;
static lv_obj_t *s_lidar_rec;
static lv_obj_t *s_lidar_link;
```

- [ ] **Step 3: Write `build_lidar_panel`**

Add near `build_minimap_panel`:

```c
#define LIDAR_CANVAS_SZ 480

static lv_obj_t *build_lidar_panel(lv_obj_t *parent)
{
    /* Full-screen hero panel, no title header / BACK button — same bare-bordered-
     * container idiom as SCANNER (build_motion_panel). */
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_size(p, SCAN_W, 600);
    lv_obj_align(p, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(p, COL_BG, 0);
    lv_obj_set_style_border_color(p, COL_AMBER, 0);
    lv_obj_set_style_border_width(p, 2, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);

    /* Render canvas: native RGB565 (swap_bytes=false matches the panel's display
     * config — no ARGB, this is a straight frame blit, not hand-painted graphics). */
    size_t canvas_sz = LV_CANVAS_BUF_SIZE(LIDAR_CANVAS_SZ, LIDAR_CANVAS_SZ, 16, LV_DRAW_BUF_STRIDE_ALIGN);
    s_lidar_canvas_buf = heap_caps_malloc(canvas_sz, MALLOC_CAP_SPIRAM);
    if (s_lidar_canvas_buf) {
        memset(s_lidar_canvas_buf, 0, canvas_sz);
        s_lidar_canvas = lv_canvas_create(p);
        lv_canvas_set_buffer(s_lidar_canvas, s_lidar_canvas_buf, LIDAR_CANVAS_SZ, LIDAR_CANVAS_SZ,
                              LV_COLOR_FORMAT_RGB565);
        lv_obj_set_pos(s_lidar_canvas, 24, 24);
        lv_obj_set_style_border_color(s_lidar_canvas, COL_DIM, 0);
        lv_obj_set_style_border_width(s_lidar_canvas, 1, 0);
    }
    s_lidar_frame_scratch = heap_caps_malloc(PROP_LIDAR_FRAME_PIXELS * 2, MALLOC_CAP_SPIRAM);
    s_lidar_last_seq = 0;

    /* Telemetry strip, right of the canvas. */
    lv_coord_t strip_x = 24 + LIDAR_CANVAS_SZ + 24;
    lv_obj_t *strip = lv_obj_create(p);
    lv_obj_set_size(strip, SCAN_W - strip_x - 24, LIDAR_CANVAS_SZ);
    lv_obj_set_pos(strip, strip_x, 24);
    lv_obj_set_style_bg_color(strip, COL_PANEL_ITEM, 0);
    lv_obj_set_style_bg_opa(strip, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(strip, COL_DIM, 0);
    lv_obj_set_style_border_width(strip, 1, 0);
    lv_obj_set_style_radius(strip, 0, 0);
    lv_obj_set_style_pad_all(strip, 12, 0);
    lv_obj_clear_flag(strip, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(strip);
    lv_label_set_text(title, "LIDAR LINK");
    lv_obj_set_style_text_font(title, FONT_HEAD, 0);
    lv_obj_set_style_text_color(title, COL_AMBER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

    static const char *kNames[] = { "LINK", "FPS", "POWER", "I3C BUS", "POINTS", "REC" };
    lv_obj_t **kSlots[] = { &s_lidar_link, &s_lidar_fps, &s_lidar_power, &s_lidar_i3c,
                             &s_lidar_pts, &s_lidar_rec };
    for (int i = 0; i < 6; i++) {
        lv_obj_t *lbl = lv_label_create(strip);
        lv_label_set_text(lbl, kNames[i]);
        lv_obj_set_style_text_font(lbl, FONT_BODY, 0);
        lv_obj_set_style_text_color(lbl, COL_MUTE, 0);
        lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 40 + i * 32);

        lv_obj_t *val = lv_label_create(strip);
        lv_label_set_text(val, "--");
        lv_obj_set_style_text_font(val, FONT_BODY, 0);
        lv_obj_set_style_text_color(val, COL_AMBER, 0);
        lv_obj_align(val, LV_ALIGN_TOP_LEFT, 140, 40 + i * 32);
        *kSlots[i] = val;
    }

    return p;
}
```

- [ ] **Step 4: Wire `open_panel`, `close_panel`, `rail_sync`, deep-link, and the SENSORS menu row**

In `open_panel`'s switch, add after the `PK_RANGE` case:

```c
        case PK_RANGE:    s_cur_panel = build_range_panel(s_root); break;
        case PK_LIDAR:    s_cur_panel = build_lidar_panel(s_root); break;
```

In `close_panel`, add (grouped near the MINIMAP cleanup at the end):

```c
    /* PK_LIDAR: widgets freed with the panel; its canvas + scratch PSRAM are ours. */
    s_lidar_canvas = NULL;
    if (s_lidar_canvas_buf) { free(s_lidar_canvas_buf); s_lidar_canvas_buf = NULL; }
    if (s_lidar_frame_scratch) { free(s_lidar_frame_scratch); s_lidar_frame_scratch = NULL; }
    s_lidar_last_seq = 0;
    s_lidar_fps = s_lidar_power = s_lidar_i3c = s_lidar_pts = s_lidar_rec = s_lidar_link = NULL;
```

In `rail_sync`'s switch, add `PK_LIDAR` to the SENSORS group line:

```c
        case PK_RFBAND: case PK_BLE: case PK_CSI: case PK_CSICFG: case PK_CSISET: case PK_MOTION:
        case PK_DIRCAL: case PK_MINIMAP: case PK_RANGE: case PK_LIDAR:
            want = PK_SENSORS; break;
```

In `build_sensors_panel`, add a row after the RANGE row:

```c
    kit_list_row(b, "RANGE",   menu_open_cb, (void *)(intptr_t)PK_RANGE);
    kit_list_row(b, "LIDAR",   menu_open_cb, (void *)(intptr_t)PK_LIDAR);
```

In `prop_ui_goto`, add after the `"range"` branch:

```c
    else if (strcmp(screen, "range") == 0)       open_panel(PK_RANGE);
    else if (strcmp(screen, "lidar") == 0)       open_panel(PK_LIDAR);
```

In `prop_ui_current_screen`, add a matching case (grep for `case PK_MOTION:      return "motion";`
and add immediately after the sibling cases in that block):

```c
        case PK_LIDAR:       return "lidar";
```

- [ ] **Step 5: Wire the render/telemetry refresh into `ui_observer`**

Locate the `if (s_cur_kind == PK_DIRCAL && s_dcal_prompt) { ... }` block inside
`ui_observer` (search for that exact condition) and add a new block immediately after it,
before the function's closing `lvgl_port_unlock();`:

```c
    /* PK_LIDAR: blit the newest thin-client frame + refresh the telemetry strip. Both
     * are cheap cache reads (prop_lidar owns the actual network I/O off this task). */
    if (s_cur_kind == PK_LIDAR && s_lidar_canvas && s_lidar_frame_scratch) {
        uint32_t seq = 0;
        if (prop_lidar_get_frame(s_lidar_frame_scratch, &seq) && seq != s_lidar_last_seq) {
            lv_draw_buf_t *db = lv_canvas_get_draw_buf(s_lidar_canvas);
            uint8_t *dst = db->data;
            uint32_t stride = db->header.stride;
            const uint8_t *src = (const uint8_t *)s_lidar_frame_scratch;
            for (int y = 0; y < PROP_LIDAR_FRAME_H; y++) {
                memcpy(dst + (size_t)y * stride, src + (size_t)y * PROP_LIDAR_FRAME_W * 2,
                       PROP_LIDAR_FRAME_W * 2);
            }
            lv_obj_invalidate(s_lidar_canvas);
            s_lidar_last_seq = seq;
        }

        prop_lidar_telemetry_t t;
        prop_lidar_get_telemetry(&t);
        static const char *kLinkText[] = { "SEARCHING", "OK", "STALE" };
        label_set_text_cached(s_lidar_link, kLinkText[t.link]);
        lv_obj_set_style_text_color(s_lidar_link,
            t.link == PROP_LIDAR_LINK_OK ? COL_AMBER : COL_ALERT, 0);

        char buf[24];
        snprintf(buf, sizeof(buf), "%.1f", t.fps);
        label_set_text_cached(s_lidar_fps, buf);
        label_set_text_cached(s_lidar_power, t.power_mode[0] ? t.power_mode : "--");
        snprintf(buf, sizeof(buf), "%.0f%%", t.i3c_airtime_pct);
        label_set_text_cached(s_lidar_i3c, buf);
        snprintf(buf, sizeof(buf), "%d", t.point_count);
        label_set_text_cached(s_lidar_pts, buf);
        label_set_text_cached(s_lidar_rec, t.recording ? "REC" : "off");
        lv_obj_set_style_text_color(s_lidar_rec, t.recording ? COL_ALERT : COL_MUTE, 0);
    }
```

(Uses the repo's `lv_label_set_text_fmt`-uses-no-`%f` gotcha correctly: `snprintf` into a
local buffer, then `label_set_text_cached`, matching the pattern documented in
`CLAUDE.md`'s LVGL section.)

- [ ] **Step 6: Reconfigure, build, flash, and screenshot**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash
```

With `tools/mock_ws_thin.py` running and reachable (see Task 9 for closing the mDNS gap —
for this step it's enough that the panel *builds and displays*, even while
`LINK: SEARCHING`):

```bash
python tools/prop.py shot lidar.png --screen lidar --wait
```

Expected: the screenshot shows the bordered hero panel, an empty/black 480×480 canvas,
and a telemetry strip reading `LINK SEARCHING`, `FPS --`, etc. — confirms the panel
builds, opens via deep-link, and doesn't crash.

- [ ] **Step 7: Commit**

```bash
git add main/prop_ui.c
git commit -m "feat(lidar): add PK_LIDAR panel with canvas render + telemetry strip"
```

---

## Task 9: Controls — orbit (touch/dial), mode cycle (tab), record toggle

**Files:**
- Modify: `main/prop_ui.c`

**Interfaces:**
- Consumes: `prop_lidar_send_orbit/mode/record()` (Task 6).
- Produces: the panel is now fully interactive per the spec's control set.

- [ ] **Step 1: Touch-drag orbit on the canvas**

In `build_lidar_panel`, after creating `s_lidar_canvas`, make it interactive and add a
drag handler:

```c
    lv_obj_add_flag(s_lidar_canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_lidar_canvas, lidar_canvas_pressing_cb, LV_EVENT_PRESSING, NULL);
```

Add the callback above `build_lidar_panel`:

```c
#define LIDAR_DRAG_DEG_PER_PX 0.5f

static void lidar_canvas_pressing_cb(lv_event_t *e)
{
    (void)e;
    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;
    lv_point_t v;
    lv_indev_get_vect(indev, &v);
    if (v.x == 0 && v.y == 0) return;
    prop_lidar_send_orbit((float)v.x * LIDAR_DRAG_DEG_PER_PX,
                           (float)-v.y * LIDAR_DRAG_DEG_PER_PX, 0.0f);
}
```

- [ ] **Step 2: Dial orbit**

In `nav_select_move` (search for `static void nav_select_move(int dir)`), add a case:

```c
#define LIDAR_DIAL_STEP_DEG 5.0f

        case PK_LIDAR:
            prop_lidar_send_orbit((float)dir * LIDAR_DIAL_STEP_DEG, 0.0f, 0.0f);
            break;
```

(Add this `case PK_LIDAR:` branch inside the existing `switch (s_cur_kind)` in that
function, alongside `PK_HOME`/`PK_ARCHIVE`/`PK_ARTICLE`.)

- [ ] **Step 3: Tab cycles view mode**

In `nav_tab` (search for `static void nav_tab(int n)`), add a guard at the top, before
its existing body:

```c
static void nav_tab(int n)
{
    if (s_cur_kind == PK_LIDAR) {
        prop_lidar_telemetry_t t;
        prop_lidar_get_telemetry(&t);
        prop_lidar_send_mode((prop_lidar_mode_t)((t.mode + 1) % 3));
        return;
    }
    s_archive_section = clamp_section(n);
    s_archive_entry = 0;
    open_panel(PK_ARCHIVE);
}
```

- [ ] **Step 4: Record toggle button**

In `build_lidar_panel`, add a themed button to the telemetry strip (after the 6-row
loop), mirroring `scan_spk_toggle_cb`'s icon-swap-by-state idiom:

```c
    lv_obj_t *rec_btn = make_btn(strip, LV_SYMBOL_VIDEO " REC", 160, LV_ALIGN_TOP_LEFT,
                                  0, LIDAR_CANVAS_SZ - 60, lidar_record_toggle_cb);
    (void)rec_btn;
```

Add the callback above `build_lidar_panel`:

```c
static void lidar_record_toggle_cb(lv_event_t *e)
{
    (void)e;
    prop_lidar_telemetry_t t;
    prop_lidar_get_telemetry(&t);
    prop_lidar_send_record(!t.recording);
}
```

- [ ] **Step 5: Reconfigure and build**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Expected: clean build.

- [ ] **Step 6: Commit**

```bash
git add main/prop_ui.c
git commit -m "feat(lidar): wire orbit/mode/record controls on PK_LIDAR"
```

---

## Task 10: End-to-end on-device verification + docs

**Files:**
- Modify: `CLAUDE.md`
- Modify: `AGENTS.md`
- (No `.claude/CLAUDE.md` edit — it's a symlink to `../AGENTS.md`.)

**Interfaces:**
- Consumes: everything from Tasks 1–9.
- Produces: closes the mDNS gap noted in Task 7 Step 5 by using the mock server's fixed
  IP directly (proving the full protocol path), and documents the new screen for future
  sessions.

- [ ] **Step 1: Update the screen-name lists**

In `CLAUDE.md`'s "API quick reference" section, find the line listing screen names
(`screens: home=console, scanner archive cassette insights menu wifi display audio leds
vitals scan spectrum rfband ble csi instruments sensors dircal minimap range about`) and
add `lidar` to it. Make the identical edit in `AGENTS.md` (same line, same section — the
two files are kept in sync manually per the repo's existing convention;
`.claude/CLAUDE.md` is a symlink to `AGENTS.md` and needs no separate edit).

- [ ] **Step 2: Run the mock server reachable from the board's network**

On a machine on the same LAN/WiFi as the CrowPanel, run:

```bash
python tools/mock_ws_thin.py --port 8000
```

Since the mock server doesn't advertise mDNS (that's the companion `lidar-roomscanner`
spec's job), temporarily hardcode its IP in `resolve_uri()` for this verification pass
only — e.g. change the function body to `snprintf(uri_out, uri_out_sz,
"ws://<mock-server-ip>:8000/ws-thin"); return true;` — reconfigure/build/flash, run the
check below, then **revert** that temporary edit before the final commit (real mDNS
discovery is the shipped behavior; this is a one-off proof that the wire protocol works
end-to-end).

- [ ] **Step 3: Flash and verify**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash
```

```bash
python tools/prop.py shot lidar_live.png --screen lidar --wait
```

Expected: the screenshot shows the animated diagonal-band test pattern from
`mock_ws_thin.py`'s `_make_frame` in the canvas, and telemetry reading `LINK OK`,
`FPS 10.0`, `POWER ULP`, `I3C BUS 36%`, `POINTS 2268`, `REC off`.

```bash
python tools/prop.py watch --only lidar --count 5
```

(If `--only` filtering doesn't apply to a non-`/telemetry` custom field — this repo's
`/telemetry` endpoint doesn't carry LiDAR fields, since those live only in
`prop_lidar`'s internal cache — instead just re-run the `shot` command after pressing the
on-screen REC button and the dial, and confirm via a second screenshot that `REC`
switched to the alert color and the mock server's stdout logged the `thin_record`/`thin_orbit`
JSON it received.)

- [ ] **Step 4: Revert the temporary hardcoded-IP edit from Step 2**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
idf.py -C "f:\git\personal\CrowPanelProp" build
```

Confirm `main/prop_lidar.c`'s `resolve_uri()` is back to the mDNS-based implementation
from Task 4 (`git diff main/prop_lidar.c` should show no changes at this point — Steps 2–3
never got committed).

- [ ] **Step 5: Commit the doc updates**

```bash
git add CLAUDE.md AGENTS.md
git commit -m "docs: add lidar to the CrowPanel screen-name reference"
```
