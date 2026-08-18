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


# The writer thread and the reader thread (PONG replies) both write to the same socket;
# a single sendall() per frame under this lock keeps them from interleaving mid-frame.
_SEND_LOCK = threading.Lock()


def _send_ws_frame(conn: socket.socket, payload: bytes, opcode: int) -> None:
    fin_op = 0x80 | opcode
    length = len(payload)
    if length < 126:
        header = struct.pack("!BB", fin_op, length)
    elif length < 65536:
        header = struct.pack("!BBH", fin_op, 126, length)
    else:
        header = struct.pack("!BBQ", fin_op, 127, length)
    with _SEND_LOCK:
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
        if opcode == 0x9:      # ping -> must PONG the same payload back, or
            try:               # esp_websocket_client drops the session on
                _send_ws_frame(conn, payload, opcode=0xA)   # pingpong_timeout_sec
            except OSError:
                break
            continue
        if opcode == 0xA:      # pong (we never ping, but tolerate it)
            continue
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
