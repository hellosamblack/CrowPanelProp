#!/usr/bin/env python3
"""prop.py - one CLI for driving + inspecting the communicator prop during dev.

Collapses the build/flash -> wait -> drive UI -> screenshot loop (and crash
decoding) into single commands, so iterating on the repo root is fast.

Networking is stdlib-only (reaches the prop at its mDNS name comm-unit-7.local).
`trace` additionally needs pyserial; `trace`/`decode` need the build/ ELF.

Examples:
    python prop.py wait                         # block until the API answers
    python prop.py state                        # pretty-print GET /state
    python prop.py scene SIGNAL_ACQUIRED        # POST /cmd scene
    python prop.py screen spectrum              # navigate the touchscreen UI
    python prop.py sens 80                       # receiver sensitivity
    python prop.py fx on 70                       # CRT overlay on at intensity 70
    python prop.py shot out.png --screen spectrum --wait   # drive + settle + capture
    python prop.py shot corner.png --crop 764,8,260,90 --zoom 3
    python prop.py trace --seconds 12            # capture serial, decode any panic/WDT
    python prop.py decode 0x40034286 0x40034206  # addr2line against the ELF
    python prop.py telemetry                     # one-shot GET /telemetry snapshot
    python prop.py watch                         # live telemetry stream (Ctrl-C to stop)
    python prop.py watch --only imu.yaw_deg,radar --count 20

Screens: home menu wifi display audio leds vitals scan spectrum about
Scenes:  IDLE SCANNING SIGNAL_ACQUIRED COMMS ALERT
"""
import sys
import os
import json
import time
import glob
import re
import struct
import socket
import base64
import subprocess
import urllib.request

import screenshot  # reuse the RGB565->PNG converter living next to this file

DEFAULT_HOST = "comm-unit-7.local"
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
ELF = os.path.join(PROJ, "build", "communicator.elf")


# ---- HTTP helpers --------------------------------------------------------
def _post_cmd(host, obj):
    body = json.dumps(obj).encode()
    req = urllib.request.Request(f"http://{host}/cmd", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=8) as r:
        return r.read().decode(errors="replace")


def _get_state(host):
    with urllib.request.urlopen(f"http://{host}/state", timeout=5) as r:
        return json.loads(r.read().decode())


def _get_telemetry(host):
    with urllib.request.urlopen(f"http://{host}/telemetry", timeout=5) as r:
        return json.loads(r.read().decode())


def wait_ready(host, timeout=90):
    """Poll /state until the API answers (after a flash/reset). Returns True/False."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            _get_state(host)
            return True
        except Exception:
            time.sleep(2)
    return False


# ---- live telemetry (minimal stdlib WebSocket client) --------------------
# The board pushes a fresh telemetry JSON object (type:"telemetry") to every
# /ws client ~5x/sec (see prop_api.c telemetry_task) -- IMU orientation, LD2450
# radar targets, PDR pose, mic level, aux presence sensors, BLE/CSI summaries.
# This lets `watch` show live variable values instead of eyeballing screenshots.
# No third-party ws library needed for a receive-only client: handshake once,
# then read server->client frames (unmasked, per RFC 6455) off the raw socket.
def _ws_connect(host, path="/ws", timeout=10):
    hostname, _, port = host.partition(":")
    sock = socket.create_connection((hostname, int(port) if port else 80), timeout=timeout)
    key = base64.b64encode(os.urandom(16)).decode()
    req = (f"GET {path} HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
    sock.sendall(req.encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(1024)
        if not chunk:
            raise ConnectionError("ws handshake: connection closed")
        buf += chunk
    header, _, rest = buf.partition(b"\r\n\r\n")
    if b" 101 " not in header.split(b"\r\n", 1)[0]:
        raise ConnectionError(f"ws handshake failed: {header!r}")
    return sock, rest


def _ws_frames(sock, leftover):
    """Yield (opcode, payload) for each frame the server sends, forever."""
    buf = bytearray(leftover)

    def fill(n):
        while len(buf) < n:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("ws stream closed")
            buf.extend(chunk)

    while True:
        fill(2)
        opcode = buf[0] & 0x0F
        masked = buf[1] & 0x80
        plen = buf[1] & 0x7F
        hlen = 2
        if plen == 126:
            fill(4)
            plen = struct.unpack(">H", buf[2:4])[0]
            hlen = 4
        elif plen == 127:
            fill(10)
            plen = struct.unpack(">Q", buf[2:10])[0]
            hlen = 10
        mask = None
        if masked:
            fill(hlen + 4)
            mask = buf[hlen:hlen + 4]
            hlen += 4
        fill(hlen + plen)
        payload = bytearray(buf[hlen:hlen + plen])
        if mask:
            for i in range(len(payload)):
                payload[i] ^= mask[i % 4]
        del buf[:hlen + plen]
        yield opcode, bytes(payload)


def watch_stream(host, path="/ws"):
    """Generator of parsed JSON messages pushed over /ws (state + telemetry)."""
    sock, leftover = _ws_connect(host, path)
    try:
        for opcode, payload in _ws_frames(sock, leftover):
            if opcode == 0x8:      # close
                break
            if opcode == 0x1:      # text
                yield json.loads(payload.decode())
            # ping (0x9) / pong (0xA): the board never sends pings today; ignore.
    finally:
        sock.close()


def _dig(obj, dotted_path):
    cur = obj
    for part in dotted_path.split("."):
        cur = cur.get(part) if isinstance(cur, dict) else None
    return cur


def watch(host, only=None, raw=False, count=0, msg_type="telemetry"):
    n = 0
    try:
        for msg in watch_stream(host):
            if msg_type and msg.get("type") != msg_type:
                continue
            shown = {k: _dig(msg, k) for k in only} if only else msg
            if raw:
                print(json.dumps(shown))
            else:
                print(f"[{time.strftime('%H:%M:%S')}] {json.dumps(shown)}")
            n += 1
            if count and n >= count:
                break
    except KeyboardInterrupt:
        pass


# ---- crash decoding ------------------------------------------------------
def _addr2line_exe():
    pats = [
        r"C:\Espressif\tools\riscv32-esp-elf\*\riscv32-esp-elf\bin\riscv32-esp-elf-addr2line.exe",
        os.path.expanduser(
            "~/.espressif/tools/riscv32-esp-elf/*/riscv32-esp-elf/bin/riscv32-esp-elf-addr2line"),
    ]
    for p in pats:
        hits = glob.glob(p)
        if hits:
            return hits[0]
    return None


def decode(addrs):
    if not os.path.exists(ELF):
        raise SystemExit(f"ELF not found: {ELF} (build first)")
    exe = _addr2line_exe()
    if not exe:
        raise SystemExit("riscv32-esp-elf-addr2line not found under C:\\Espressif or ~/.espressif")
    out = subprocess.run([exe, "-f", "-C", "-e", ELF, *addrs],
                         capture_output=True, text=True)
    lines = [l for l in out.stdout.splitlines() if l.strip()]
    for i, a in enumerate(addrs):
        fn = lines[2 * i] if 2 * i < len(lines) else "??"
        loc = lines[2 * i + 1] if 2 * i + 1 < len(lines) else "??"
        print(f"  {a}  {fn}  @ {loc}")


_ADDR_RE = re.compile(r"\b(0x4[0-9a-fA-F]{7})\b")
_FLAG_RE = re.compile(r"mempool|no mem|assert failed|Guru Meditation|abort|"
                      r"watchdog|StackCanaryCheck|LoadProhibited|StoreProhibited",
                      re.IGNORECASE)


def trace(port, seconds):
    """Capture serial (opening the port resets the board -> fresh boot), print
    flagged lines, and addr2line-decode any code addresses in panic/WDT dumps."""
    try:
        import serial  # pyserial
    except ImportError:
        raise SystemExit("pyserial not installed (pip install pyserial)")
    s = serial.Serial(port, 115200, timeout=0.3)
    print(f"[trace] capturing {seconds}s on {port} (board reset on open)...")
    end = time.time() + seconds
    flagged, addrs = [], []
    while time.time() < end:
        line = s.readline().decode(errors="replace").rstrip()
        if not line:
            continue
        if _FLAG_RE.search(line):
            flagged.append(line)
            print("  !!", line)
        for m in _ADDR_RE.findall(line):
            if m not in addrs:
                addrs.append(m)
    s.close()
    if flagged and addrs:
        print("\n[trace] decoding code addresses seen during the fault:")
        decode(addrs[:12])
    elif not flagged:
        print("[trace] no panic/WDT/mempool markers seen (clean run?)")


# ---- shot (drive + settle + capture) -------------------------------------
def shot(host, out, screen=None, scene=None, do_wait=False, crop=None, zoom=1, settle=1.5):
    if do_wait and not wait_ready(host):
        raise SystemExit("device never became ready")
    if scene:
        _post_cmd(host, {"cmd": "scene", "value": scene})
    if screen:
        _post_cmd(host, {"cmd": "ui", "screen": screen})
    if scene or screen:
        time.sleep(settle)
    w, h, data = screenshot.fetch(f"http://{host}/screenshot")
    raw = screenshot.rgb565le_to_rgb888_rows(data, w, h)
    if crop or zoom > 1:
        ow, oh, raw = screenshot.crop_rgb888_rows(raw, w, h, crop or (0, 0, w, h), zoom)
        screenshot.write_png(out, ow, oh, raw)
        print(f"saved {out} ({ow}x{oh} from {w}x{h})")
    else:
        screenshot.write_png(out, w, h, raw)
        print(f"saved {out} ({w}x{h})")


# ---- arg plumbing --------------------------------------------------------
def _pop_opt(args, name):
    if name in args:
        i = args.index(name)
        val = args[i + 1]
        del args[i:i + 2]
        return val
    return None


def detect_port():
    # 1. Windows: Query via PowerShell Get-CimInstance for CH340 / CH341
    if os.name == "nt":
        try:
            import subprocess
            cmd = 'Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -like "*CH340*" -or $_.Caption -like "*CH340*" -or $_.Name -like "*CH341*" } | Select-Object -First 1 -ExpandProperty Name'
            out = subprocess.run(["powershell.exe", "-NoProfile", "-Command", cmd], capture_output=True, text=True, timeout=5)
            m = re.search(r'\((COM\d+)\)', out.stdout)
            if m:
                return m.group(1)
        except Exception:
            pass
        return "COM7"

    # 2. Native Linux (and usbipd in WSL): Check /dev/serial/by-id for 1a86 / CH340 / CH341
    by_id_dir = "/dev/serial/by-id"
    if os.path.exists(by_id_dir):
        for name in os.listdir(by_id_dir):
            if any(k in name.lower() for k in ["1a86", "ch340", "ch341"]):
                target = os.path.realpath(os.path.join(by_id_dir, name))
                if os.path.exists(target):
                    return target

    # 3. Native Linux (and usbipd in WSL): Check /sys/class/tty/ttyUSB* or ttyACM* devices via sysfs (vendor ID 1a86)
    import glob
    for path in glob.glob("/sys/class/tty/ttyUSB*") + glob.glob("/sys/class/tty/ttyACM*"):
        try:
            device_path = os.path.realpath(os.path.join(path, "device"))
            usb_dev_path = os.path.dirname(device_path)
            vendor_path = os.path.join(usb_dev_path, "idVendor")
            if os.path.exists(vendor_path):
                with open(vendor_path, "r") as f:
                    vendor = f.read().strip()
                if vendor == "1a86":
                    dev_name = os.path.basename(path)
                    return f"/dev/{dev_name}"
        except Exception:
            pass

    # 4. Check if we are running in WSL (WSL fallback to Windows COM mapping)
    is_wsl = False
    try:
        if os.path.exists("/proc/version"):
            with open("/proc/version", "r") as f:
                if "microsoft" in f.read().lower():
                    is_wsl = True
    except Exception:
        pass

    if is_wsl:
        try:
            import subprocess
            # Use PowerShell to find the COM port on Windows host, then map COMx to /dev/ttySx
            cmd = 'Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -like "*CH340*" -or $_.Caption -like "*CH340*" -or $_.Name -like "*CH341*" } | Select-Object -First 1 -ExpandProperty Name'
            out = subprocess.run(["powershell.exe", "-NoProfile", "-Command", cmd], capture_output=True, text=True, timeout=5)
            m = re.search(r'\(COM(\d+)\)', out.stdout)
            if m:
                return f"/dev/ttyS{m.group(1)}"
        except Exception:
            pass

    # 5. Linux fallbacks
    for p in ["/dev/ttyUSB0", "/dev/ttyACM0"]:
        if os.path.exists(p):
            return p
    return "/dev/ttyUSB0"


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    host = _pop_opt(args, "--host") or DEFAULT_HOST
    cmd = args[0]
    rest = args[1:]

    if cmd == "wait":
        sys.exit(0 if wait_ready(host) else 1)
    elif cmd == "state":
        print(json.dumps(_get_state(host), indent=2))
    elif cmd == "telemetry":
        print(json.dumps(_get_telemetry(host), indent=2))
    elif cmd == "watch":
        only = _pop_opt(rest, "--only")
        count = int(_pop_opt(rest, "--count") or 0)
        msg_type = _pop_opt(rest, "--type") or "telemetry"
        raw = "--raw" in rest
        watch(host, only.split(",") if only else None, raw, count, msg_type)
    elif cmd == "scene":
        print(_post_cmd(host, {"cmd": "scene", "value": rest[0]}))
    elif cmd == "screen":
        print(_post_cmd(host, {"cmd": "ui", "screen": rest[0]}))
    elif cmd == "sens":
        print(_post_cmd(host, {"cmd": "sens", "value": int(rest[0])}))
    elif cmd == "fx":
        obj = {"cmd": "fx", "on": rest[0] == "on"}
        if len(rest) > 1:
            obj["value"] = int(rest[1])
        print(_post_cmd(host, obj))
    elif cmd == "status":
        print(_post_cmd(host, {"cmd": "status", "value": " ".join(rest)}))
    elif cmd == "shot":
        out = rest[0] if rest and not rest[0].startswith("--") else "shot.png"
        crop = _pop_opt(rest, "--crop")
        zoom = int(_pop_opt(rest, "--zoom") or 1)
        screen = _pop_opt(rest, "--screen")
        scene = _pop_opt(rest, "--scene")
        do_wait = "--wait" in rest
        shot(host, out, screen, scene, do_wait,
             tuple(int(v) for v in crop.split(",")) if crop else None, zoom)
    elif cmd == "trace":
        trace(_pop_opt(rest, "--port") or detect_port(), int(_pop_opt(rest, "--seconds") or 12))
    elif cmd == "decode":
        decode(rest)
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == "__main__":
    main()
