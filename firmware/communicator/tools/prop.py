#!/usr/bin/env python3
"""prop.py - one CLI for driving + inspecting the communicator prop during dev.

Collapses the build/flash -> wait -> drive UI -> screenshot loop (and crash
decoding) into single commands, so iterating on firmware/communicator is fast.

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

Screens: home menu wifi display audio leds vitals scan spectrum about
Scenes:  IDLE SCANNING SIGNAL_ACQUIRED COMMS ALERT
"""
import sys
import os
import json
import time
import glob
import re
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
        trace(_pop_opt(rest, "--port") or "COM7", int(_pop_opt(rest, "--seconds") or 12))
    elif cmd == "decode":
        decode(rest)
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == "__main__":
    main()
