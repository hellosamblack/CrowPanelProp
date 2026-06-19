#!/usr/bin/env python3
"""Fetch the live UI from the prop's /screenshot endpoint and save a PNG.

The device returns raw RGB565 (little-endian) with X-Width/X-Height headers.
This converts to PNG using only the Python stdlib (no Pillow needed) so the
image can be opened/inspected directly.

Usage:
    python screenshot.py [host_or_url] [out.png]

Examples:
    python screenshot.py 192.168.4.1                 # AP address, -> screenshot.png
    python screenshot.py 172.17.2.167 wifi_page.png  # STA address on the LAN
    python screenshot.py http://10.0.0.5/screenshot shot.png
"""
import sys
import struct
import zlib
import urllib.request


def fetch(url):
    with urllib.request.urlopen(url, timeout=15) as r:
        w = int(r.headers.get("X-Width", "0"))
        h = int(r.headers.get("X-Height", "0"))
        data = r.read()
    if not w or not h:
        raise SystemExit("device did not report X-Width/X-Height")
    return w, h, data


def rgb565le_to_rgb888_rows(data, w, h):
    px = memoryview(data).cast("H")  # native (LE) uint16 on x86 == RGB565LE
    rows = []
    for y in range(h):
        row = bytearray(b"\x00")  # PNG filter type 0 per scanline
        base = y * w
        for x in range(w):
            v = px[base + x]
            r = (v >> 11) & 0x1F
            g = (v >> 5) & 0x3F
            b = v & 0x1F
            row += bytes(((r * 255) // 31, (g * 255) // 63, (b * 255) // 31))
        rows.append(bytes(row))
    return b"".join(rows)


def write_png(path, w, h, raw):
    def chunk(typ, payload):
        return (struct.pack(">I", len(payload)) + typ + payload +
                struct.pack(">I", zlib.crc32(typ + payload) & 0xFFFFFFFF))
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit RGB
    idat = zlib.compress(raw, 9)
    with open(path, "wb") as f:
        f.write(sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b""))


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.1"
    out = sys.argv[2] if len(sys.argv) > 2 else "screenshot.png"
    url = host if host.startswith("http") else f"http://{host}/screenshot"
    w, h, data = fetch(url)
    raw = rgb565le_to_rgb888_rows(data, w, h)
    write_png(out, w, h, raw)
    print(f"saved {out} ({w}x{h}, {len(data)} bytes raw)")


if __name__ == "__main__":
    main()
