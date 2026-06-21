#!/usr/bin/env python3
"""Fetch the live UI from the prop's /screenshot endpoint and save a PNG.

The device returns raw RGB565 (little-endian) with X-Width/X-Height headers.
This converts to PNG using only the Python stdlib (no Pillow needed) so the
image can be opened/inspected directly.

Usage:
    python screenshot.py [host_or_url] [out.png] [--crop X,Y,W,H] [--zoom N]

Examples:
    python screenshot.py comm-unit-7.local            # mDNS name (no IP hunting)
    python screenshot.py 192.168.4.1                  # AP address, -> screenshot.png
    python screenshot.py 172.17.2.167 wifi_page.png   # STA address on the LAN
    # inspect a small detail (e.g. the top-right LINK/signal corner) up close:
    python screenshot.py comm-unit-7.local corner.png --crop 764,8,260,90 --zoom 3
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


def crop_rgb888_rows(raw, w, h, box, zoom):
    """Crop (and optionally nearest-neighbour zoom) already-decoded RGB888 rows.

    `raw` is PNG-filtered scanlines (a leading 0 byte per row); `box` is
    (x, y, cw, ch). Returns (out_w, out_h, new filtered rows). Handy for reading
    small details on the 7" panel without squinting at a 1024x600 dump.
    """
    x, y, cw, ch = box
    x = max(0, min(x, w)); y = max(0, min(y, h))
    cw = max(1, min(cw, w - x)); ch = max(1, min(ch, h - y))
    stride = 1 + w * 3
    out = []
    for ry in range(ch):
        src = (y + ry) * stride + 1 + x * 3
        line = raw[src:src + cw * 3]
        if zoom > 1:
            line = b"".join(line[i:i + 3] * zoom for i in range(0, len(line), 3))
        row = bytes(line)
        out.extend([b"\x00" + row] * zoom)
    return cw * zoom, ch * zoom, b"".join(out)


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
    args = sys.argv[1:]
    crop = None
    zoom = 1
    pos = []
    i = 0
    while i < len(args):
        if args[i] == "--crop" and i + 1 < len(args):
            crop = tuple(int(v) for v in args[i + 1].split(",")); i += 2
        elif args[i] == "--zoom" and i + 1 < len(args):
            zoom = max(1, int(args[i + 1])); i += 2
        else:
            pos.append(args[i]); i += 1
    host = pos[0] if pos else "192.168.4.1"
    out = pos[1] if len(pos) > 1 else "screenshot.png"
    url = host if host.startswith("http") else f"http://{host}/screenshot"
    w, h, data = fetch(url)
    raw = rgb565le_to_rgb888_rows(data, w, h)
    if crop or zoom > 1:
        ow, oh, raw = crop_rgb888_rows(raw, w, h, crop or (0, 0, w, h), zoom)
        write_png(out, ow, oh, raw)
        print(f"saved {out} ({ow}x{oh} from {w}x{h}, crop={crop} zoom={zoom})")
    else:
        write_png(out, w, h, raw)
        print(f"saved {out} ({w}x{h}, {len(data)} bytes raw)")


if __name__ == "__main__":
    main()
