#!/usr/bin/env python3
"""Compare PNG screenshots (stdlib only) — the visual-regression gate for kit refactors.

Reports the % of changed pixels and the max per-channel delta between two captures of the
same screen, and optionally writes an amber heatmap of what moved. Takes two files or two
directories (directory mode diffs every PNG present in both).

Usage:
    python tools/diff_png.py baselines/scan.png after/scan.png         # one screen
    python tools/diff_png.py baselines/scan.png after/scan.png d.png   # + heatmap
    python tools/diff_png.py baselines after                           # whole gallery
    python tools/diff_png.py baselines after --thresh 8                # ignore tiny deltas

A few % of change from anti-aliasing/animation is normal; a screen you didn't touch
showing big deltas is the red flag. Pairs with gallery.py (--out baselines / --out after).
"""
import sys
import os
import struct
import zlib


def decode_png(path):
    """Return (w, h, bytearray rgb) for an 8-bit RGB (color type 2) PNG."""
    with open(path, "rb") as f:
        sig = f.read(8)
        if sig != b"\x89PNG\r\n\x1a\n":
            raise ValueError("%s: not a PNG" % path)
        w = h = ctype = bitdepth = None
        idat = b""
        while True:
            head = f.read(8)
            if len(head) < 8:
                break
            ln, typ = struct.unpack(">I", head[:4])[0], head[4:]
            payload = f.read(ln)
            f.read(4)  # crc
            if typ == b"IHDR":
                w, h, bitdepth, ctype = struct.unpack(">IIBB", payload[:10])
            elif typ == b"IDAT":
                idat += payload
            elif typ == b"IEND":
                break
    if ctype != 2 or bitdepth != 8:
        raise ValueError("%s: only 8-bit RGB PNGs supported (got ctype=%s bd=%s)"
                         % (path, ctype, bitdepth))
    raw = zlib.decompress(idat)
    bpp, stride = 3, w * 3
    out = bytearray(stride * h)
    prev = bytearray(stride)
    pos = 0
    for y in range(h):
        ft = raw[pos]; pos += 1
        line = bytearray(raw[pos:pos + stride]); pos += stride
        for i in range(stride):
            a = out[y * stride + i - bpp] if i >= bpp else 0
            b = prev[i]
            c = prev[i - bpp] if i >= bpp else 0
            x = line[i]
            if ft == 1:      x += a
            elif ft == 2:    x += b
            elif ft == 3:    x += (a + b) >> 1
            elif ft == 4:    # Paeth
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                x += a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
            out[y * stride + i] = x & 0xFF
        prev = out[y * stride:(y + 1) * stride]
    return w, h, out


def diff(a_path, b_path, thresh, heatmap=None):
    aw, ah, a = decode_png(a_path)
    bw, bh, b = decode_png(b_path)
    if (aw, ah) != (bw, bh):
        return "size %dx%d vs %dx%d" % (aw, ah, bw, bh)
    n = aw * ah
    changed = 0
    maxd = 0
    hm = bytearray(n * 3) if heatmap else None
    for i in range(n):
        j = i * 3
        d = max(abs(a[j] - b[j]), abs(a[j + 1] - b[j + 1]), abs(a[j + 2] - b[j + 2]))
        if d > maxd:
            maxd = d
        if d > thresh:
            changed += 1
            if hm:
                hm[j], hm[j + 1], hm[j + 2] = 0xE0, 0xB0, 0x00  # amber = changed
    if hm:
        rows = b"".join(b"\x00" + bytes(hm[y * aw * 3:(y + 1) * aw * 3]) for y in range(ah))
        _write_png(heatmap, aw, ah, rows)
    pct = 100.0 * changed / n
    return "%6.2f%% changed  maxΔ=%-3d" % (pct, maxd)


def _write_png(path, w, h, rows):
    def chunk(typ, p):
        return struct.pack(">I", len(p)) + typ + p + struct.pack(">I", zlib.crc32(typ + p) & 0xFFFFFFFF)
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) +
                chunk(b"IDAT", zlib.compress(bytes(rows), 9)) + chunk(b"IEND", b""))


def main():
    args = sys.argv[1:]
    thresh = int(args[args.index("--thresh") + 1]) if "--thresh" in args else 0
    pos = [a for a in args if not a.startswith("--") and args[args.index(a) - 1] != "--thresh"]
    if len(pos) < 2:
        raise SystemExit(__doc__)
    a, b = pos[0], pos[1]
    if os.path.isdir(a) and os.path.isdir(b):
        names = sorted(f for f in os.listdir(a) if f.endswith(".png") and os.path.exists(os.path.join(b, f)))
        for f in names:
            print("%-16s %s" % (f, diff(os.path.join(a, f), os.path.join(b, f), thresh)))
    else:
        out = pos[2] if len(pos) > 2 else None
        print(diff(a, b, thresh, out))


if __name__ == "__main__":
    main()
