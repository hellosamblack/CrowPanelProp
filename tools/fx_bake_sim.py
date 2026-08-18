#!/usr/bin/env python3
# Offline pixel-exact verification that the LUT bake in main/prop_fx.c::paint_canvas
# reproduces the previous fx_fill-based src-over output. Run: python3 tools/fx_bake_sim.py
# (full 1024x600, ~1 min). Written 2026-08-18 alongside the Task 2.4 deviation.
# Pixel-exact simulation: old fx_fill-based bake vs new LUT bake (prop_fx.c).
FX_W, FX_H = 1024, 600           # full size, exact
SCAN_STEP, SCAN_OPA = 3, 95
VIGN, VIGN_MAX = 80, 70
AMBER = (0xE0, 0xB0, 0x00)
BLACK = (0, 0, 0)

def scale_opa(base, pct): return base * min(pct,100) // 100

def fx_fill_px(d, color, opa):   # src-over into ARGB tuple d=(a,r,g,b)
    if opa == 0: return d
    sr, sg, sb, sa = color[0], color[1], color[2], opa
    da = d[0]; ia = 255 - opa
    oa = sa + da * ia // 255
    if oa == 0: return (0,0,0,0)
    r = (sr*sa + d[1]*da*ia//255) // oa
    g = (sg*sa + d[2]*da*ia//255) // oa
    b = (sb*sa + d[3]*da*ia//255) // oa
    return (oa, r, g, b)

def old_bake(scan_pct, phos_pct, vign_pct):
    wash = scale_opa(20, phos_pct)
    # lv_canvas_fill_bg: every pixel = (wash, amber)
    buf = [[(wash,)+AMBER for _ in range(FX_W)] for _ in range(FX_H)]
    scan = scale_opa(SCAN_OPA, scan_pct)
    if scan:
        for y in range(0, FX_H, SCAN_STEP):
            for x in range(FX_W):
                buf[y][x] = fx_fill_px(buf[y][x], BLACK, scan)
    for i in range(VIGN):
        edge = VIGN_MAX * (VIGN - i) // VIGN
        o = scale_opa(edge, vign_pct)
        if not o: continue
        for x in range(FX_W):
            buf[i][x] = fx_fill_px(buf[i][x], BLACK, o)
            buf[FX_H-1-i][x] = fx_fill_px(buf[FX_H-1-i][x], BLACK, o)
        for y in range(FX_H):
            buf[y][i] = fx_fill_px(buf[y][i], BLACK, o)
            buf[y][FX_W-1-i] = fx_fill_px(buf[y][FX_W-1-i], BLACK, o)
    return buf

def combine(a1, a2): return a1 + a2 - a1*a2//255

def new_bake(scan_pct, phos_pct, vign_pct):
    wash = scale_opa(20, phos_pct)
    scan = scale_opa(SCAN_OPA, scan_pct)
    lut = []
    for a in range(256):
        oa = a + wash - a*wash//255
        if oa == 0: lut.append((0,0,0,0)); continue
        k = wash*(255-a)//255
        lut.append((oa, AMBER[0]*k//oa, AMBER[1]*k//oa, AMBER[2]*k//oa))
    col_a = [0]*FX_W
    for i in range(VIGN):
        edge = VIGN_MAX*(VIGN-i)//VIGN
        o = scale_opa(edge, vign_pct)
        if not o: continue
        col_a[i] = combine(col_a[i], o)
        col_a[FX_W-1-i] = combine(col_a[FX_W-1-i], o)
    buf = []
    for y in range(FX_H):
        row_a = scan if (y % SCAN_STEP == 0) else 0
        vd = min(y, FX_H-1-y)
        if vd < VIGN:
            edge = VIGN_MAX*(VIGN-vd)//VIGN
            row_a = combine(row_a, scale_opa(edge, vign_pct))
        row = [lut[combine(row_a, col_a[x])] if col_a[x] else lut[row_a] for x in range(FX_W)]
        buf.append(row)
    return buf

for cfg in [(60,30,45), (0,0,100), (100,100,100), (60,0,45), (0,30,0)]:
    o, n = old_bake(*cfg), new_bake(*cfg)
    diffs, maxd = 0, 0
    for y in range(FX_H):
        for x in range(FX_W):
            if o[y][x] != n[y][x]:
                diffs += 1
                d = max(abs(a-b) for a,b in zip(o[y][x], n[y][x]))
                maxd = max(maxd, d)
    print(f"cfg={cfg}: mismatched px={diffs}/{FX_W*FX_H} max channel delta={maxd}")

print("--- visual compare (premultiplied) + worst examples ---")
def vis(p):  # premultiplied channels = what the compositor effectively shows
    a,r,g,b = p
    return (a, r*a//255, g*a//255, b*a//255)

for cfg in [(60,30,45), (0,0,100), (100,100,100), (60,0,45)]:
    o, n = old_bake(*cfg), new_bake(*cfg)
    worst, wpos, diffs = 0, None, 0
    for y in range(FX_H):
        for x in range(FX_W):
            vo, vn = vis(o[y][x]), vis(n[y][x])
            if vo != vn:
                diffs += 1
                d = max(abs(a-b) for a,b in zip(vo,vn))
                if d > worst: worst, wpos = d, (x,y,o[y][x],n[y][x])
    print(f"cfg={cfg}: visual-diff px={diffs} worst delta={worst} at {wpos}")
