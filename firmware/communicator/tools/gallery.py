#!/usr/bin/env python3
"""Capture every prop UI screen to PNGs in one pass — the batch visual loop.

Phase-1 lesson: capturing screens one-at-a-time is slow and the navigation/settle
dance is easy to get wrong. This drives the whole set: navigate (/cmd ui) -> settle ->
capture (/screenshot reads the DPI framebuffer, so the prop_fx CRT overlay IS included).
Retries transient 500s, then leaves the device on `home`.

Usage:
    python tools/gallery.py                         # all screens -> shots/<screen>.png
    python tools/gallery.py --out baselines         # name the output dir (for before/after)
    python tools/gallery.py --only home scan wifi   # a subset
    python tools/gallery.py --host 172.17.2.172     # explicit host (default mDNS name)
    python tools/gallery.py --settle 2.5            # seconds to wait after nav (live screens)

Pair with diff_png.py to gate a refactor:
    python tools/gallery.py --out baselines         # before
    ... make changes, build, flash ...
    python tools/gallery.py --out after             # after
    python tools/diff_png.py baselines after        # report per-screen pixel deltas
"""
import sys
import os
import time
import json
import urllib.request
import urllib.error

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import screenshot

DEFAULT_HOST = "comm-unit-7.local"
# Canonical screen tokens (prop_ui.c prop_ui_goto routing); "scanner" = the bare readout.
SCREENS = ["home", "scanner", "menu", "wifi", "display", "audio", "leds", "about",
           "vitals", "scan", "spectrum", "archive", "cassette", "insights"]


def _cmd(host, obj):
    urllib.request.urlopen("http://%s/cmd" % host,
                           data=json.dumps(obj).encode(), timeout=6).read()


def capture(host, out_dir, screen, settle, retries=4):
    _cmd(host, {"cmd": "ui", "screen": screen})
    time.sleep(settle)
    for _ in range(retries):
        try:
            w, h, data = screenshot.fetch("http://%s/screenshot" % host)
            raw = screenshot.rgb565le_to_rgb888_rows(data, w, h)
            screenshot.write_png(os.path.join(out_dir, screen + ".png"), w, h, raw)
            return True, "%dx%d" % (w, h)
        except urllib.error.HTTPError:
            time.sleep(1.0)
        except Exception as e:  # network/timeout — report and move on
            return False, str(e)
    return False, "500 x%d" % retries


def _opt(args, name, default=None):
    return args[args.index(name) + 1] if name in args else default


def main():
    args = sys.argv[1:]
    host = _opt(args, "--host", DEFAULT_HOST)
    out = _opt(args, "--out", "shots")
    settle = float(_opt(args, "--settle", "2.0"))
    screens = SCREENS
    if "--only" in args:
        screens = []
        for a in args[args.index("--only") + 1:]:
            if a.startswith("--"):
                break
            screens.append(a)
    os.makedirs(out, exist_ok=True)
    ok = 0
    for s in screens:
        good, info = capture(host, out, s, settle)
        print(("OK   %-9s %s" if good else "FAIL %-9s %s") % (s, info))
        ok += good
    print("--- %d/%d captured into %s/ ---" % (ok, len(screens), out))
    try:
        _cmd(host, {"cmd": "ui", "screen": "home"})
    except Exception:
        pass


if __name__ == "__main__":
    main()
