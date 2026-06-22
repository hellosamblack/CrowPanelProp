#!/usr/bin/env python3
"""drift.py — turn a git range into a C<->XML reconciliation worklist.

The collaborator commits XML edits from the LVGL editor; sync starts by looking at
*his commits*, not a prose description. This lists which UI files a range touched and
prints each one's counterpart on the other side, so you know exactly what to reconcile
and in which direction.

Usage (from repo root):
  drift.py                       # working tree vs HEAD (uncommitted edits)
  drift.py --range HEAD~3..HEAD  # a span of his commits
  drift.py --range origin/master..master   # what he pushed since you last synced
  drift.py --since <ref>         # <ref>..HEAD

Stdlib only. Prints a worklist; exit 0 always (it's informational).
"""
import os, re, sys, subprocess, argparse

# screen name (file stem) -> C counterpart in main/prop_ui.c. Keep in step with
# references/screen-map.md.
SCREEN_TO_C = {
    "screen_home": "build_home_panel",
    "screen_scanner": "build_screen (root PK_NONE readout)",
    "screen_setup": "build_menu_panel",
    "screen_wifi": "build_wifi_panel",
    "screen_display": "build_display_panel",
    "screen_audio": "build_audio_panel",
    "screen_leds": "build_leds_panel",
    "screen_about": "build_about_panel",
    "screen_instruments": "build_instruments_panel",
    "screen_sensors": "build_sensors_panel",
    "screen_vitals": "build_vitals_panel",
    "screen_scan": "build_signal_panel",
    "screen_spectrum": "build_spectrum_panel",
    "screen_rfband": "build_rfband_panel",
    "screen_ble": "build_ble_panel",
    "screen_csi": "build_csi_panel",
    "screen_archive": "build_archive_panel",
    "screen_article": "build_article_panel + vis_climate",
    "screen_article_map": "build_article_panel + vis_map",
    "screen_article_wildlife": "build_article_panel + vis_wildlife",
    "screen_article_plants": "build_article_panel + vis_plants",
    "screen_cassette": "build_cassette_panel",
    "screen_insights": "build_insights_panel",
    "screen_io": "build_io_panel",
    "screen_io_pin": "build_io_pin_panel",
}

def sh(args):
    return subprocess.run(args, capture_output=True, text=True).stdout

def repo_root():
    r = sh(["git", "rev-parse", "--show-toplevel"]).strip()
    return r or "."

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--range", help="git diff range, e.g. HEAD~3..HEAD")
    ap.add_argument("--since", help="shorthand for <ref>..HEAD")
    args = ap.parse_args()
    root = repo_root()
    os.chdir(root)

    if args.since:
        rng = [f"{args.since}..HEAD"]
        label = f"{args.since}..HEAD"
    elif args.range:
        rng = [args.range]
        label = args.range
    else:
        rng = []                       # working tree vs HEAD
        label = "working tree (uncommitted)"

    files = [f for f in sh(["git", "diff", "--name-only", *rng]).splitlines() if f.strip()]
    ui_screens = [f for f in files if re.match(r"ui/screens/.*\.xml$", f)]
    ui_other   = [f for f in files if re.match(r"ui/(globals|project)\.xml$", f)
                  or re.match(r"ui/components/.*\.xml$", f)]
    c_ui       = [f for f in files if f in ("main/prop_ui.c", "main/prop_kit.c", "main/prop_kit.h")]

    print(f"# reconciliation worklist for: {label}\n")
    if not (ui_screens or ui_other or c_ui):
        print("no UI files touched in this range — nothing to sync.")
        return 0

    if ui_screens:
        print("XML screens changed -> sync INTO the C builder (XML -> C):")
        for f in ui_screens:
            stem = os.path.splitext(os.path.basename(f))[0]
            c = SCREEN_TO_C.get(stem, "??? (not in map — new screen? see CLAUDE.md 'Add a screen')")
            print(f"  {f}\n      -> main/prop_ui.c : {c}")
        print()
    if ui_other:
        print("Shared XML changed -> fans out (palette/fonts/rail):")
        for f in ui_other:
            if f.endswith("globals.xml"):
                tgt = "main/prop_kit.h (COL_* / FONT_* must match hex/size exactly)"
            elif "nav_rail" in f:
                tgt = "main/prop_ui.c : build_rail + s_rail[] + draw_icon"
            else:
                tgt = "every screen using it"
            print(f"  {f}\n      -> {tgt}")
        print()
    if c_ui:
        print("Firmware UI changed -> mirror INTO the XML design (C -> XML):")
        c_to_screen = {}
        for stem, c in SCREEN_TO_C.items():
            c_to_screen.setdefault(c.split()[0], []).append(stem)
        for f in c_ui:
            print(f"  {f}\n      -> reflect the changed builder(s) in the matching ui/screens/*.xml "
                  f"(see references/screen-map.md), then run check_ui.py")
        print()

    print("Next: read references/translation.md for each, apply the minimal equivalent "
          "edit, then verify (check_ui.py for XML, idf.py build for C).")
    return 0

if __name__ == "__main__":
    sys.exit(main())
