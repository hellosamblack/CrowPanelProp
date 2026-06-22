#!/usr/bin/env python3
"""check_ui.py — validate the LVGL XML project under ui/ for viewer-safety.

The collaborator edits screens in viewer.lvgl.io; the online viewer is strict and
fails a screen render on the smallest mistake (an unknown attribute aborts the
whole screen; a missing glyph spams the tiny_ttf cache log). This catches the
classes of error we hit by hand so a sync never ships a screen that won't preview.

Run from the repo root (or pass --ui PATH). Exit code 1 if any ERROR is found;
warnings don't fail the run. Stdlib only — no pip installs.

Checks (ERROR unless noted):
  - every screen/component/globals/project XML is well-formed
  - every <screen_create_event screen="X"> target X.xml exists
  - every bind_*="subj_*" subject is declared in globals.xml
  - every text_font / style_text_font name is declared in globals.xml <fonts>
  - non-ASCII chars or &#NNN; > 127 in any rendered text/options (tiny_ttf can't
    draw glyphs Eurostile lacks -> "cache not allocated" log spam)        [ERROR]
  - bare min=/max= on lv_slider/lv_bar (the props are min_value/max_value)  [ERROR]
  - bundled part keys (indicator_*/knob_*/items_*) inside a <style>         [ERROR]
  - a <screen> containing <animations> or <api> (screens allow only
    consts/styles/view; animations live on components)                     [ERROR]
  - lv_line points not in "(x y) (x y)" form                               [ERROR]
  - subjects declared but never bound                                      [warn]
"""
import os, re, sys, glob, argparse
import xml.dom.minidom as minidom

def find_ui(start):
    for d in (start, os.path.join(start, "ui")):
        if os.path.isfile(os.path.join(d, "project.xml")):
            return d
    return None

def rel(ui, p): return os.path.relpath(p, ui)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ui", default=".", help="repo root or ui/ dir")
    args = ap.parse_args()
    ui = find_ui(os.path.abspath(args.ui))
    if not ui:
        print("ERROR: could not locate the ui/ project (no project.xml found)")
        return 1

    screens   = sorted(glob.glob(os.path.join(ui, "screens", "*.xml")))
    components = sorted(glob.glob(os.path.join(ui, "components", "*", "*.xml")))
    globals_f = os.path.join(ui, "globals.xml")
    all_xml = screens + components + [globals_f, os.path.join(ui, "project.xml")]

    errors, warns = [], []
    def err(f, m):  errors.append(f"{rel(ui,f)}: {m}")
    def warn(f, m): warns.append(f"{rel(ui,f)}: {m}")

    # ---- 1. well-formedness -------------------------------------------------
    text = {}
    for f in all_xml:
        if not os.path.isfile(f):
            continue
        try:
            raw = open(f, encoding="utf-8").read()
            minidom.parseString(raw)
            text[f] = raw
        except Exception as e:
            err(f, f"not well-formed XML: {e}")
    if not os.path.isfile(globals_f):
        err(globals_f, "missing globals.xml")

    g = text.get(globals_f, "")
    declared_subjects = set(re.findall(r'<(?:int|string|float)\s+name="([^"]+)"', g))
    declared_fonts    = set(re.findall(r'<(?:tiny_ttf|bin|freetype)\s+[^>]*name="([^"]+)"', g))
    screen_names = {os.path.splitext(os.path.basename(s))[0] for s in screens}
    bound_subjects = set()

    # ---- 2. per-file structural + smell checks ------------------------------
    for f in screens + components:
        raw = text.get(f)
        if raw is None:
            continue
        is_screen = os.path.dirname(f).endswith("screens")

        # screen_create_event targets
        for tgt in re.findall(r'screen_create_event\s+[^>]*screen="([^"]+)"', raw):
            if tgt not in screen_names:
                err(f, f'screen_create_event -> "{tgt}" but screens/{tgt}.xml does not exist')

        # bound subjects must be declared
        for s in re.findall(r'bind_[a-z_]+="([A-Za-z_][\w]*)"', raw):
            bound_subjects.add(s)
            if s not in declared_subjects:
                err(f, f'binds subject "{s}" not declared in globals.xml <subjects>')

        # font references must be declared
        for fn in re.findall(r'(?:style_text_font|text_font)="([^"#][^"]*)"', raw):
            if fn not in declared_fonts:
                err(f, f'references font "{fn}" not declared in globals.xml <fonts>')

        # non-ASCII glyphs in rendered text (text= / options= / label text)
        for attr, val in re.findall(r'(text|options)="([^"]*)"', raw):
            for ent in re.findall(r'&#(\d+);', val):
                if int(ent) > 127:
                    err(f, f'{attr}= contains non-ASCII glyph &#{ent}; (tiny_ttf can\'t draw it)')
            for ch in val:
                if ord(ch) > 127:
                    err(f, f'{attr}= contains non-ASCII char U+{ord(ch):04X} {ch!r}')

        # invalid slider/bar range attrs
        for m in re.findall(r'<lv_(?:slider|bar)\b[^>]*', raw):
            if re.search(r'\bmin="', m) or re.search(r'\bmax="', m):
                err(f, "lv_slider/lv_bar uses min=/max=; the props are min_value/max_value")

        # bundled part keys inside a <style> (must be selector= instead)
        for st in re.findall(r'<style\b[^>]*>', raw):
            for bad in ("indicator_bg_color", "indicator_radius", "indicator_border",
                        "knob_bg_color", "knob_radius", "items_bg_color",
                        "items_text_color", "items_border", "items_radius"):
                if bad in st:
                    err(f, f'<style> bundles part key "{bad}"; style a part via selector= or style_<prop>-<part>')

        # lv_line points format
        for pts in re.findall(r'<lv_line\b[^>]*\bpoints="([^"]*)"', raw):
            if pts.strip() and not pts.lstrip().startswith("("):
                err(f, 'lv_line points must be "(x y) (x y) ..." parenthesized pairs')

        # screens may only hold consts/styles/view
        if is_screen:
            if "<animations" in raw:
                err(f, "a <screen> cannot contain <animations> (component-only)")
            if re.search(r'<api\b', raw):
                err(f, "a <screen> cannot contain <api>")

    for s in sorted(declared_subjects - bound_subjects):
        warn(globals_f, f'subject "{s}" declared but never bound (ok as a future hook)')

    # ---- report -------------------------------------------------------------
    n_files = len([f for f in all_xml if f in text])
    print(f"checked {n_files} XML files under {rel(ui, ui) or '.'}/  "
          f"({len(screens)} screens, {len(components)} components)")
    for w in warns:  print(f"  warn  {w}")
    for e in errors: print(f"  ERROR {e}")
    if errors:
        print(f"\nFAIL: {len(errors)} error(s), {len(warns)} warning(s)")
        return 1
    print(f"\nOK: viewer-safe ({len(warns)} warning(s))")
    return 0

if __name__ == "__main__":
    sys.exit(main())
