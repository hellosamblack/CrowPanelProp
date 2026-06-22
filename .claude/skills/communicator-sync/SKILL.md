---
name: communicator-sync
description: Keep the CrowPanel communicator firmware UI (hand-written C in main/prop_ui.c) and the LVGL Pro XML design project (ui/) in sync — in either direction. A collaborator edits screens in the LVGL editor / viewer.lvgl.io; this reconciles their XML changes into the firmware, or mirrors firmware changes back into the XML. Use whenever the C and XML UIs have drifted or might have: "sync the UI", "port the editor's <screen> changes to firmware", "my collaborator redesigned the WIFI/ARCHIVE/SETUP screen in LVGL", "pull the LVGL changes into prop_ui.c", "the XML and C are out of sync", "regenerate/update the XML for <panel>", "reflect this prop_ui.c change in the design", or after either ui/*.xml or main/prop_ui.c is edited and the other side should follow. Also use to validate that ui/ still previews in the viewer.
---

# Communicator — C ↔ XML UI sync

The live firmware UI is hand-written C in `main/prop_ui.c`. The `ui/` folder is an
LVGL Pro XML project a collaborator edits in viewer.lvgl.io — a **design surface that
mirrors the same screens** but does not drive the build. They must not silently
drift: a screen the collaborator restyles in the editor should reach the firmware, and
a panel reworked in C should be reflected back so the design surface stays honest.

This skill reconciles the two **in whichever direction the drift points**. Neither side
is globally authoritative — match by intent, screen by screen. Read the repo
`CLAUDE.md` first for the architecture; this is the *how to reconcile them* loop.

## The three references — read the ones you need

- **`references/screen-map.md`** — the table mapping each `screen_*.xml` to its
  `build_*_panel()` in `prop_ui.c`, plus **what is intentionally not in sync** (live
  data, procedural art, the CRT overlay). Read this first to find counterparts.
- **`references/translation.md`** — how a construct converts: palette/fonts, the
  `prop_kit` helpers, widget styling, navigation, and the observer↔subjects mapping.
  Read when actually porting an edit.
- **`references/xml-authoring.md`** — viewer-safe XML rules (tiny_ttf ASCII-only glyphs,
  part selectors, `min_value`/`max_value`, screens-can't-hold-`<animations>`, …). Read
  when writing/editing XML; the online viewer fails hard on these.

## Workflow

### 1. Scope the drift from the collaborator's git history
The sync is driven by **what he actually committed**, not by a prose description — so
start in git. Find his UI commits and turn them into a worklist:

```bash
git log --oneline -15 -- ui/                  # his recent editor commits
git show <sha> -- ui/                         # what one commit changed
# the helper turns a range into a C<->XML worklist (changed screens -> their builders):
python3 .claude/skills/communicator-sync/scripts/drift.py --range <sha>..HEAD
python3 .claude/skills/communicator-sync/scripts/drift.py          # uncommitted, both sides
```

`drift.py` reads the diff and prints each changed file's counterpart and the sync
direction, so you don't have to recall the map:

- His editor commits touch `ui/screens/*.xml` / `ui/globals.xml` → **XML → C**.
- Firmware work touches `main/prop_ui.c` / `prop_kit.*` → **C → XML**.
- A shared thing (palette colour, font, the rail) fans out to **both**
  `globals.xml`/`prop_kit.h` (or every screen using it).
- Both sides touched, or a real conflict (he moved a button, C moved it elsewhere) →
  surface it and ask which is intended; don't guess away someone's work.

Read the actual diff of his commit before editing — the worklist tells you *where*, the
diff tells you *what*. `screen-map.md` has the full table and the "not in sync" list.

### 2. Translate the change
Apply the equivalent edit on the other side using `translation.md`. Keep it minimal and
in the surrounding idiom — reach for the `prop_kit` helpers in C and named `<style>`s in
XML, match existing x/y geometry, and keep each screen's header comment pointing at its
counterpart. Skip the things `screen-map.md` lists as *not in sync* — reconcile layout,
palette, and the nav graph, not live values or procedural pixel math. Surface those as
notes instead of editing them away.

### 3. Verify (tiered — do as much as the environment allows)

Always, after any XML edit:
```bash
python3 .claude/skills/communicator-sync/scripts/check_ui.py
```
Green = the project loads and every screen previews. Fix every ERROR before moving on.

After any C edit, build (ESP-IDF 6.0.1, not on PATH — see CLAUDE.md / communicator-ui):
```bash
. ~/.local/esp/esp-idf/export.sh && idf.py build      # Linux
# new main/*.c file? idf.py reconfigure first (CMake GLOBs at configure time)
```

If the board is on USB and you want visual confirmation, close the loop with a
screenshot and compare it to the editor's render of the same screen:
```bash
python tools/prop.py shot synced.png --screen <name> --wait
```
Degrade gracefully: no board → build only; no IDF → `check_ui.py` + a careful read of
the mapping. Say which tier you reached.

### 4. Report
State what synced (which screens, which direction), what you deliberately left
divergent and why (the not-in-sync list), and any genuine conflict that needs the
user's call. If a brand-new screen appeared in the editor with no C counterpart, note
that wiring it into firmware is the full "Add a screen" procedure in `CLAUDE.md`
(new `PK_*`, builder, `open_panel` case, menu row, `close_panel` NULLs, `reconfigure`).

## Guardrails
- **Don't delete the other side's work to make a diff smaller.** If the editor moved a
  button and C moved it elsewhere, that's a conflict — surface it, don't pick silently.
- **Palette/fonts change in lockstep.** A colour edited in `globals.xml` must change the
  matching `COL_*` in `prop_kit.h` to the identical hex, and vice-versa.
- **The XML is for humans + the viewer; the C is for the panel.** When a construct only
  makes sense on one side (observer data, `prop_fx`, `LV_SYMBOL_*` glyphs, animations),
  keep it where it belongs and note the asymmetry — don't fabricate a fake counterpart.
- **Always finish with `check_ui.py` green** when XML was touched.
