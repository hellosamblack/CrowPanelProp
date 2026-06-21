# Communicator Prop — Archive Reframing (addendum)

**Date:** 2026-06-19
**Status:** Implemented (initial pass)
**Supersedes the top-level framing of:** `2026-06-19-communicator-prop-roadmap-design.md`
(that roadmap stays valid for the **SCANNER instrument**, which is now one function
among many rather than the whole device).

## Why

The book author's brief (`resources/authorNotes/`) reframes the prop. The device is
Zarrah's **multi-data interface** — plugged into an *Armada data hub*, a *"sponge of
data"*, showing temp/time/date, holding a downloaded archive (plant history ≈ a
"download of Wikipedia"), storing/playing cassettes, and programmable to make insights.
For conventions it should be **picked up and played with**, themed to the Traxian world,
with browsable sections (**Traxian Climate**, **a map of the desert**, **Traxian
wildlife bios**, **plant guide**). Controls: a **dial moves between functions**,
**switches act as tabs**, buttons change with context.

The firmware was a generic scanner. Three decisions set the direction:
1. **Archive is primary.** The data-archive console is the main experience; the scanner
   demotes to one instrument.
2. **Controls simulated now.** All IO is wired eventually, but today the dial/tab/action
   model is driven from the web portal; real knobs/switches drop in later behind the same
   entry point.
3. **Placeholder content.** Author populates later; current content uses **real-Earth
   desert** stand-ins (saguaro/agave, fennec/sidewinder, real desert climate).

## What changed (this pass)

- **Input abstraction** (`prop_ui_input(control,arg)` in `prop_ui.c`, declared in
  `prop_ui.h`): `selector` (rotate/press), `tab` (archive section), `action`. Driven by
  `POST /cmd {"cmd":"input",...}` + new buttons on the web cue board (`prop_api.c`). The two
  wired GPIO buttons now stand in for the SELECTOR dial (`main.c`). GPIO knob/switch wiring
  is the remaining hardware step — route them into the same `prop_ui_input()`.
- **Console home** (`PK_HOME`, the default landing screen): device identity
  (ARMADA MULTI-DATA INTERFACE // ZARRAH), a data-sponge status strip
  (clock=uptime, date=placeholder, core temp, intake/link), and the **function rail** the
  selector drives: `ARCHIVE · SCANNER · VITALS · SIGNAL SCAN · SPECTRUM · CASSETTE ·
  INSIGHTS · SETUP`. SETUP now holds **configuration only** (instruments moved to the rail).
- **Archive browser** (`PK_ARCHIVE` + `PK_ARTICLE`): tabbed sections from a new
  author-editable content layer **`main/prop_content.{c,h}`** (sections → entries →
  `{title, body}`). Tabs = the author's switches; dial scrolls/selects.
- **Worldbuilding copy**: in-world boot sequence (`prop_engine.c boot_status`) and titles.
- **Cassette + Insights** stubs on the rail (no audio output yet).

All panels stay **lazily built, one alive at a time** (LV_MEM is hard-capped — see firmware
`CLAUDE.md`). Builds clean on ESP-IDF 6.0.1.

## Still open / next

- **Wire the real controls** into `prop_ui_input` (encoder + switches + buttons via a
  `bsp_io` control table, mirroring `led_table`/`button_table`).
- **Map image** for the desert section (PNG→C array, PSRAM) — the section is text-only now.
- **Real time/date** (SNTP when STA is up) for the console clock; temp already live.
- **Author content** replaces the real-Earth placeholders in `prop_content.c`.
- **Cassette/insights** promotion (audio output stays low priority per the original roadmap).

## Verify on device

```
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash
python tools/prop.py shot console.png --screen home --wait
# drive the dial/tabs, then capture:
#   POST /cmd {"cmd":"input","control":"selector","arg":"cw"}    -> rail highlight moves
#   POST /cmd {"cmd":"input","control":"selector","arg":"press"} -> opens highlighted function
#   POST /cmd {"cmd":"input","control":"tab","arg":2}            -> archive WILDLIFE tab
python tools/prop.py shot archive.png --screen archive --wait
```
