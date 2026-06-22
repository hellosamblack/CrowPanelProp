# Screen ↔ builder map

Every LVGL XML screen mirrors a builder (or visual) in `main/prop_ui.c`. Use this
to find the counterpart of whatever changed. The XML file's name (minus `.xml`) is
its screen name; the viewer auto-discovers screens in `ui/screens/`.

| XML screen (`ui/screens/`) | C builder (`main/prop_ui.c`) | `PK_*` kind |
|---|---|---|
| `screen_home` | `build_home_panel` | `PK_HOME` |
| `screen_scanner` | `build_screen` (the root readout shown when no panel is up) | `PK_NONE` |
| `screen_setup` | `build_menu_panel` (titled "SETUP") | `PK_MENU` |
| `screen_wifi` | `build_wifi_panel` | `PK_WIFI` |
| `screen_display` | `build_display_panel` | `PK_DISPLAY` |
| `screen_audio` | `build_audio_panel` | `PK_AUDIO` |
| `screen_leds` | `build_leds_panel` | `PK_LEDS` |
| `screen_about` | `build_about_panel` | `PK_ABOUT` |
| `screen_instruments` | `build_instruments_panel` | `PK_INSTRUMENTS` |
| `screen_sensors` | `build_sensors_panel` | `PK_SENSORS` |
| `screen_vitals` | `build_vitals_panel` | `PK_VITALS` |
| `screen_scan` | `build_signal_panel` ("SIGNAL SCAN") | `PK_SCAN` |
| `screen_spectrum` | `build_spectrum_panel` | `PK_SPECTRUM` |
| `screen_rfband` | `build_rfband_panel` | `PK_RFBAND` |
| `screen_ble` | `build_ble_panel` ("CONTACTS") | `PK_BLE` |
| `screen_csi` | `build_csi_panel` ("SIGNAL ENV") | `PK_CSI` |
| `screen_archive` | `build_archive_panel` | `PK_ARCHIVE` |
| `screen_article` | `build_article_panel` + `vis_climate` | `PK_ARTICLE` (section 0) |
| `screen_article_map` | `build_article_panel` + `vis_map` | `PK_ARTICLE` (section 1) |
| `screen_article_wildlife` | `build_article_panel` + `vis_wildlife` | `PK_ARTICLE` (section 2) |
| `screen_article_plants` | `build_article_panel` + `vis_plants` | `PK_ARTICLE` (section 3) |
| `screen_cassette` | `build_cassette_panel` | `PK_CASSETTE` |
| `screen_insights` | `build_insights_panel` | `PK_INSIGHTS` |
| `screen_io` | `build_io_panel` | `PK_IO` |
| `screen_io_pin` | `build_io_pin_panel` | `PK_IO_PIN` |

Non-screen counterparts:

| XML | C |
|---|---|
| `ui/components/nav_rail/nav_rail.xml` | `build_rail` + `s_rail[]` + `draw_icon` (rail glyphs are primitive-drawn in C) |
| `ui/components/column`, `row` | `kit_body` / flex containers (LVGL flex; no C component) |
| `ui/globals.xml` `<consts>` colors | `prop_kit.h` `COL_*` |
| `ui/globals.xml` `<fonts>` | `prop_kit.h` `FONT_*` (`LV_FONT_DECLARE`d C arrays) |
| `ui/globals.xml` `<subjects>` | `prop_engine` state pushed through `ui_observer` (no 1:1 C object) |

## The four ARCHIVE motifs are one C function

`build_article_panel` switches on `section` (0–3) and calls one of `vis_climate` /
`vis_map` / `vis_wildlife` / `vis_plants`. In XML they're four separate screens
(`screen_article*`) because the viewer can't branch. When syncing an article, edit
the matching `vis_*` (C) or the matching `screen_article*` (XML) — not all four.

## What does NOT map (don't force these into sync)

Some things are intentionally different between the live C and the XML design surface.
Flag them in your summary instead of trying to reconcile them:

- **Live / observer-driven values** — clocks, RSSI, temperatures, scan results, the
  waveform, FFT/CSI bars. In C they're filled by `ui_observer` (~20 Hz) and background
  tasks; in XML they're static demo values, optionally `bind_*` to a `subj_*`. Matching
  the *layout* is the goal; the *data* is meant to differ.
- **Procedural art** — the article `vis_*` visuals, rail glyphs (`draw_icon`), the
  channel gauge, signal-cell meter: drawn from primitives/hashes in C. The XML mocks
  them with fixed shapes/polylines. Reconcile structure and palette, not pixel math.
- **The CRT overlay (`prop_fx`)** and the FPS HUD — C-only post effects; no XML.
- **Dynamic page structure** — `build_io_pin_panel` changes its rows by pin MODE;
  `build_article_panel` picks a motif by section. XML shows one representative variant.
- **Navigation** — C uses `open_panel(PK_*)` via callbacks; XML uses `screen_create_event`.
  Same graph, different mechanism (see translation.md → Navigation).
