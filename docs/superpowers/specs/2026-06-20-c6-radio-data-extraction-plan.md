# Plan: Extract & display C6 radio data (BLE scan, WiFi channel density, WiFi CSI)

## Context

The communicator prop currently uses the ESP32-C6 co-processor (via esp-hosted over
SDIO) only for ordinary WiFi: AP+STA connectivity, a scan for the WiFi setup panel, and
a cached RSSI readout. The C6 radios can yield far richer, prop-flavoured "sensor" data.
Exploration confirmed three viable sources, in increasing risk:

1. **WiFi channel density** — pure reuse of the existing scan path; zero new radio config.
2. **BLE scan** — the **C6 slave already runs the BLE controller** (`CONFIG_BT_ENABLED=y`,
   `CONFIG_BT_CONTROLLER_ONLY=y`, HCI-over-RAM). The P4 host just needs NimBLE + esp_hosted
   BLE flags enabled. Gives device count / RSSI / MAC / name / manufacturer Company-ID.
3. **WiFi CSI** — APIs are already exported by esp-hosted
   (`esp_wifi_remote_set_csi`, `esp_wifi_remote_set_csi_rx_cb`,
   `CONFIG_SLAVE_SOC_WIFI_CSI_SUPPORT=1`), but per-frame CSI throughput over SDIO and the
   extra internal-RAM cost are unproven — prototype before committing to UI.

Goal: surface these as cassette-futurism instruments ("CONTACT SIGNATURES", "RF BAND",
signal waterfall) on the existing rail/menu, reusing the established
**background-task → cached value → `ui_observer` reads under LVGL lock** pattern (the
`prop_mic` model). The hard constraint is internal RAM: esp_hosted's SDIO DMA mempool +
WiFi + LVGL already make it tight (`LV_MEM` capped at 32 KB). Adding the BLE host stack and
CSI buffers competes for the same pool, so the BLE/CSI work is **phased behind a boot/RAM
verification spike** with a WiFi-only fallback.

## Architecture recap (reuse, don't reinvent)

- **Data cache pattern** — `main/prop_mic.c`: a FreeRTOS task samples in the background and
  writes a small cache (`s_bands[24]`, `s_db`); the UI reads it with a cheap copy. Mirror this
  for `prop_ble` and `prop_csi`. Never touch radio/SDIO APIs under `lvgl_port_lock()`
  (see `prop_net.c:280` `rssi_task`).
- **New screen wiring** (per `firmware/communicator/CLAUDE.md`): add `PK_*` to `panel_kind_t`
  (`prop_ui.c:~129`); write `build_<x>_panel(parent)` using
  `make_panel(parent,"TITLE",back_to_home_cb)`; add a `case` in `open_panel`
  (`prop_ui.c:~281`); add a row to `s_rail[]` (`prop_ui.c:202`) and/or `menu_item(...)`
  (`prop_ui.c:~1886`); add a `prop_ui_goto` name (`prop_ui.c:2487`); NULL widget pointers in
  `close_panel` (`prop_ui.c:~231`) and guard observer/async use with
  `s_cur_kind == PK_<X> && s_widget`.
- **Live readout** — extend `ui_observer` (`prop_ui.c:~2252`) with an `if (s_cur_kind == PK_<X> ...)`
  block, following the `PK_SPECTRUM` example.
- Adding a rail entry grows `RAIL_COUNT` automatically (`RAIL_CELL_H = 600/RAIL_COUNT`), but the
  rail is getting full (9 items). Prefer putting secondary instruments under SETUP/menu or an
  "INSTRUMENTS" sub-list rather than adding 3 more rail icons (see Open question).

---

## Phase 0 — BLE/CSI enablement spike (verify RAM before building UI)

Goal: prove the radios light up and the board still boots, *before* any UI work.

1. **Enable BLE host on the P4** in `sdkconfig.defaults`:
   ```ini
   CONFIG_BT_ENABLED=y
   CONFIG_BT_CONTROLLER_DISABLED=y          # controller lives on the C6
   CONFIG_BT_NIMBLE_ENABLED=y               # NimBLE (lighter than Bluedroid)
   CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE=y     # route HCI to the C6 over esp-hosted
   CONFIG_ESP_HOSTED_NIMBLE_HCI_VHCI=y      # HCI over shared RAM (no extra pins)
   ```
   Reference: `managed_components/espressif__esp_hosted/examples/host_nimble_bleprph_host_only_vhci/sdkconfig.defaults`.
2. Build + flash. **Watch the boot log for `HS_MP: mempool create failed: no mem`** (the
   internal-RAM boot loop). If it appears, the first lever is
   `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` (push WiFi/LWIP buffers to PSRAM), per the
   migration memory — NOT shrinking `LV_MEM`.
3. Temporary smoke test in `app_main` (throwaway): init NimBLE, start a passive scan, log
   discovered devices to serial for ~10 s. Confirms HCI-over-SDIO actually delivers adverts.
4. CSI smoke test (throwaway): call `esp_wifi_set_csi_rx_cb(cb)` + `esp_wifi_set_csi(true)`
   after WiFi start in `prop_net.c`; log whether `cb` ever fires and the subcarrier count.

**Decision gate:**
- BLE boots + scans → proceed to Phase 2.
- Boot loop survives the PSRAM lever → fall back to **WiFi-only** (Phases 1 + 3-as-stub),
  defer BLE, and record the RAM ceiling in the `idf6-migration` memory.
- CSI callback never fires → keep CSI as "WiFi-derived synthetic waterfall" (drive the
  existing waveform from per-channel RSSI variance) instead of true CSI, and note it.

---

## Phase 1 — WiFi channel density ("RF BAND") — lowest risk, do first

New instrument: a 2.4 GHz channel-occupancy bar chart (channels 1-13), each bar height =
count/strength of APs on that channel. Pure reuse of the scan path.

- **`prop_net.c` / `include/prop_net.h`**: add `int prop_net_scan_channels(uint8_t out[14])`
  that runs the same `esp_wifi_scan_start`/`get_ap_records` as `prop_net_scan`
  (`prop_net.c:173`) but aggregates `recs[i].primary` (channel) into a 14-slot histogram
  (weight by RSSI so strong APs read taller). Reuse the existing blocking-scan body; consider
  refactoring the shared scan-and-fetch into a static helper both functions call.
- **`prop_ui.c`**: add `PK_RFBAND`, `build_rfband_panel()` (model on `build_spectrum_panel`,
  `prop_ui.c:~1086` — 13 vertical bars + channel labels), kick a background scan task on open
  (model on `scan_task`, `prop_ui.c:369`, which already runs off the LVGL thread and updates
  widgets under `lvgl_port_lock`), cache the histogram, redraw bars in the `ui_observer`
  `PK_RFBAND` block with the same peak-hold decay used for the mic spectrum.
- Wire menu/`prop_ui_goto` name `rfband`; reuse `prop_engine_set_scene(SCANNING)` styling while a
  scan is in flight.

## Phase 2 — BLE scan ("CONTACT SIGNATURES") — after Phase 0 passes

New module `main/prop_ble.c` + `main/include/prop_ble.h`, mirroring `prop_mic`:

- **Background task**: init NimBLE host (controller is remote on C6), run a continuous passive
  GAP `disc` scan; in the discovery callback cache into a small fixed array:
  ```c
  typedef struct { uint8_t mac[6]; int8_t rssi; char name[20]; uint16_t company_id; uint32_t last_seen; } prop_ble_dev_t;
  ```
  Keep a bounded table (e.g. 24 entries, LRU/age-out by `last_seen`), plus aggregates:
  total count, strongest RSSI, named-vs-anonymous, count of recognised Company-IDs.
- **API** (cache reads, benign-race like `prop_mic_get_bands`):
  `bool prop_ble_available(void)`, `int prop_ble_get_devices(prop_ble_dev_t *out, int max)`,
  `void prop_ble_get_summary(int *count, int8_t *strongest, ...)`.
- **Company-ID flavour**: small static lookup of common Bluetooth SIG Company IDs
  (Apple 0x004C, Microsoft 0x0006, Samsung 0x0075, Google 0x00E0…) → prop labels like
  "CIVILIAN DEVICE" / "UNKNOWN EMITTER". Author-editable table (mirror `prop_content.c`
  spirit). Keep it short — it's flavour, not a real OUI database.
- **`prop_ble_init()`** called from `app_main` after `prop_net_init` (shares the C6 link).
  Guard so failure (BLE disabled / RAM) is non-fatal and the panel shows "-- BLE OFFLINE --"
  (mirror `prop_mic_available()` handling in `build_spectrum_panel`).
- **UI**: `PK_BLE`, `build_ble_panel()` — a header summary (N CONTACTS / strongest dBm) plus a
  scrolling list of contact rows (RSSI bar + name/Company label). Update the list in
  `ui_observer` under the `PK_BLE` guard, throttled (e.g. every ~8 ticks) since the device set
  changes slowly. NULL `s_ble_*` widgets in `close_panel`.
- Wire `prop_ui_goto` name `ble`, plus a menu/rail entry (see Open question on rail vs menu).

## Phase 3 — WiFi CSI waterfall ("SIGNAL ENVIRONMENT") — gated on Phase 0 CSI result

Only build the real version if the Phase 0 CSI callback fired.

- New `main/prop_csi.c` + header, `prop_mic`-style: register `esp_wifi_set_csi_rx_cb`, in the
  callback (keep it tiny — it runs in WiFi context) copy the raw CSI buffer into a ring, and a
  background task folds subcarrier I/Q into N amplitude bins (0..100), cached for the UI.
  Allocate all CSI working buffers in **PSRAM** (`MALLOC_CAP_SPIRAM`), like `prop_mic.c:init`.
- CSI only arrives on received frames, so enable it while associated (STA) and/or periodically
  ping the gateway to generate traffic; document that an idle link yields a static trace.
- **UI**: `PK_CSI`, `build_csi_panel()` — a scrolling waterfall (or reuse the existing waveform
  renderer driven by CSI bins). Wire `prop_ui_goto` name `csi`.
- **Fallback (if CSI doesn't deliver):** drive the same waterfall widget from a cheap synthetic
  source — per-channel RSSI variance sampled in a background task — and label it honestly in
  code comments. No new radio config needed for the fallback.

---

## Critical files

| File | Change |
|------|--------|
| `firmware/communicator/sdkconfig.defaults` | Phase 0: BLE/NimBLE + esp_hosted BT flags; possibly `SPIRAM_TRY_ALLOCATE_WIFI_LWIP`. |
| `firmware/communicator/main/prop_net.c` + `include/prop_net.h` | Phase 1: `prop_net_scan_channels()` (refactor shared scan body). Phase 0/3: CSI enable hooks. |
| `firmware/communicator/main/prop_ble.c` + `include/prop_ble.h` (new) | Phase 2: NimBLE scan task + cache + Company-ID table. |
| `firmware/communicator/main/prop_csi.c` + `include/prop_csi.h` (new) | Phase 3: CSI capture/fold (or synthetic fallback). |
| `firmware/communicator/main/prop_ui.c` | `PK_RFBAND/PK_BLE/PK_CSI`, `build_*_panel`, `open_panel`/`close_panel`, `s_rail[]`/`menu_item`, `prop_ui_goto`, `ui_observer` blocks. |
| `firmware/communicator/main/CLAUDE.md` (module map) | Document new modules + any new RAM ceiling learned in Phase 0. |
| `firmware/communicator/main/prop_api.c` | (optional) extend `/state` JSON with ble/rfband summaries for scripted screenshots. |

## Verification (per phase)

- **Build/flash**: `& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"; idf.py -C firmware/communicator build` then `... -p COM7 flash monitor` (or `pwsh tools/dev.ps1 bf -Port COM7`).
- **Boot health (Phase 0 gate)**: monitor log shows no `HS_MP: mempool ... no mem` loop; WiFi STA still gets an IP; (BLE) NimBLE host reports sync; (CSI) callback fires with a subcarrier count.
- **Visual** (per `communicator-ui` skill / `tools/prop.py`):
  - `python firmware/communicator/tools/prop.py shot rfband.png --screen rfband --wait`
  - `python firmware/communicator/tools/prop.py shot ble.png --screen ble --wait`
  - `python firmware/communicator/tools/prop.py shot csi.png --screen csi --wait`
  Inspect the PNGs directly (CRT `prop_fx` overlay isn't captured — judge that on the panel).
- **Functional**: with the prop near a phone/laptop, the BLE panel device count should track
  devices appearing/disappearing; the RF BAND chart should shift when scanning near a known AP's
  channel; CSI trace should animate when STA traffic flows and freeze when idle.
- **No-regression**: confirm existing WiFi connect, `/screenshot`, and the mic SPECTRUM panel
  still work (BLE/CSI must not starve their shared internal RAM).

## Open questions (decide during execution)

- **Rail vs menu placement**: the left rail already has 9 entries. Recommend BLE → rail
  ("CONTACTS"), RF BAND + CSI → under an INSTRUMENTS/SETUP sub-list to avoid overcrowding.
- **BLE scan duty cycle**: continuous passive scan (more current, fresher) vs periodic windows
  (less RAM/radio contention with WiFi on the shared C6). Start periodic, tune after Phase 0.
