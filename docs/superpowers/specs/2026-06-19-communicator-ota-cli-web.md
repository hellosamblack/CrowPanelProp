# Add OTA updates (CLI + management webpage)

## Context

We want to update the communicator firmware over WiFi instead of plugging in USB
each time. **The firmware already supports OTA**: `POST /ota?token=…` in
`main/prop_api.c` streams a `.bin` into the inactive A/B slot,
finalizes it, sets the boot partition, and reboots; `partitions.csv` already defines
`ota_0`/`ota_1`. It's even documented in `README.md` as a raw `curl`.

What's missing is the *client/ergonomics* side the user asked for:

1. **CLI** — `tools/dev.ps1` (build/flash/monitor helper) has no `ota` action.
2. **Management webpage** — the `/` "cue board" (`s_console_html` in `prop_api.c`) has
   no firmware-upload control.

And one safety gap: **app rollback is disabled**. A bad OTA image would brick the
board into a boot loop, forcing the exact USB reflash we're trying to avoid. So we
also enable rollback and report the running firmware version so an update can be
confirmed.

## Changes

### 1. CLI: `ota` action in `tools/dev.ps1`
- Add `"ota"` to the `[ValidateSet(...)]` for `$Action`.
- Add params: `[string]$DeviceHost = "comm-unit-7.local"` (the mDNS name from
  `PROP_HOSTNAME` in `prop_net.c`) and `[string]$Token = "prop-ota-2024"` (matches
  `PROP_OTA_TOKEN` in `main/include/prop_api.h`).
- `ota` branch: build first (`idf.py -C $proj build`); on success POST the artifact
  `"$proj/build/communicator.bin"` to `http://$DeviceHost/ota?token=$Token` via
  `Invoke-WebRequest -Method Post -InFile <bin> -Uri <url> -TimeoutSec 120`. Print the
  JSON response (`{"ok":true,"rebooting":true}`) and note the device is rebooting.
- Update the usage comment block at the top with the new `ota` example
  (`dev.ps1 ota -DeviceHost <ip>`).

### 2. Web console: upload control in `s_console_html` (`main/prop_api.c`, ~line 292)
- Add to the HTML: a token text input (prefilled `prop-ota-2024`), a
  `<input type=file accept=.bin>`, an "UPLOAD FIRMWARE" button, and a status/progress
  line — themed amber-on-black to match.
- JS: on click, `XMLHttpRequest` `POST` the selected file to
  `/ota?token=<token>` with the file as the raw body; use `xhr.upload.onprogress` to
  show a percentage, and on completion show the result / "rebooting…". (XHR, not
  `fetch`, so we get upload progress.)
- Keep it inline/minimal — this file is a single embedded C string literal; mind the
  escaping (existing code escapes `\\n`).

### 3. Safety: enable rollback + confirm-valid on boot
- `sdkconfig.defaults`: add `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`. With this, a
  freshly-OTA'd app boots in `PENDING_VERIFY` and is rolled back on the next reset
  unless it explicitly marks itself valid.
- `main/main.c` (`app_main`, after the display/engine/UI/net/API bring-up succeeds):
  call `esp_ota_mark_app_valid_cancel_rollback()` (from `esp_ota_ops.h`). Reaching a
  good UI + running API is our "healthy" signal. Guard is harmless on factory boots.
- Note: `partitions.csv` keeps `factory` + `ota_0` + `ota_1` (fits 16 MB); no change
  needed. `esp_ota_get_next_update_partition` already returns the correct inactive
  slot when running from factory.

### 4. Version reporting (confirm an update landed)
- `main/prop_api.c` `state_to_json()`: add a `"version"` field from
  `esp_app_get_description()->version` (include `esp_app_desc.h`). IDF auto-derives
  this from `git describe`, so each build is distinguishable.
- Surface it on the web console state line and (optionally) extend the existing
  `LINK:/IP:` line in `s_console_html`.

### 5. Docs
- `README.md` "WiFi OTA" section: replace/augment the raw curl
  with `pwsh tools/dev.ps1 ota -DeviceHost <ip>` and mention the web-console upload
  button and the rollback behavior.
- `CLAUDE.md` module map / dev-helper notes: mention `dev.ps1 ota`.

## Files
- `tools/dev.ps1` — new `ota` action (+ params, usage).
- `main/prop_api.c` — web upload UI in `s_console_html`;
  `"version"` in `state_to_json()`.
- `main/main.c` — `esp_ota_mark_app_valid_cancel_rollback()`.
- `sdkconfig.defaults` — `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`.
- `README.md`, `CLAUDE.md` — docs.

## Verification (end-to-end)
1. Build: `pwsh tools/dev.ps1 build` — compiles clean.
2. Flash the rollback-enabled base once over USB:
   `pwsh tools/dev.ps1 flash -Port COM7`, monitor and confirm
   `esp_ota_mark_app_valid` runs (no rollback on the next manual reset).
3. Note current version: `curl http://comm-unit-7.local/state` → record `"version"`.
4. Make a visible trivial change (e.g. a status string), rebuild, then OTA via CLI:
   `pwsh tools/dev.ps1 ota -DeviceHost comm-unit-7.local`.
   Expect `{"ok":true,"rebooting":true}`, device reboots.
5. Re-query `/state` — `"version"` changed and the visible change is present →
   confirms the OTA + version reporting + auto-mark-valid all worked.
6. Web path: open `http://comm-unit-7.local/`, use the UPLOAD FIRMWARE control to push
   the same `build/communicator.bin`, watch the progress %, confirm reboot and new
   version.
7. Rollback sanity (optional): the device staying up after reset (not reverting)
   proves `mark_app_valid` fired.
