---
name: sdkconfig-tuning
description: Change and verify Kconfig/sdkconfig settings on the CrowPanel communicator prop (repo root) — especially esp_hosted/WiFi/SPIRAM stability levers. Use whenever editing sdkconfig.defaults or sdkconfig, or investigating intermittent boot panics, mempool failures, or WiFi-hosted-mode instability.
---

# sdkconfig tuning — change + verify loop

This board's stability history (see `AGENTS.md`'s Crash forensics section and the
`ftm-ranging-and-ram-margin` memory) is a string of internal-RAM-margin and
esp_hosted-WiFi Kconfig fixes, each only trusted after a real reboot-stress test on
hardware — not just "it built." Use this loop for any sdkconfig change in that vein.

## 1. Edit both files

Edit `sdkconfig.defaults` (source of truth, with a dated comment explaining *why*) **and**
the generated `sdkconfig` directly — `idf.py reconfigure` alone will NOT apply a
`sdkconfig.defaults` change if the key already has a value in `sdkconfig`; defaults only
seed keys that don't exist yet.

## 2. Watch for Kconfig aliases before trusting a "no effect" result

Under `CONFIG_ESP_WIFI_REMOTE_LIBRARY_HOSTED=y` (this board's P4+C6 setup),
`CONFIG_ESP_WIFI_*` names are mirrored aliases of `CONFIG_WIFI_RMT_*` (defined in
`managed_components/espressif__esp_wifi_remote/*/Kconfig.wifi.in`) — editing the alias
directly silently no-ops on reconfigure. If a WiFi-Kconfig edit doesn't stick after
`idf.py reconfigure`, grep `managed_components/espressif__esp_wifi_remote` for the config
name before assuming the change is unsupported. Other Kconfig indirections may exist
elsewhere in this build (BT/BLE hosted proxy, etc.) — same principle applies: verify the
*real* key actually changed post-reconfigure, don't just trust your edit.

## 3. Reconfigure and verify the real key stuck

```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" reconfigure
```
```bash
grep -E "^CONFIG_YOUR_KEY=" "f:/git/personal/CrowPanelProp/sdkconfig"   # confirm, don't assume
```

## 4. Build + flash

```powershell
idf.py -C "f:\git\personal\CrowPanelProp" build
idf.py -C "f:\git\personal\CrowPanelProp" -p COM11 flash   # port varies, check GetPortNames()
```

## 5. Stress-test reboots, don't just boot once

```bash
python tools/prop.py trace --port COM11 --seconds 15 --trials 8
```
Intermittent panics have historically taken 2-3 reboots to surface (see sys_evt stack fix
history) — a single clean boot is not sufficient evidence. 8 trials, 0 flagged is the bar
this project has used before landing a stability-related sdkconfig change.

## 6. Confirm functional health, not just "didn't crash"

```bash
python tools/prop.py state       # WiFi/BLE/FTM summary
python tools/prop.py telemetry   # IMU/radar/mic/BLE/CSI all populated?
python tools/prop.py shot x.png --screen spectrum --wait   # visual sanity check,
                                                            # especially for allocator/PSRAM changes
```

## 7. Test isolated when a change is invasive

If a change affects allocation/behavior system-wide (e.g.
`SPIRAM_MALLOC_ALWAYSINTERNAL`, which changes the default placement of every
capability-less `malloc()`), run its own dedicated trials round rather than bundling it
with narrower, lower-risk changes — makes it possible to attribute a regression to the
right change.

## 8. Document why, not just what

Land the reasoning in `sdkconfig.defaults` comments (the settings' actual source of
truth) and, if it's part of an ongoing stability saga, update `AGENTS.md`'s Crash
forensics section too — that section is the narrative history future sessions read
before touching these settings again.
