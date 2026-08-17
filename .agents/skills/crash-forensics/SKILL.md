---
name: crash-forensics
description: >-
  Decode a panic or boot-time crash on the CrowPanel communicator prop (repo root) using the
  flash core dump partition, and the ESP-IDF/board-specific gotchas already worked out for this
  hardware's intermittent boot panics. Use whenever investigating a crash, panic, boot loop, or
  intermittent reboot on this board, or before touching CONFIG_ESP_COREDUMP_* / FREERTOS_ISR_STACKSIZE
  / ESP_SYSTEM_EVENT_TASK_STACK_SIZE settings.
---

# Crash forensics (flash core dump)

**Enabled as of 2026-07-01.** It was disabled for a while: enabling it forces `FREERTOS_ISR_STACKSIZE`
from 1536→2096 bytes/core (a documented ESP-IDF Kconfig floor, `components/freertos/Kconfig`), and
that extra ~1.1 KB used to land in the narrow pre-scheduler window where `main_task`'s own 8192 B
stack gets allocated, starving it (measured: `internal free=10832, largest contiguous block=7168`
bytes at that point — enough total, not enough contiguous). The fix was
`CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM` (`sdkconfig.defaults`) — moves esp_hosted's SDIO DMA mempool
to PSRAM (safe on the P4: its PSRAM DMA is cache-coherent, unlike classic ESP32) — confirmed on
hardware to free ~89 KB of internal RAM, comfortably clearing that ~1.1 KB shortfall.

Re-enabling coredump then exposed a second, separate, genuinely-intermittent bug: the esp_event
`sys_evt` task's default 2304 B stack overflows the hardware stack guard on some boots, in the
STA_GOT_IP → mDNS registration → netif handler callback chain (`Stack protection fault`, SP landing
12 bytes below the stack's lower bound). This is very likely the original "intermittent boot-time
panic" this section was written to catch, just never pinned to a task before — coredump's stack-canary
check is what finally caught it. Fixed by bumping `CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE` to 3584 B
(`sdkconfig.defaults`); verified with 8/8 clean reboots afterward (was intermittent before the fix —
2 of 3 initial trials panicked at that exact point). If a new intermittent panic ever reappears here,
check `sys_evt` first before assuming it's the same root cause.

**Further internal-RAM headroom (2026-07-01).** Cross-referenced xiaozhi-esp32's `elecrow-p4-board`
port (a community AI-chatbot firmware with a maintained board port for this exact P4+C6 hardware) for
esp_hosted tuning ideas. Landed five changes (`sdkconfig.defaults`), each verified with its own
`prop.py trace --trials 8` (8/8 clean): `CONFIG_WIFI_RMT_{IRAM_OPT,EXTRA_IRAM_OPT,RX_IRAM_OPT,SLP_IRAM_OPT}=n`,
`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`, `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL` 32768→49152,
`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` 16384→4096, `CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`, and
`CONFIG_ESP_TASK_WDT_TIMEOUT_S` 5→10. **Gotcha worth knowing before touching WiFi IRAM/Kconfig options
again:** under `CONFIG_ESP_WIFI_REMOTE_LIBRARY_HOSTED`, the familiar `CONFIG_ESP_WIFI_*` names (e.g.
`ESP_WIFI_IRAM_OPT`) are just mirrored aliases defined in
`managed_components/espressif__esp_wifi_remote/*/Kconfig.wifi.in` — the real settings live under
`CONFIG_WIFI_RMT_*`. Setting the `ESP_WIFI_*` alias directly in `sdkconfig`/`sdkconfig.defaults` has no
effect and silently gets recomputed back to its default on the next `idf.py reconfigure`.

Live serial capture (`prop.py trace`) still can't reliably catch a one-off intermittent panic —
opening the port doesn't cleanly resync with the exact reboot moment, and often nobody's watching
when it happens. `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH` (`sdkconfig.defaults`) persists the full
register dump + backtrace to a dedicated `coredump` partition (`partitions.csv`, 64 KB) on any panic,
independent of serial. Decode after the fact:
```powershell
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 coredump-info
```
`CONFIG_ESP_COREDUMP_FLASH_NO_OVERWRITE=y` keeps only the **first** crash after a reflash/erase — this
doubles as the flash-wear guard for a boot loop: once one dump is captured, further panics just reboot
without touching flash again. To capture a *new* dump later, erase the partition first (a normal
`idf.py flash` does **not** touch data partitions, so an old dump survives reflashes):
```powershell
parttool.py -p COM7 erase_partition --partition-table-file build/partition_table/partition-table.bin --partition-name coredump
```

See also `tools/prop.py trace`/`decode` (in the `communicator-ui` skill) for live-serial capture and
PC decoding against `build/communicator.elf` when coredump isn't enough.
