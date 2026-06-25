# c6_slave — custom ESP32-C6 esp-hosted slave firmware

This is the firmware that runs on the prop's **ESP32-C6** companion radio. It is a
**copied-out, editable** version of the esp-hosted-mcu *slave* project (the C6 normally
runs the stock image, which the P4 never touches). We copy it out because the IDF
component manager overwrites `managed_components/` in place — edits there don't survive.

The C6 stays a **full esp-hosted slave** (it still provides the P4's Wi-Fi + BLE). On top
of that it runs **on-C6 Wi-Fi CSI capture**: CSI works natively here (the real `esp_wifi`
driver runs on the C6), but esp-hosted's RPC layer never forwards CSI to the P4. So instead
of shipping raw CSI north, the C6 captures + processes it locally and sends only a small
result to the P4 over esp-hosted **custom RPC**. This is what feeds the prop's SIGNAL ENV
instrument with *real* motion data instead of the synthetic fallback.

## Provenance

- Source: `espressif/esp-hosted-mcu` slave, **v2.12.9** (commit `09d9e983…`), copied from
  `managed_components/espressif__esp_hosted/{slave,common}` at that pin. Layout preserved so
  the slave's `../common` reference resolves (`c6_slave/{slave,common}`).
- To re-sync to a newer esp-hosted: re-copy `slave/` + `common/` and re-apply the prop diffs
  (see "Prop additions" below — they're small and self-contained).

## Prop additions (the only files we changed)

- `slave/main/prop_csi_slave.{c,h}` — on-C6 CSI capture. **Spike stage:** enables CSI and
  counts frames, shipping a 1 Hz stats heartbeat to the P4 (`PROP_MSG_ID_CSI_STATS`) to gauge
  whether capture starves Wi-Fi/BLE on the single 160 MHz core. The ESPectre MVS/NBVI pipeline
  gets added here once the spike passes the headroom gate.
- `slave/main/CMakeLists.txt` — adds `prop_csi_slave.c` to the build (under the existing
  `CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER` gate, since it uses the custom-RPC framework).
- `slave/main/esp_hosted_coprocessor.c` — calls `prop_csi_slave_init()` from `app_main()`.

The P4 host side lives in the main project: `main/prop_coproc.c` registers the custom-RPC
receiver; the wire struct/IDs are mirrored in `main/include/prop_coproc.h` and
`slave/main/prop_csi_slave.h` — **keep those two byte-identical.**

## License — GPL-3.0 boundary

The full ESPectre pipeline (added at the port stage) is **GPL-3.0**. Once it is vendored here,
**this C6 slave image is a GPL-3.0 derivative** and its source must be published under GPL-3.0.
That is fine — this tree is committed. The boundary is the SDIO link: the **P4 host firmware
links no ESPectre code** (it only exchanges bytes over the documented custom-RPC interface), so
it stays separate. Do **not** `#include` any ESPectre header into the main project's `main/`.

## Build

```bash
. ~/.local/esp/esp-idf/export.sh           # ESP-IDF 6.0.1
cd c6_slave/slave
idf.py set-target esp32c6
idf.py build
```

The base `sdkconfig.defaults` selects the SDIO transport (matches the P4 host's SDIO config).
Build artefacts (`build/`, generated `sdkconfig`, fetched `managed_components/`) are gitignored.

## Flash — over SDIO from the P4 (no C6 UART needed)

The C6 is flashed **from the P4 over the SDIO link** using esp-hosted's co-processor OTA API
(`esp_hosted_slave_ota_begin/_write/_end/_activate`). This board has an established, proven path:
Elecrow's **`host_performs_slave_ota`** example (vendor repo →
`example/V1.0/Upgrade P4 to C6 firmware/host_performs_slave_ota.zip`), whose `sdkconfig.defaults`
is already pinned to this board's SDIO (1-bit, 40 MHz, CMD=19/CLK=18). It is a standalone P4 app
that reads a slave image (LITTLEFS partition by default, or HTTPS / flash partition) and streams it
to the C6. Point it at this project's `c6_slave/slave/build/network_adapter.bin`.

Spike run sheet:
1. Build both images: this slave (`idf.py -C c6_slave/slave build`) and the prop (`idf.py build`).
2. Flash Elecrow `host_performs_slave_ota` to the P4 with our `network_adapter.bin` as its source
   → it OTAs the C6 over SDIO and activates it.
3. Flash the prop (`idf.py -p <port> flash`) back onto the P4.
4. Boot + `idf.py -p <port> monitor`: watch for `prop_coproc: C6 CSI: fps=… bad_len=… rssi=…`
   (the P4 receiving the C6's 1 Hz stats) and confirm Wi-Fi/BLE come up with no `HS_MP` mempool loop.

Optional future polish: fold the slave-OTA into the prop itself (a `/cmd slaveota` verb that pulls
`network_adapter.bin` over HTTP and runs `esp_hosted_slave_ota_*`), so updating the C6 doesn't need
swapping the P4 firmware. Not needed for the spike. Direct C6 UART flashing is the recovery fallback.
