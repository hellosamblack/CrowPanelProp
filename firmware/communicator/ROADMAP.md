# Communicator prop — roadmap / future features

Ideas not yet built. See `CLAUDE.md` for current architecture.

## 802.15.4 sensing — Thread / Zigbee scan (Tier 3, future)

The onboard ESP32-C6 has a third radio beyond WiFi + BLE: an **802.15.4** PHY, and
esp-hosted-mcu exposes **OpenThread RCP** and **Zigbee** over the hosted link. That
opens a whole new "RF band" for the SENSORS rail group — detecting/scanning nearby
**Thread / Zigbee smart-home devices** (the way CONTACTS does BLE and RF BAND does
WiFi). A natural fourth SENSORS instrument.

**Status:** not started. **Effort/risk: high.**
- Requires bringing up the OpenThread (or Zigbee) host stack on the P4 — real
  internal-RAM cost on top of WiFi + NimBLE + LVGL (we run ~332 KB internal free
  today; budget carefully and re-verify the boot RAM gate).
- 802.15.4 must time-share the single C6 radio with WiFi + BLE coexistence — needs
  testing for starvation / scan-window tuning.
- Likely needs `CONFIG_ESP_HOSTED_OT_RCP_ENABLED` / Zigbee host configs + the matching
  C6 slave firmware capability (verify the flashed slave supports the RCP path, the way
  we found real WiFi CSI is *not* implemented in esp-hosted — see the
  `c6-radio-data-extraction` notes).

**Suggested approach when picked up:** a Phase-0-style enablement spike first (does the
host stack come up + does the board still boot within the RAM budget?) before any UI,
mirroring how BLE/CSI were de-risked.

## Confirmed dead ends (don't chase)

- **WiFi CSI** (real channel data): `esp_wifi_set_csi*` are unimplemented weak stubs in
  esp-hosted-mcu 2.12.9 (returns `ESP_ERR_NOT_SUPPORTED`); never added in any released
  version. The SIGNAL ENV panel uses a synthetic RSSI-variance trace instead.
- **WiFi promiscuous / monitor mode** and **FTM ranging**: also unimplemented over the
  hosted transport (signature-only entries in the esp-hosted API doc comment, no RPC).
