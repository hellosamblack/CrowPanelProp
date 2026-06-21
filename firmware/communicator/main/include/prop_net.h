#ifndef _PROP_NET_H_
#define _PROP_NET_H_

/* prop_net — WiFi in AP+STA mode via the onboard ESP32-C6 (esp_hosted).
 *
 * Always brings up our own hotspot (so an operator laptop can reach the prop on
 * set with zero infrastructure) AND tries to join a known upstream network. STA
 * credentials are stored in NVS so they can be changed at runtime (from the API)
 * without recompiling. Connection state is pushed to prop_engine as the LINK_*
 * indicator.
 *
 * Forked from example/V1.0/idf-code/Lesson17-Wi-Fi_function/ESP32_P4-softap_sta.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define PROP_AP_SSID     "PROP-COMMS"
#define PROP_AP_PASSWD   "scanner99"     /* >=8 chars; change for production */

/* mDNS / DHCP hostname. The prop advertises itself so it can be reached at
 * "<PROP_HOSTNAME>.local" instead of a hunt-the-IP-in-the-serial-log dance. */
#define PROP_HOSTNAME    "comm-unit-7"

/* PHY generation of an AP / link, coarsest-useful granularity. */
typedef enum {
    PROP_PHY_11B, PROP_PHY_11G, PROP_PHY_11N, PROP_PHY_11AC, PROP_PHY_11AX,
} prop_phy_t;

/* Short label for a PHY generation, e.g. "WiFi 6". */
const char *prop_phy_label(prop_phy_t phy);

/* One discovered access point (from prop_net_scan). */
typedef struct {
    char    ssid[33];
    uint8_t bssid[6];   /* AP MAC; first 3 bytes = OUI -> vendor */
    int8_t  rssi;       /* dBm; higher (closer to 0) is stronger */
    uint8_t channel;    /* primary 2.4 GHz channel */
    prop_phy_t phy;     /* highest PHY the AP advertises */
    char    sec[8];     /* security: "OPEN"/"WEP"/"WPA"/"WPA2"/"WPA3"/"OWE"/"ENT" */
    bool    secured;    /* true if it needs a password (authmode != OPEN/OWE) */
    bool    ftm;        /* AP advertises FTM responder (round-trip ranging capable) */
} prop_ap_t;

/* Vendor label for a MAC's OUI (first 3 bytes), e.g. "UBIQUITI", or NULL if the
 * prefix isn't in the curated table. Real OUI assignment, partial coverage (flavour,
 * not an exhaustive IEEE database) — mirrors the BLE Company-ID table. */
const char *prop_net_oui_vendor(const uint8_t mac[6]);

/* The unit's own uplink (cached ~1 Hz by a background task; cheap to read). */
typedef struct {
    bool    connected;
    char    ssid[33];
    uint8_t bssid[6];
    int8_t  rssi;
    uint8_t channel;
    char    phy[20];    /* negotiated PHY label, e.g. "WiFi 6 / HE20" */
    char    country[4]; /* regulatory country code, e.g. "US" */
} prop_uplink_t;

/* Copy the cached uplink dossier (info about the AP we're associated with). */
void prop_net_get_uplink(prop_uplink_t *out);

/* STA association progress, for UI feedback. */
typedef enum {
    STA_IDLE,        /* no STA credentials / not attempting */
    STA_CONNECTING,  /* association in progress */
    STA_CONNECTED,   /* got an IP */
    STA_FAILED,      /* gave up after retries (bad password / out of range) */
} prop_sta_state_t;

/* Bring up NVS + netif + WiFi in APSTA mode. Non-blocking: returns once WiFi has
 * started; STA association completes asynchronously and updates the engine link. */
esp_err_t prop_net_init(void);

/* Blocking WiFi scan (~2 s). Fills `out` with up to `max` unique APs (strongest
 * first) and returns the count, or -1 on error. Call from a task, NOT the LVGL
 * thread. Pattern mirrors the factory Setting app (esp_wifi_scan_start +
 * esp_wifi_scan_get_ap_records). */
int prop_net_scan(prop_ap_t *out, int max);

/* Blocking 2.4 GHz channel-occupancy scan (~2 s). Runs the same active scan as
 * prop_net_scan but folds the results into a per-channel occupancy histogram:
 * out[ch] (ch 1..13) gets a 0..100 index weighted by AP RSSI (strong/multiple APs
 * read taller; out[0] and out[13]+ are unused/edge). Returns the raw AP count, or
 * -1 on error. Call from a task, NOT the LVGL thread. */
#define PROP_NET_CHAN_SLOTS 14
int prop_net_scan_channels(uint8_t out[PROP_NET_CHAN_SLOTS]);

/* Apply new STA credentials and reconnect. If `remember` is true they are also
 * persisted to NVS (survive reboot/reflash); if false the connection is for this
 * session only. Pass-through for the API {"cmd":"wifi",...} command and the
 * touchscreen setup panel. */
esp_err_t prop_net_set_sta_credentials(const char *ssid, const char *password, bool remember);

/* Forget the saved STA network: clears stored credentials and disconnects. */
esp_err_t prop_net_forget(void);

/* Current STA association state (for the setup panel's status text). */
prop_sta_state_t prop_net_sta_state(void);

/* Copy the current STA IP ("0.0.0.0" if not connected) into out (>=16 bytes). */
void prop_net_get_ip(char *out, size_t out_len);

/* Current STA signal strength in dBm (negative; closer to 0 = stronger), or 0 if
 * not connected / unavailable. Crosses the SDIO bus to the C6 — call sparingly. */
int prop_net_get_rssi(void);

#endif /* _PROP_NET_H_ */
