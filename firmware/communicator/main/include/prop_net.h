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

/* One discovered access point (from prop_net_scan). */
typedef struct {
    char    ssid[33];
    int8_t  rssi;       /* dBm; higher (closer to 0) is stronger */
    bool    secured;    /* true if it needs a password (authmode != OPEN/OWE) */
} prop_ap_t;

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

#endif /* _PROP_NET_H_ */
