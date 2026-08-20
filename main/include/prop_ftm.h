#ifndef _PROP_FTM_H_
#define _PROP_FTM_H_

/* prop_ftm — WiFi 802.11mc Fine Time Measurement (FTM) ranging, driving the
 * RANGE instrument.
 *
 * esp-hosted's stock RPC never wires FTM through to the C6 (see the project
 * plan) — instead the C6 runs the real esp_wifi FTM initiator locally and
 * ships only the aggregate per-session result north over esp-hosted custom
 * RPC (main/prop_coproc.c). This module owns the per-BSSID tracking table:
 * a background task periodically re-scans (per-BSSID, NOT the SSID-deduped
 * prop_net_scan used by the WIFI connect panel — a mesh network's nodes must
 * stay distinct), issues FTM requests to APs that advertise responder support,
 * and applies progressive backoff to APs that don't reply, so the (common)
 * case of an environment with few or no FTM-capable APs doesn't hammer them.
 *
 * Non-fatal: if the C6 co-processor link isn't up, prop_ftm_available() stays
 * true but the table simply never gets fresh scans/results — the prop never
 * hangs waiting on it.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_FTM_TABLE_MAX 32   /* bounded table (LRU age-out by last_seen_ms) */

typedef enum {
    PROP_FTM_ENTRY_NEVER = 0,   /* seen in a scan, never probed yet */
    PROP_FTM_ENTRY_OK,          /* last probe succeeded */
    PROP_FTM_ENTRY_NO_REPLY,    /* last probe failed (peer didn't respond / rejected) */
    PROP_FTM_ENTRY_TIMEOUT,     /* last probe timed out */
    PROP_FTM_ENTRY_BUSY,        /* C6 radio was mid-session; will retry */
} prop_ftm_entry_status_t;

typedef struct {
    uint8_t  bssid[6];
    char     ssid[33];        /* best-effort label from the last scan that saw it */
    bool     ftm_capable;     /* responder bit from the scan record */
    int8_t   rssi;
    uint8_t  channel;
    prop_ftm_entry_status_t status;
    int32_t  last_dist_cm;    /* -1 if never successfully measured */
    uint32_t last_seen_ms;    /* esp_timer ms this BSSID last appeared in a scan */
    uint32_t next_probe_ms;   /* backoff: don't probe again before this (esp_timer ms) */
    uint8_t  consecutive_fails;
} prop_ftm_entry_t;

/* Start the background scan+range task. Call after prop_coproc_init() (needs
 * the custom-RPC transport registered) — order vs prop_ble_init/prop_csi_init
 * doesn't matter. */
esp_err_t prop_ftm_init(void);

bool prop_ftm_available(void);

/* Tells the ranging task whether the RANGE panel is currently on screen. A scan cycle
 * is a FULL WiFi scan, which puts the shared C6 radio off-channel for a second or more;
 * running it unconditionally taxes every other network user on the board for the whole
 * boot (see the comment on the gating state in prop_ftm.c). So the cycle only runs while
 * something is displaying its output, with a short grace window so navigating away and
 * back doesn't cost a repopulate. Safe from the UI thread under the LVGL lock — it only
 * sets a flag and an event bit. Same contract as prop_lidar_set_active. */
void prop_ftm_set_active(bool active);

/* Copy up to `max` tracked entries (FTM-capable first, then strongest RSSI)
 * into out. Cheap cached read (copied under a short critical section, mirrors
 * prop_ble_get_devices). Returns the count copied. */
int prop_ftm_get_table(prop_ftm_entry_t *out, int max);

/* Aggregates over the live table. Any out pointer may be NULL.
 *   tracked   — BSSIDs currently tracked (seen recently)
 *   capable   — of those, how many advertise FTM responder support
 *   ranged_ok — of the capable ones, how many have a valid last_dist_cm */
void prop_ftm_get_summary(int *tracked, int *capable, int *ranged_ok);

#endif /* _PROP_FTM_H_ */
