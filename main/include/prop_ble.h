#ifndef _PROP_BLE_H_
#define _PROP_BLE_H_

/* prop_ble — passive BLE scan via the ESP32-C6, feeding the CONTACT SIGNATURES
 * instrument.
 *
 * The C6 esp_hosted slave already runs the BLE controller (HCI over the shared
 * SDIO link / VHCI). The P4 host runs the NimBLE host stack and a continuous
 * passive GAP discovery scan; discovered advertisers are cached into a small,
 * bounded, age-out table (mirrors the prop_mic / prop_net cached-poll pattern so
 * the UI reads it cheaply). For each contact we keep MAC, RSSI, advertised name
 * and the manufacturer Company-ID (mapped to a prop label).
 *
 * Non-fatal: if BLE can't be brought up (controller/RAM), prop_ble_available()
 * stays false and the panel shows "-- BLE OFFLINE --" — the prop never hangs.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_BLE_MAX  24      /* bounded contact table (LRU age-out by last_seen) */
#define PROP_BLE_NONE 0xFFFF  /* company_id sentinel: no manufacturer data */

typedef struct {
    uint8_t  mac[6];
    int8_t   rssi;
    int8_t   tx_power;     /* advertised TX power (dBm), or 127 if not present */
    char     name[20];     /* advertised local name, "" if anonymous */
    uint16_t company_id;   /* BT SIG Company ID, or PROP_BLE_NONE */
    uint16_t appearance;   /* BT SIG appearance (device class), 0 if absent */
    uint8_t  uuid[16];     /* most-identifying advertised service UUID (LE bytes) */
    uint8_t  uuid_len;     /* 0 = none, else 2 / 4 / 16 bytes */
    uint32_t last_seen;    /* esp_timer ms at last advert */
} prop_ble_dev_t;

#define PROP_BLE_TXPWR_NONE 127

/* This scanner only realistically hears devices within ~10 m; distance estimates
 * are clamped here, and an implausible advertised TX power is overridden. */
#define PROP_BLE_RANGE_MAX_M 10.0f
#define PROP_BLE_TXPWR_TYP   (-52)   /* assumed TX power (dBm) when none / implausible */

/* Bring up the NimBLE host + start the passive scan task. Returns an error
 * (non-fatal) if BLE is unavailable. Call after prop_net_init (shares the C6). */
esp_err_t prop_ble_init(void);

bool prop_ble_available(void);

/* Copy up to `max` tracked contacts (strongest RSSI first) into out. Returns the
 * count copied. Cheap cached read (benign-race-free: copied under a short lock). */
int prop_ble_get_devices(prop_ble_dev_t *out, int max);

/* Aggregates over the live table. Any out pointer may be NULL.
 *   count     — contacts currently tracked
 *   strongest — strongest RSSI seen (0 if none)
 *   named     — contacts advertising a name
 *   known     — contacts with a recognised Company-ID */
void prop_ble_get_summary(int *count, int8_t *strongest, int *named, int *known);

/* Prop label for a Company-ID (e.g. "APPLE"), or NULL if unrecognised. */
const char *prop_ble_company_label(uint16_t company_id);

/* Device-class label from a BT SIG appearance value (e.g. "WATCH"), or NULL if
 * the advert carried no appearance. */
const char *prop_ble_appearance_label(uint16_t appearance);

/* Rough distance estimate (metres) from RSSI via the log-distance path-loss model.
 * If tx_power is a real value (not PROP_BLE_TXPWR_NONE) it sharpens the estimate.
 * Implausible advertised TX powers are overridden (see prop_ble_calib_txpower) and
 * the result is clamped to PROP_BLE_RANGE_MAX_M — the scanner's realistic range.
 * It's a prop readout, not a tape measure — accurate to "near / across the room / far". */
float prop_ble_distance_m(int8_t rssi, int8_t tx_power);

/* The TX power actually used for ranging: the advertised value when it is present
 * and plausible, otherwise PROP_BLE_TXPWR_TYP. Lets the UI show "advertised → used". */
int8_t prop_ble_calib_txpower(int8_t tx_power);

#endif /* _PROP_BLE_H_ */
