/*
 * prop_coproc — P4 host side of the esp-hosted custom-RPC link to the C6.
 *
 * The C6 slave runs Wi-Fi CSI capture locally (where CSI works natively, unlike
 * over the hosted RPC layer) and ships results north over esp-hosted custom RPC.
 * This module registers the host-side receive callbacks. SPIKE stage: it only
 * receives a 1 Hz CSI capture-stats heartbeat to validate the channel and gauge
 * C6 headroom; later it feeds a real motion digest into prop_csi.
 *
 * The wire structs/IDs below MUST stay byte-identical to the slave's
 * c6_slave/slave/main/prop_csi_slave.h (both ends are little-endian RISC-V).
 */
#ifndef PROP_COPROC_H
#define PROP_COPROC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Keep in sync with c6_slave/slave/main/prop_csi_slave.h */
#define PROP_MSG_ID_CSI_STATS  0x43534931u  /* 'CSI1' slave -> host: capture stats */
#define PROP_MSG_ID_CSI_CTRL   0x43534943u  /* 'CSIC' host -> slave: control (unused in spike) */

/* ---- host -> slave config (PROP_MSG_ID_CSI_CTRL). Generic key/value: any
 * setting in prop_csi_cfg_list.h can be set at runtime. Keep this struct and
 * the cfg list byte-identical with c6_slave/slave/main/. --------------------- */
#define PROP_CSI_KEYLEN  32

/* Bits for the "acquire" setting — which PPDU types yield CSI frames. */
#define PROP_CSI_ACQ_LEGACY  0x01
#define PROP_CSI_ACQ_HT20    0x02
#define PROP_CSI_ACQ_HT40    0x04

/* Reserved action keys (not in the cfg list, not persisted): '@'-prefixed. */
#define PROP_CSI_ACTION_RECAL  "@recal"   /* re-run NBVI subcarrier calibration */

typedef struct __attribute__((packed)) {
    char    key[PROP_CSI_KEYLEN];  /* NUL-terminated; a cfg-list key or an @action */
    int32_t val;                   /* int/enum/bool, or float×1000 — per the key's type */
} prop_csi_cfg_t;

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t frames_total;
    uint16_t fps;
    uint16_t bad_len;
    int16_t  last_rssi;
    uint16_t last_csi_len;
    uint8_t  csi_enabled;   /* 1 once CSI detection is running on the C6 */
    int8_t   _pad;
    int32_t  enable_err;    /* last esp_err_t from the slave's start attempt */
    uint8_t  motion;          /* ESPectre verdict: 1 = motion detected */
    uint8_t  _pad2;
    int16_t  movement_milli;  /* motion metric ×1000 */
    int16_t  threshold_milli; /* active detection threshold ×1000 */
    int16_t  turbulence_milli;/* raw per-packet spatial disturbance ×1000 */
    uint8_t  agc_gain;        /* locked AGC gain */
    int8_t   fft_gain;        /* locked FFT gain */
    uint8_t  gain_locked;     /* 1 if gain lock succeeded */
    uint8_t  _pad3;
    uint8_t  subcarriers[12]; /* NBVI-selected subcarrier band (fingerprint) */
} prop_csi_stats_t;

/* Register the custom-RPC receive callbacks. Call AFTER prop_net_init() — the
 * SDIO transport must be up. NON-fatal: returns an error rather than aborting. */
esp_err_t prop_coproc_init(void);

/* Latest CSI capture stats from the C6, plus how long ago they arrived.
 * Returns false if no stats have ever been received. For /state + diagnostics. */
bool prop_coproc_get_csi_stats(prop_csi_stats_t *out, uint32_t *age_ms);

/* Apply + PERSIST one setting by name: validates/clamps against the cfg list,
 * stores it in NVS (survives reflash), and pushes it to the C6. This is the
 * configure-without-reflash entry point used by the HTTP /cmd handler.
 * `val` is the raw stored form (float settings are ×1000). Returns
 * ESP_ERR_NOT_FOUND for an unknown key, ESP_ERR_INVALID_ARG if out of range. */
esp_err_t prop_coproc_csi_set(const char *key, int32_t val);

/* Fire a transient action key (e.g. PROP_CSI_ACTION_RECAL) — sent, not persisted. */
esp_err_t prop_coproc_csi_action(const char *action);

/* Push a setting to the C6 WITHOUT persisting it (no NVS write). For values that
 * change continuously — e.g. the adaptive auto-threshold — so we don't wear flash;
 * such values just re-derive at runtime. Not range-validated (caller clamps). */
esp_err_t prop_coproc_csi_push(const char *key, int32_t val);

/* Re-push all persisted settings to the C6. Called once stats first arrive
 * (slave is alive) so a freshly-OTA'd or reset C6 picks up the saved config. */
void prop_coproc_push_settings(void);

/* Enumerate the config surface for /state / UI. Returns the number of settings;
 * for index i (< count), fills the key, current (stored-or-default) value, type
 * char ('I','B','E','F'), and [lo,hi]. Any out-pointer may be NULL. */
int  prop_coproc_csi_count(void);
bool prop_coproc_csi_describe(int i, const char **key, int32_t *val,
                              char *type, int32_t *lo, int32_t *hi,
                              const char **desc, const char **opts);

#ifdef __cplusplus
}
#endif

#endif /* PROP_COPROC_H */
