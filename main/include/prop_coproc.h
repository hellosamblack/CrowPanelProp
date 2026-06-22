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

/* ---- host -> slave control (PROP_MSG_ID_CSI_CTRL). Keep in sync with
 * c6_slave/slave/main/prop_csi_slave.h. ------------------------------------ */
enum {
    PROP_CSI_CMD_SET_ACQUIRE   = 1,
    PROP_CSI_CMD_SET_THRESHOLD = 2,
    PROP_CSI_CMD_SET_EVAL      = 3,
    PROP_CSI_CMD_SET_HITS      = 4,
    PROP_CSI_CMD_RECAL         = 5,
};
#define PROP_CSI_ACQ_LEGACY  0x01
#define PROP_CSI_ACQ_HT20    0x02
#define PROP_CSI_ACQ_HT40    0x04

typedef struct __attribute__((packed)) {
    uint8_t  cmd;
    uint8_t  flags;
    uint16_t u16;
    int32_t  i32;
} prop_csi_ctrl_t;

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t frames_total;
    uint16_t fps;
    uint16_t bad_len;
    int16_t  last_rssi;
    uint16_t last_csi_len;
    uint8_t  csi_enabled;   /* 1 once esp_wifi_set_csi(true) succeeded on the C6 */
    int8_t   _pad;
    int32_t  enable_err;    /* last esp_err_t from the slave's enable attempt */
} prop_csi_stats_t;

/* Register the custom-RPC receive callbacks. Call AFTER prop_net_init() — the
 * SDIO transport must be up. NON-fatal: returns an error rather than aborting. */
esp_err_t prop_coproc_init(void);

/* Latest CSI capture stats from the C6, plus how long ago they arrived.
 * Returns false if no stats have ever been received. For /state + diagnostics. */
bool prop_coproc_get_csi_stats(prop_csi_stats_t *out, uint32_t *age_ms);

/* Send one control message to the C6 (PROP_MSG_ID_CSI_CTRL). Low-level. */
esp_err_t prop_coproc_csi_ctrl(uint8_t cmd, uint8_t flags, uint16_t u16, int32_t i32);

/* Apply + PERSIST an ESPectre/CSI setting: stores it in NVS (survives reflash)
 * and pushes it to the C6 — the configure-without-reflash entry point used by
 * the HTTP /cmd handler. `cmd` is a PROP_CSI_CMD_*; value semantics match it
 * (acquire flags, threshold x1000, eval packets, packed hits; ignored for RECAL). */
esp_err_t prop_coproc_csi_set(uint8_t cmd, int32_t value);

/* Re-push all persisted CSI settings to the C6. Called once stats first arrive
 * (slave is alive) so a freshly-OTA'd or reset C6 picks up the saved config. */
void prop_coproc_push_settings(void);

#ifdef __cplusplus
}
#endif

#endif /* PROP_COPROC_H */
