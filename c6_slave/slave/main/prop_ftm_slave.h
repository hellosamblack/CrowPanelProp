/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CrowPanel prop — on-C6 WiFi FTM (802.11mc) ranging.
 *
 * esp-hosted's stock RPC never wires FTM through (the host-side glue and the
 * slave's RPC dispatch table both lack it — see the project plan). The C6
 * runs the real esp_wifi FTM initiator locally (that's where esp_wifi and
 * the radio actually live) and ships only the aggregate per-session result
 * to the P4 host over esp-hosted custom RPC, one session at a time.
 */
#ifndef PROP_FTM_SLAVE_H
#define PROP_FTM_SLAVE_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Custom-RPC message IDs shared with the P4 host (main/include/prop_coproc.h). */
#define PROP_MSG_ID_FTM_REQ    0x46544d52u  /* 'FTMR' host -> slave: initiate one FTM session */
#define PROP_MSG_ID_FTM_RESULT 0x46544d53u  /* 'FTMS' slave -> host: session result */

typedef enum {
    PROP_FTM_OK = 0,
    PROP_FTM_FAIL,
    PROP_FTM_TIMEOUT,
    PROP_FTM_BUSY,
} prop_ftm_status_t;

/* Keep byte-identical with main/include/prop_coproc.h. */
typedef struct __attribute__((packed)) {
    uint8_t  bssid[6];
    uint8_t  channel;
    uint8_t  frm_count;      /* FTM frames/burst; 0 = driver default */
    uint16_t burst_period;   /* 100ms units; 0 = single-shot */
    uint16_t req_id;         /* host-assigned, echoed back for correlation */
} prop_ftm_req_t;

typedef struct __attribute__((packed)) {
    uint8_t  bssid[6];
    uint16_t req_id;
    uint8_t  status;         /* prop_ftm_status_t */
    uint8_t  fail_reason;    /* raw wifi_ftm_status_t on failure, else 0 */
    int32_t  dist_est_cm;    /* wifi_event_ftm_report_t.dist_est: one-way distance, cm */
    int32_t  rtt_est_ns;     /* wifi_event_ftm_report_t.rtt_est: round-trip time, nanoseconds */
    int8_t   rssi;
    uint8_t  _pad[3];
} prop_ftm_result_t;

/* Start the FTM worker: registers the request callback + FTM report event
 * handler and spawns the session task. Call from app_main() after the hosted
 * coprocessor is up (see esp_hosted_coprocessor.c). */
esp_err_t prop_ftm_slave_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PROP_FTM_SLAVE_H */
