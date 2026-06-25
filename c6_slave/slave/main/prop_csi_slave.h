/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CrowPanel prop — on-C6 CSI capture (SPIKE).
 *
 * Headroom gate for running Wi-Fi CSI motion detection on the ESP32-C6 itself
 * (where esp_wifi + CSI run natively) and shipping only a small result to the
 * P4 host over esp-hosted custom RPC. This SPIKE only *enables CSI + counts
 * frames*; it does no DSP. Its job is to answer one question: does capturing
 * CSI on the C6 starve WiFi/BLE on this single 160 MHz core?
 *
 * See the project plan: real-CSI-motion-sensing (port ESPectre into the slave).
 */
#ifndef PROP_CSI_SLAVE_H
#define PROP_CSI_SLAVE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Custom-RPC message IDs shared with the P4 host (main/prop_coproc.c). */
#define PROP_MSG_ID_CSI_STATS  0x43534931u  /* 'CSI1' slave -> host: capture stats */
#define PROP_MSG_ID_CSI_CTRL   0x43534943u  /* 'CSIC' host -> slave: control (unused in spike) */

/* ---- host -> slave config (PROP_MSG_ID_CSI_CTRL) --------------------------
 * Generic key/value: the P4 sets any setting in prop_csi_cfg_list.h at runtime
 * (no C6 reflash). The host owns the values in NVS and re-pushes on boot/change.
 * Keep this struct + the cfg list byte-identical with main/include/. */
#define PROP_CSI_KEYLEN  32

/* Bits for the "acquire" setting — which PPDU types yield CSI frames. Beacons
 * are legacy OFDM, so PROP_CSI_ACQ_LEGACY gives a steady stream from an idle
 * link; HT20/HT40 capture data frames. */
#define PROP_CSI_ACQ_LEGACY  0x01
#define PROP_CSI_ACQ_HT20    0x02
#define PROP_CSI_ACQ_HT40    0x04

/* Reserved transient action keys ('@'-prefixed, not in the cfg list). */
#define PROP_CSI_ACTION_RECAL  "@recal"

typedef struct __attribute__((packed)) {
    char    key[PROP_CSI_KEYLEN];  /* cfg-list key or an @action */
    int32_t val;                   /* int/enum/bool, or float×1000 — per the key's type */
} prop_csi_cfg_t;

/* Stats heartbeat sent ~1 Hz to the host. Packed so the P4 can cast the RPC
 * payload straight to this struct (both are little-endian RISC-V). */
typedef struct __attribute__((packed)) {
    uint32_t seq;           /* monotonically increasing heartbeat counter */
    uint32_t frames_total;  /* total CSI frames captured since boot */
    uint16_t fps;           /* CSI frames in the last 1 s window */
    uint16_t bad_len;       /* frames whose buf len != expected HT20 length */
    int16_t  last_rssi;     /* rx RSSI of the most recent CSI frame */
    uint16_t last_csi_len;  /* buf len of the most recent CSI frame */
    uint8_t  csi_enabled;   /* 1 once CSI detection is running */
    int8_t   _pad;
    int32_t  enable_err;    /* last esp_err_t from the start attempt (0 = OK) */
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

/* Start the spike: enable CSI locally and spawn the stats task.
 * Call from app_main() after the hosted coprocessor is up. */
esp_err_t prop_csi_slave_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PROP_CSI_SLAVE_H */
