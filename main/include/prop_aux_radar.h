/* prop_aux_radar — optional auxiliary radar sensors.
 *
 * Drives two plug-in 24 GHz mmWave sensors over dedicated UARTs:
 *   Sensor A: Seeed 101991030 (UART3, GPIO47/48, J2)  — HiLink binary protocol
 *   Sensor B: DFRobot SEN0395 (UART1, GPIO34/33, J10) — ASCII $JYBSS protocol
 *
 * Both sensors are optional.  If a sensor is not plugged in, or does not
 * deliver a valid frame within 5 s of init, its state stays AUX_OFFLINE.
 * prop_aux_radar_init() is non-fatal: it returns ESP_OK unless both UARTs
 * fail to install; individual sensor failures only move that sensor to
 * AUX_OFFLINE and leave the other running.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    AUX_OFFLINE = 0,  /* no frames in last 5 s, or init failed */
    AUX_CLEAR   = 1,  /* sensor alive, no target */
    AUX_PRESENT = 2,  /* sensor alive, target detected */
} aux_radar_state_t;

/* Bring up UART3 (Seeed) and UART1 (SEN0395), start background tasks.
 * Non-fatal: each sensor's state starts at AUX_OFFLINE. Returns ESP_OK unless
 * both UARTs fail (returns first error). */
esp_err_t prop_aux_radar_init(void);

/* Latest state for each sensor (AUX_OFFLINE / AUX_CLEAR / AUX_PRESENT). */
aux_radar_state_t prop_aux_radar_seeed(void);
aux_radar_state_t prop_aux_radar_sen0395(void);
