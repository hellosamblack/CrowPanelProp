#ifndef _PROP_CSI_H_
#define _PROP_CSI_H_

/* prop_csi — WiFi Channel State Information "signal environment" instrument.
 *
 * CSI is the per-subcarrier complex channel response computed for each received
 * WiFi frame. The C6 slave supports it (CONFIG_SLAVE_SOC_WIFI_CSI_SUPPORT) and
 * esp_wifi_remote forwards esp_wifi_set_csi* to it over the hosted link. While the
 * unit is STA-associated, beacons/data from the AP generate CSI frames; we fold
 * each frame's subcarrier amplitudes into a small bin column the UI renders.
 *
 * Self-adapting: if real CSI frames are arriving, the column is real; if none have
 * arrived recently (idle link, AP-only, or slave CSI unavailable) it falls back to
 * a synthetic column driven by link-RSSI variance so the instrument is always live.
 * prop_csi_is_live() reports which source is active so the panel can label it.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_CSI_BINS 32

/* Register the CSI callback, request CSI from the slave, and start the fold task.
 * Always succeeds enough to drive the synthetic fallback; returns the result of the
 * real-CSI enable attempt for logging. Call after prop_net_init (shares the C6). */
esp_err_t prop_csi_init(void);

bool prop_csi_available(void);   /* true once init ran (synthetic guarantees data) */
bool prop_csi_is_live(void);     /* true when real CSI frames are currently arriving */

/* Copy the latest PROP_CSI_BINS amplitudes (0..100) into out (benign meter race). */
void prop_csi_get_column(uint8_t *out);

#endif /* _PROP_CSI_H_ */
