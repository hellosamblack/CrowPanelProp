/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CrowPanel prop — on-C6 CSI motion detection. Drives the (GPL-3.0) ESPectre
 * detector via the prop_espectre C glue, streams the verdict + stats to the P4
 * over esp-hosted custom RPC, and accepts runtime config from the host (no C6
 * reflash). See prop_csi_slave.h. Compiles only with the custom-RPC framework.
 */
#include "sdkconfig.h"

#ifdef CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_hosted_peer_data.h"   /* esp_hosted_send_custom_data() */
#include "prop_csi_slave.h"
#include "prop_espectre.h"          /* C glue around the ESPectre detector */

static const char *TAG = "prop_csi_slave";

/* ---- runtime config store -------------------------------------------------
 * Generated from the shared cfg list (same file the host uses). The host pushes
 * values over PROP_MSG_ID_CSI_CTRL; we store them, build the detector config,
 * and apply live changes via the prop_espectre setters. */
static const char *const s_cfg_keys[] = {
#define PROP_CSI_CFG(key, type, deflt, lo, hi, desc, opts) #key,
#include "prop_csi_cfg_list.h"
};
static int32_t s_cfg_val[] = {
#define PROP_CSI_CFG(key, type, deflt, lo, hi, desc, opts) (deflt),
#include "prop_csi_cfg_list.h"
};
#define CFG_N  ((int)(sizeof(s_cfg_keys) / sizeof(s_cfg_keys[0])))


static int cfg_index(const char *key)
{
    for (int i = 0; i < CFG_N; i++) {
        if (strcmp(key, s_cfg_keys[i]) == 0) {
            return i;
        }
    }
    return -1;
}

static int32_t cfg_get(const char *key)
{
    int i = cfg_index(key);
    return (i >= 0) ? s_cfg_val[i] : 0;
}

/* Build the detector init config from the current cfg store. The segmentation
 * threshold encodes mode in sentinels: -1=auto, -2=min, else value×1000. */
static void load_espectre_cfg(prop_espectre_cfg_t *e)
{
    int32_t t = cfg_get("segmentation_threshold");
    e->window_size          = cfg_get("segmentation_window_size");
    e->threshold_mode       = (t == -1) ? 0 : (t == -2) ? 1 : 2;
    e->threshold_milli      = (t >= 0) ? t : 0;
    e->eval_interval        = cfg_get("evaluation_interval");
    e->on_hits              = cfg_get("motion_on_hits");
    e->off_hits             = cfg_get("motion_off_hits");
    e->lowpass_en           = cfg_get("lowpass_enabled");
    e->lowpass_cutoff_milli = cfg_get("lowpass_cutoff");
    e->hampel_en            = cfg_get("hampel_enabled");
    e->hampel_window        = cfg_get("hampel_window");
    e->hampel_thresh_milli  = cfg_get("hampel_threshold");
    e->gain_lock_mode       = cfg_get("gain_lock");
    e->publish_interval     = cfg_get("publish_interval");
}

/* Apply a single changed setting to the running detector (live, no reflash). */
static void apply_live(const char *key)
{
    if (!prop_espectre_started()) {
        return;   /* not running yet; picked up at start from the store */
    }
    if (strcmp(key, "segmentation_threshold") == 0) {
        int32_t t = cfg_get("segmentation_threshold");
        prop_espectre_set_threshold((t == -1) ? 0 : (t == -2) ? 1 : 2, (t >= 0) ? t : 0);
    } else if (strcmp(key, "evaluation_interval") == 0) {
        prop_espectre_set_eval(cfg_get("evaluation_interval"));
    } else if (strcmp(key, "motion_on_hits") == 0 || strcmp(key, "motion_off_hits") == 0) {
        prop_espectre_set_hits(cfg_get("motion_on_hits"), cfg_get("motion_off_hits"));
    } else if (strcmp(key, "lowpass_enabled") == 0 || strcmp(key, "lowpass_cutoff") == 0) {
        prop_espectre_set_lowpass(cfg_get("lowpass_enabled"), cfg_get("lowpass_cutoff"));
    } else if (strcmp(key, "hampel_enabled") == 0 || strcmp(key, "hampel_window") == 0 ||
               strcmp(key, "hampel_threshold") == 0) {
        prop_espectre_set_hampel(cfg_get("hampel_enabled"), cfg_get("hampel_window"),
                                 cfg_get("hampel_threshold"));
    }
    /* acquire / traffic_generator_* / detection_algorithm: applied at start
     * (CSIManager owns the HT20 CSI config; algo switch needs a restart). */
}

/* Host -> slave config RX callback (PROP_MSG_ID_CSI_CTRL). RPC RX thread. */
static void on_csi_cfg(uint32_t msg_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)msg_id;
    (void)ctx;
    if (len != sizeof(prop_csi_cfg_t) || data == NULL) {
        ESP_LOGW(TAG, "cfg: bad len %u", (unsigned)len);
        return;
    }
    prop_csi_cfg_t m;
    memcpy(&m, data, sizeof(m));
    m.key[sizeof(m.key) - 1] = '\0';

    if (m.key[0] == '@') {                 /* transient action */
        if (strcmp(m.key, PROP_CSI_ACTION_RECAL) == 0) {
            ESP_LOGI(TAG, "NBVI recalibration requested");
            prop_espectre_recalibrate();
        } else {
            ESP_LOGW(TAG, "unknown action '%s'", m.key);
        }
        return;
    }

    int i = cfg_index(m.key);
    if (i < 0) {
        ESP_LOGW(TAG, "unknown cfg key '%s'", m.key);
        return;
    }
    s_cfg_val[i] = m.val;
    ESP_LOGI(TAG, "cfg %s=%ld", m.key, (long)m.val);
    apply_live(m.key);
}

static void csi_slave_task(void *arg)
{
    (void)arg;

    /* Start ESPectre once esp_wifi is up (the host brings it up over RPC). The
     * config comes from the store, which the host has typically already pushed
     * by now; if not, defaults apply and live edits update the detector. */
    esp_err_t last_err = ESP_FAIL;
    while (!prop_espectre_started()) {
        prop_espectre_cfg_t cfg;
        load_espectre_cfg(&cfg);
        last_err = prop_espectre_start(&cfg);
        if (last_err == ESP_OK) {
            ESP_LOGI(TAG, "ESPectre detection running on C6");
            break;
        }
        ESP_LOGW(TAG, "espectre start: %s — wifi not ready, retry 1s",
                 esp_err_to_name(last_err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    uint32_t seq = 0;
    uint32_t prev = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t now = prop_espectre_packets();
        int mv = prop_espectre_movement_milli();   /* raw metric; clamp to int16 */
        if (mv > 32767) { mv = 32767; }
        prop_csi_stats_t st = {
            .seq            = seq++,
            .frames_total   = now,
            .fps            = (uint16_t)(now - prev),
            .csi_enabled    = prop_espectre_started() ? 1 : 0,
            ._pad           = (int8_t)(prop_espectre_calibrating() ? 1 : 0),
            .enable_err     = (int32_t)last_err,
            .motion         = (uint8_t)prop_espectre_motion(),
            .movement_milli = (int16_t)mv,
            .threshold_milli = (int16_t)prop_espectre_threshold_milli(),
            .turbulence_milli = (int16_t)prop_espectre_turbulence_milli(),
            .agc_gain       = (uint8_t)prop_espectre_agc_gain(),
            .fft_gain       = (int8_t)prop_espectre_fft_gain(),
            .gain_locked    = (uint8_t)prop_espectre_gain_locked(),
        };
        prop_espectre_get_subcarriers(st.subcarriers);
        prev = now;

        ESP_LOGI(TAG, "CSI en=%d fps=%u total=%lu motion=%d move=%d thr=%d",
                 (int)st.csi_enabled, (unsigned)st.fps, (unsigned long)now,
                 (int)st.motion, (int)st.movement_milli, (int)st.threshold_milli);

        esp_hosted_send_custom_data(PROP_MSG_ID_CSI_STATS,
                                    (const uint8_t *)&st, sizeof(st));
    }
}

esp_err_t prop_csi_slave_init(void)
{
    ESP_LOGI(TAG, "on-C6 CSI motion detection starting (%d configurable settings)", CFG_N);

    /* Accept runtime config pushes from the host (no C6 reflash to tune). */
    esp_err_t err = esp_hosted_register_custom_callback(PROP_MSG_ID_CSI_CTRL,
                                                        on_csi_cfg, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "register cfg callback failed: %s", esp_err_to_name(err));
    }

    BaseType_t ok = xTaskCreate(csi_slave_task, "csi_slave", 4096, NULL, 4, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

#endif /* CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER */
