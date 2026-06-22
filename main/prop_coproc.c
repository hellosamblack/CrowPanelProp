/*
 * prop_coproc — P4 host side of the esp-hosted custom-RPC link to the C6.
 * See include/prop_coproc.h.
 */
#include "prop_coproc.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_hosted.h"   /* esp_hosted_register_custom_callback() (via esp_hosted_misc.h) */
#include "prop_settings.h"

static const char *TAG = "prop_coproc";

static prop_csi_stats_t s_stats;
static int64_t s_stats_us = -1;   /* esp_timer time of last stats; <0 = none yet */
static bool s_settings_pushed;    /* have we pushed saved settings since the slave came up? */

/* NVS keys + defaults for the runtime-configurable CSI/ESPectre settings.
 * These live on the P4 (survive reflash) and are pushed to the C6 over RPC, so
 * the C6 never needs reflashing to change them. */
#define NVS_CSI_ACQ        "csi_acq"     /* acquire bitmask (PROP_CSI_ACQ_*) */
#define NVS_CSI_THRESH     "csi_thresh"  /* segmentation threshold, x1000 */
#define NVS_CSI_EVAL       "csi_eval"    /* evaluation_interval, packets */
#define NVS_CSI_HITS       "csi_hits"    /* (on_hits<<8)|off_hits */

#define CSI_ACQ_DEFAULT    (PROP_CSI_ACQ_LEGACY | PROP_CSI_ACQ_HT20) /* beacons + data → steady stream */
#define CSI_THRESH_DEFAULT 1500u   /* 1.5 (placeholder until ESPectre is tuned) */
#define CSI_EVAL_DEFAULT   16u
#define CSI_HITS_DEFAULT   0x0305u  /* on=3, off=5 */

static const char *nvs_key_for(uint8_t cmd)
{
    switch (cmd) {
    case PROP_CSI_CMD_SET_ACQUIRE:   return NVS_CSI_ACQ;
    case PROP_CSI_CMD_SET_THRESHOLD: return NVS_CSI_THRESH;
    case PROP_CSI_CMD_SET_EVAL:      return NVS_CSI_EVAL;
    case PROP_CSI_CMD_SET_HITS:      return NVS_CSI_HITS;
    default:                         return NULL;  /* RECAL is transient, not persisted */
    }
}

/* Pack a value into the right ctrl field for its command and send it. */
static esp_err_t send_setting(uint8_t cmd, int32_t value)
{
    switch (cmd) {
    case PROP_CSI_CMD_SET_ACQUIRE:   return prop_coproc_csi_ctrl(cmd, (uint8_t)value, 0, 0);
    case PROP_CSI_CMD_SET_THRESHOLD: return prop_coproc_csi_ctrl(cmd, 0, 0, value);
    case PROP_CSI_CMD_SET_EVAL:      return prop_coproc_csi_ctrl(cmd, 0, (uint16_t)value, 0);
    case PROP_CSI_CMD_SET_HITS:      return prop_coproc_csi_ctrl(cmd, 0, (uint16_t)value, 0);
    case PROP_CSI_CMD_RECAL:         return prop_coproc_csi_ctrl(cmd, 0, 0, 0);
    default:                         return ESP_ERR_INVALID_ARG;
    }
}

/* esp-hosted custom-RPC RX callback. Runs in the RPC RX thread — keep it cheap. */
static void on_csi_stats(uint32_t msg_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)msg_id;
    (void)ctx;
    if (len != sizeof(prop_csi_stats_t) || data == NULL) {
        ESP_LOGW(TAG, "CSI stats: unexpected len %u (want %u)",
                 (unsigned)len, (unsigned)sizeof(prop_csi_stats_t));
        return;
    }
    memcpy(&s_stats, data, sizeof(s_stats));
    s_stats_us = esp_timer_get_time();

    ESP_LOGI(TAG, "C6 CSI: en=%u stage=%d err=%d fps=%u total=%lu bad_len=%u rssi=%d len=%u",
             (unsigned)s_stats.csi_enabled, (int)s_stats._pad, (int)s_stats.enable_err,
             (unsigned)s_stats.fps, (unsigned long)s_stats.frames_total,
             (unsigned)s_stats.bad_len, (int)s_stats.last_rssi,
             (unsigned)s_stats.last_csi_len);
}

esp_err_t prop_coproc_init(void)
{
    esp_err_t err = esp_hosted_register_custom_callback(PROP_MSG_ID_CSI_STATS,
                                                        on_csi_stats, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register CSI-stats callback failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "listening for C6 CSI custom-RPC (msg 0x%08x)",
             (unsigned)PROP_MSG_ID_CSI_STATS);
    return ESP_OK;
}

bool prop_coproc_get_csi_stats(prop_csi_stats_t *out, uint32_t *age_ms)
{
    if (s_stats_us < 0) {
        return false;
    }
    if (out) {
        *out = s_stats;
    }
    if (age_ms) {
        *age_ms = (uint32_t)((esp_timer_get_time() - s_stats_us) / 1000);
    }
    return true;
}
