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
#include "prop_traffic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "prop_coproc";

static prop_csi_stats_t s_stats;
static int64_t s_stats_us = -1;   /* esp_timer time of last stats; <0 = none yet */
static volatile bool s_slave_alive;  /* a heartbeat has arrived since boot */

/* FTM: at most one outstanding request at a time (the C6 only runs one
 * session at a time), so a volatile cache + poll is enough — no semaphore
 * needed. Mirrors the CSI stats cache above. */
static volatile prop_ftm_result_t s_ftm_result;
static volatile uint32_t s_ftm_result_seq;   /* bumped on every RX */

/* The runtime-configurable settings, generated from the shared cfg list. These
 * live on the P4 (NVS survives reflash) and are pushed to the C6 over RPC, so
 * the C6 never needs reflashing to change them. */
typedef struct { const char *key; char type; int32_t def, lo, hi; const char *desc, *opts; } csi_setting_t;

static const csi_setting_t s_settings[] = {
#define PROP_CSI_CFG(key, type, deflt, lo, hi, desc, opts) { #key, type, (deflt), (lo), (hi), (desc), (opts) },
#include "prop_csi_cfg_list.h"
};
#define CSI_NSETTINGS  ((int)(sizeof(s_settings) / sizeof(s_settings[0])))

/* NVS key for setting i: index-based ("csiNN") to stay within NVS's 15-char limit
 * regardless of how long the human-readable setting name is. */
static void nvs_key_for_index(int i, char out[8])
{
    out[0] = 'c'; out[1] = 's'; out[2] = 'i';
    out[3] = (char)('0' + (i / 10));
    out[4] = (char)('0' + (i % 10));
    out[5] = '\0';
}

/* Some settings also have a HOST-side effect (the C6 can't generate IP traffic
 * itself — lwip lives on the P4), so apply those here in addition to pushing. */
static void apply_host_side(const char *key, int32_t val)
{
    if (strcmp(key, "traffic_generator_rate") == 0) {
        prop_traffic_set_rate((int)val);
    } else if (strcmp(key, "traffic_generator_mode") == 0) {
        prop_traffic_set_mode((int)val);
    }
}

/* Send one generic key/value config message to the C6 (+ any host-side effect). */
static esp_err_t send_cfg(const char *key, int32_t val)
{
    apply_host_side(key, val);

    prop_csi_cfg_t m;
    memset(&m, 0, sizeof(m));
    strncpy(m.key, key, sizeof(m.key) - 1);
    m.val = val;
    return esp_hosted_send_custom_data(PROP_MSG_ID_CSI_CTRL,
                                       (const uint8_t *)&m, sizeof(m));
}

/* Stored-or-default value of setting i. */
static int32_t setting_value(int i)
{
    char k[8];
    nvs_key_for_index(i, k);
    uint32_t v;
    prop_settings_get_u32(k, &v, (uint32_t)s_settings[i].def);
    return (int32_t)v;
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

    ESP_LOGI(TAG, "C6 CSI: en=%u cal=%d fps=%u total=%lu MOTION=%u move=%d thr=%d",
             (unsigned)s_stats.csi_enabled, (int)s_stats._pad,
             (unsigned)s_stats.fps, (unsigned long)s_stats.frames_total,
             (unsigned)s_stats.motion, (int)s_stats.movement_milli,
             (int)s_stats.threshold_milli);

    /* First heartbeat since boot ⇒ the slave is alive. Flag it; the push task
     * (NOT this RX callback) sends the saved config — pushing 16 messages from
     * inside the RPC RX thread stalls the hosted transport (and with it WiFi). */
    s_slave_alive = true;
}

/* esp-hosted custom-RPC RX callback for FTM results. RPC RX thread — keep it
 * cheap (just cache + bump the sequence counter, same discipline as on_csi_stats). */
static void on_ftm_result(uint32_t msg_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)msg_id;
    (void)ctx;
    if (len != sizeof(prop_ftm_result_t) || data == NULL) {
        ESP_LOGW(TAG, "FTM result: unexpected len %u (want %u)",
                 (unsigned)len, (unsigned)sizeof(prop_ftm_result_t));
        return;
    }
    memcpy((void *)&s_ftm_result, data, sizeof(s_ftm_result));
    s_ftm_result_seq++;
}

/* One-shot: wait for the slave to report in, then push the persisted config to
 * it, spaced out so we don't flood the RPC TX path. Runs off the RX thread. */
static void push_task(void *arg)
{
    (void)arg;
    while (!s_slave_alive) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    prop_coproc_push_settings();
    vTaskDelete(NULL);
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

    err = esp_hosted_register_custom_callback(PROP_MSG_ID_FTM_RESULT, on_ftm_result, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register FTM-result callback failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Push persisted settings once the slave reports in (off the RX thread). */
    xTaskCreatePinnedToCore(push_task, "csi_push", 3072, NULL, 4, NULL, 0);
    return ESP_OK;
}

esp_err_t prop_coproc_ftm_request(const prop_ftm_req_t *req)
{
    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_hosted_send_custom_data(PROP_MSG_ID_FTM_REQ, (const uint8_t *)req, sizeof(*req));
}

bool prop_coproc_ftm_wait_result(uint16_t req_id, uint32_t timeout_ms, prop_ftm_result_t *out)
{
    /* Wait for the RX callback to actually bump the sequence counter — do NOT
     * trust the cache's initial/stale content on its own (its zero-valued boot
     * state has req_id=0, status=PROP_FTM_OK=0, which would otherwise look like
     * a false "success" for a caller's very first req_id-0 request). */
    uint32_t seen_seq = s_ftm_result_seq;
    uint32_t waited = 0;
    const uint32_t step_ms = 50;
    while (waited <= timeout_ms) {
        uint32_t cur_seq = s_ftm_result_seq;
        if (cur_seq != seen_seq) {
            prop_ftm_result_t snap;
            memcpy(&snap, (const void *)&s_ftm_result, sizeof(snap));
            if (snap.req_id == req_id) {
                if (out) {
                    *out = snap;
                }
                return true;
            }
            seen_seq = cur_seq;   /* mismatched/stale result; keep waiting */
        }
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        waited += step_ms;
    }
    return false;
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

esp_err_t prop_coproc_csi_set(const char *key, int32_t val)
{
    if (!key) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < CSI_NSETTINGS; i++) {
        if (strcmp(key, s_settings[i].key) != 0) {
            continue;
        }
        if (val < s_settings[i].lo || val > s_settings[i].hi) {
            ESP_LOGW(TAG, "csi_set %s=%ld out of range [%ld,%ld]",
                     key, (long)val, (long)s_settings[i].lo, (long)s_settings[i].hi);
            return ESP_ERR_INVALID_ARG;
        }
        char k[8];
        nvs_key_for_index(i, k);
        prop_settings_set_u32(k, (uint32_t)val);   /* persist (survives reflash) */
        esp_err_t err = send_cfg(key, val);        /* push live */
        ESP_LOGI(TAG, "csi_set %s=%ld (push %s)", key, (long)val, esp_err_to_name(err));
        return err;
    }
    ESP_LOGW(TAG, "csi_set: unknown key '%s'", key);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t prop_coproc_csi_action(const char *action)
{
    if (!action) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_cfg(action, 0);   /* transient: sent, not persisted */
}

esp_err_t prop_coproc_csi_push(const char *key, int32_t val)
{
    if (!key) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_cfg(key, val);    /* push only — no NVS write */
}

void prop_coproc_push_settings(void)
{
    for (int i = 0; i < CSI_NSETTINGS; i++) {
        send_cfg(s_settings[i].key, setting_value(i));
        vTaskDelay(pdMS_TO_TICKS(20));   /* space sends so we don't flood RPC TX */
    }
    ESP_LOGI(TAG, "pushed %d persisted CSI settings to C6", CSI_NSETTINGS);
}

int prop_coproc_csi_count(void)
{
    return CSI_NSETTINGS;
}

bool prop_coproc_csi_describe(int i, const char **key, int32_t *val,
                              char *type, int32_t *lo, int32_t *hi,
                              const char **desc, const char **opts)
{
    if (i < 0 || i >= CSI_NSETTINGS) {
        return false;
    }
    if (key)  { *key  = s_settings[i].key; }
    if (val)  { *val  = setting_value(i); }
    if (type) { *type = s_settings[i].type; }
    if (lo)   { *lo   = s_settings[i].lo; }
    if (hi)   { *hi   = s_settings[i].hi; }
    if (desc) { *desc = s_settings[i].desc; }
    if (opts) { *opts = s_settings[i].opts; }
    return true;
}
