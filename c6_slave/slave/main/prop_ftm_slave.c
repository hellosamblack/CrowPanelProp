/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CrowPanel prop — on-C6 WiFi FTM (802.11mc) ranging. Runs the real esp_wifi
 * FTM initiator locally (esp-hosted's stock RPC never wires FTM through —
 * see prop_ftm_slave.h) and ships only the aggregate per-session result to
 * the P4 host over esp-hosted custom RPC. Only one FTM session runs at a
 * time on the radio; requests are queued and served one at a time.
 */
#include "sdkconfig.h"

#ifdef CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_hosted_peer_data.h"   /* esp_hosted_send_custom_data() */
#include "prop_ftm_slave.h"

static const char *TAG = "prop_ftm_slave";

#ifndef MACSTR
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif

/* Single-shot ASAP-mode FTM (burst_period 0) can take a few seconds to
 * complete; the stock IDF ftm example's own default wait is 10s. Backstop
 * only — the WIFI_EVENT_FTM_REPORT handler normally signals well before this. */
#define FTM_SESSION_TIMEOUT_MS  10000

static QueueHandle_t     s_req_queue;      /* depth 1: at most one outstanding request */
static SemaphoreHandle_t s_report_sem;
static volatile bool     s_busy;
static wifi_event_ftm_report_t s_last_report;

/* Host -> slave: one FTM request. RPC RX thread — keep it cheap (per
 * esp_hosted_peer_data.h's callback contract): validate, and either queue
 * the work or bounce a BUSY result straight back if a session is already
 * running, rather than blocking this thread on the radio. */
static void on_ftm_req(uint32_t msg_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)msg_id;
    (void)ctx;
    if (len != sizeof(prop_ftm_req_t) || data == NULL) {
        ESP_LOGW(TAG, "req: bad len %u", (unsigned)len);
        return;
    }
    prop_ftm_req_t req;
    memcpy(&req, data, sizeof(req));

    if (s_busy) {
        prop_ftm_result_t res = { .req_id = req.req_id, .status = PROP_FTM_BUSY };
        memcpy(res.bssid, req.bssid, 6);
        esp_hosted_send_custom_data(PROP_MSG_ID_FTM_RESULT, (const uint8_t *)&res, sizeof(res));
        return;
    }
    s_busy = true;
    if (xQueueSend(s_req_queue, &req, 0) != pdTRUE) {
        s_busy = false;   /* queue unexpectedly full; drop and let the host retry */
    }
}

/* WIFI_EVENT_FTM_REPORT — registered once at init, not per-request. Stash the
 * report and wake the worker task. */
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base != WIFI_EVENT || id != WIFI_EVENT_FTM_REPORT) {
        return;
    }
    memcpy(&s_last_report, data, sizeof(s_last_report));
    xSemaphoreGive(s_report_sem);
}

static void ftm_worker_task(void *arg)
{
    (void)arg;
    for (;;) {
        prop_ftm_req_t req;
        if (xQueueReceive(s_req_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        wifi_ftm_initiator_cfg_t cfg = {
            .channel     = req.channel,
            .frm_count   = req.frm_count,
            .burst_period = req.burst_period,
        };
        memcpy(cfg.resp_mac, req.bssid, 6);

        prop_ftm_result_t res = { .req_id = req.req_id };
        memcpy(res.bssid, req.bssid, 6);

        xSemaphoreTake(s_report_sem, 0);   /* drain any stale give */
        esp_err_t err = esp_wifi_ftm_initiate_session(&cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "initiate_session "MACSTR" ch%u failed: %s",
                     MAC2STR(req.bssid), req.channel, esp_err_to_name(err));
            res.status = PROP_FTM_FAIL;
        } else if (xSemaphoreTake(s_report_sem, pdMS_TO_TICKS(FTM_SESSION_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGW(TAG, "session "MACSTR" timed out", MAC2STR(req.bssid));
            esp_wifi_ftm_end_session();
            res.status = PROP_FTM_TIMEOUT;
        } else if (s_last_report.status != FTM_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "session "MACSTR" failed: status=%d",
                     MAC2STR(req.bssid), (int)s_last_report.status);
            res.status = PROP_FTM_FAIL;
            res.fail_reason = (uint8_t)s_last_report.status;
        } else {
            res.status       = PROP_FTM_OK;
            res.dist_est_cm  = (int32_t)s_last_report.dist_est;
            res.rtt_est_ns   = (int32_t)s_last_report.rtt_est;
            ESP_LOGI(TAG, "session "MACSTR" ok: dist=%ld cm rtt=%ld ns",
                     MAC2STR(req.bssid), (long)res.dist_est_cm, (long)res.rtt_est_ns);
        }
        /* Free the internal FTM report we didn't otherwise consume (attention
         * note on esp_wifi_ftm_get_report: passing NULL merely frees it). Only
         * meaningful after a successful exchange; harmless no-op otherwise. */
        esp_wifi_ftm_get_report(NULL, 0);

        esp_hosted_send_custom_data(PROP_MSG_ID_FTM_RESULT, (const uint8_t *)&res, sizeof(res));
        s_busy = false;
    }
}

esp_err_t prop_ftm_slave_init(void)
{
    s_req_queue = xQueueCreate(1, sizeof(prop_ftm_req_t));
    s_report_sem = xSemaphoreCreateBinary();
    if (!s_req_queue || !s_report_sem) {
        ESP_LOGE(TAG, "queue/sem alloc failed");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_FTM_REPORT,
                                                         &on_wifi_event, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "register FTM_REPORT handler failed: %s", esp_err_to_name(err));
    }

    err = esp_hosted_register_custom_callback(PROP_MSG_ID_FTM_REQ, on_ftm_req, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "register FTM-req callback failed: %s", esp_err_to_name(err));
    }

    BaseType_t ok = xTaskCreate(ftm_worker_task, "ftm_slave", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "on-C6 FTM initiator ready");
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

#endif /* CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER */
