/* prop_traffic — host-side CSI traffic generator (pings the gateway). See header. */
#include "prop_traffic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "ping/ping_sock.h"
#include "lwip/ip_addr.h"

#define TR_TAG "PROP_TRAFFIC"

static volatile int s_want_rate;   /* desired packets/sec (0 = off) */
static volatile int s_mode;        /* 0 ping, 1 dns (v1: ping) */
static esp_ping_handle_t s_ping;   /* current session, or NULL */
static int s_cur_rate;             /* rate the current session was built for */
static esp_ip4_addr_t s_cur_gw;    /* gateway the current session targets */

/* Current STA gateway, or 0.0.0.0 if not connected. */
static esp_ip4_addr_t sta_gateway(void)
{
    esp_ip4_addr_t none = { 0 };
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK) {
        return ip.gw;
    }
    return none;
}

static void stop_session(void)
{
    if (s_ping) {
        esp_ping_stop(s_ping);
        esp_ping_delete_session(s_ping);
        s_ping = NULL;
    }
    s_cur_rate = 0;
}

static void start_session(int rate, esp_ip4_addr_t gw)
{
    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    ip_addr_t target = { 0 };
    target.type = IPADDR_TYPE_V4;
    target.u_addr.ip4.addr = gw.addr;
    cfg.target_addr = target;
    cfg.count = ESP_PING_COUNT_INFINITE;
    cfg.interval_ms = (uint32_t)(rate > 0 ? (1000 / rate) : 1000);
    if (cfg.interval_ms < 1) { cfg.interval_ms = 1; }
    cfg.task_stack_size = 2560;
    cfg.task_prio = 2;                /* low: it's filler traffic */

    esp_ping_callbacks_t cbs = { 0 };  /* we don't care about results, only RX */
    if (esp_ping_new_session(&cfg, &cbs, &s_ping) != ESP_OK) {
        ESP_LOGW(TR_TAG, "ping session create failed");
        s_ping = NULL;
        return;
    }
    esp_ping_start(s_ping);
    s_cur_rate = rate;
    s_cur_gw = gw;
    ESP_LOGI(TR_TAG, "generating %d pkt/s to gw " IPSTR, rate, IP2STR(&gw));
}

static void traffic_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        int want = s_want_rate;
        esp_ip4_addr_t gw = sta_gateway();
        bool have_gw = gw.addr != 0;

        /* Tear down if we shouldn't be running or the target/rate changed. */
        if (s_ping && (want <= 0 || !have_gw ||
                       want != s_cur_rate || gw.addr != s_cur_gw.addr)) {
            stop_session();
        }
        /* (Re)start when we should be running and aren't. */
        if (!s_ping && want > 0 && have_gw) {
            start_session(want, gw);
        }
    }
}

esp_err_t prop_traffic_init(void)
{
    if (xTaskCreatePinnedToCore(traffic_task, "prop_traffic", 3072, NULL, 3, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TR_TAG, "CSI traffic generator ready");
    return ESP_OK;
}

void prop_traffic_set_rate(int pps)
{
    s_want_rate = (pps < 0) ? 0 : pps;
}

void prop_traffic_set_mode(int mode)
{
    s_mode = mode;
    if (mode == 1) {
        ESP_LOGW(TR_TAG, "dns mode not implemented in v1 — using ping");
    }
}
