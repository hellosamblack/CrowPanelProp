/* prop_net — AP+STA WiFi via the C6, with NVS-stored STA credentials. */
#include "prop_net.h"
#include "prop_engine.h"
#include "prop_settings.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "mdns.h"

#define NET_TAG "PROP_NET"
#define MAX_RETRY 8

/* Compile-time fallback STA creds, used only if none are stored in NVS yet. */
#define DEFAULT_STA_SSID    ""
#define DEFAULT_STA_PASSWD  ""

static esp_netif_t *s_netif_ap;
static esp_netif_t *s_netif_sta;
static int s_retry_num;
static char s_ip[16] = "0.0.0.0";
static volatile prop_sta_state_t s_sta_state = STA_IDLE;
static volatile int s_rssi;      /* cached signal strength (dBm), 0 = unknown */
static void rssi_task(void *arg);

static void load_sta_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    prop_settings_get_str("sta_ssid", ssid, ssid_len, DEFAULT_STA_SSID);
    prop_settings_get_str("sta_pass", pass, pass_len, DEFAULT_STA_PASSWD);
}

static esp_err_t apply_sta_config(const char *ssid, const char *pass)
{
    wifi_config_t sta = {
        .sta = {
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .failure_retry_cnt = MAX_RETRY,
            .threshold.authmode = WIFI_AUTH_OPEN,  /* accept open..WPA3 */
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    strlcpy((char *)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, pass, sizeof(sta.sta.password));
    return esp_wifi_set_config(WIFI_IF_STA, &sta);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        strlcpy(s_ip, "0.0.0.0", sizeof(s_ip));
        if (s_retry_num < MAX_RETRY) {
            s_retry_num++;
            esp_wifi_connect();
            ESP_LOGI(NET_TAG, "STA retry %d", s_retry_num);
        } else {
            s_sta_state = STA_FAILED;
            ESP_LOGW(NET_TAG, "STA connect failed; running AP-only");
        }
        prop_engine_set_link(LINK_AP);   /* hotspot is still up */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&event->ip_info.ip, s_ip, sizeof(s_ip));
        ESP_LOGI(NET_TAG, "STA got IP: %s", s_ip);
        s_retry_num = 0;
        s_sta_state = STA_CONNECTED;
        prop_engine_set_link(LINK_STA);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(NET_TAG, "client "MACSTR" joined hotspot", MAC2STR(e->mac));
    }
}

/* Bring up mDNS so the prop answers "<PROP_HOSTNAME>.local" and advertises its
 * web console over _http._tcp. Best-effort: failure here just means you fall back
 * to the raw IP, so we log and carry on rather than aborting WiFi. */
static void start_mdns(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(NET_TAG, "mdns_init failed: %s (no .local name)", esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(PROP_HOSTNAME);
    mdns_instance_name_set("Communicator Prop");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(NET_TAG, "mDNS up: http://%s.local/", PROP_HOSTNAME);
}

static esp_err_t wifi_init_softap(void)
{
    s_netif_ap = esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(s_netif_ap, PROP_HOSTNAME);
    wifi_config_t ap = {
        .ap = {
            .ssid = PROP_AP_SSID,
            .ssid_len = strlen(PROP_AP_SSID),
            .channel = 1,
            .password = PROP_AP_PASSWD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = false },
        },
    };
    if (strlen(PROP_AP_PASSWD) == 0) {
        ap.ap.authmode = WIFI_AUTH_OPEN;
    }
    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (err == ESP_OK) {
        ESP_LOGI(NET_TAG, "hotspot up: SSID=%s", PROP_AP_SSID);
    }
    return err;
}

esp_err_t prop_net_init(void)
{
    /* NVS is initialized earlier by prop_settings_init(). */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    /* From here on, calls cross the SDIO bus to the ESP32-C6 co-processor. If the
     * C6's esp_hosted slave firmware is missing or version-incompatible, these
     * fail — we log and return an error rather than ESP_ERROR_CHECK-aborting, so
     * the rest of the prop (display/LEDs/buttons/UI) keeps running without WiFi. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(NET_TAG, "esp_wifi_init failed: %s — ESP32-C6 co-processor not responding "
                 "or its hosted firmware is incompatible. Running without WiFi.",
                 esp_err_to_name(err));
        prop_engine_set_link(LINK_DOWN);
        return err;
    }

    if ((err = esp_wifi_set_mode(WIFI_MODE_APSTA)) != ESP_OK ||
        (err = wifi_init_softap()) != ESP_OK) {
        ESP_LOGE(NET_TAG, "WiFi AP setup failed: %s. Running without WiFi.", esp_err_to_name(err));
        prop_engine_set_link(LINK_DOWN);
        return err;
    }

    s_netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(s_netif_sta, PROP_HOSTNAME);   /* DHCP option 12 */
    char ssid[33] = {0}, pass[65] = {0};
    load_sta_credentials(ssid, sizeof(ssid), pass, sizeof(pass));
    apply_sta_config(ssid, pass);   /* STA config is best-effort */
    s_sta_state = ssid[0] ? STA_CONNECTING : STA_IDLE;

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(NET_TAG, "esp_wifi_start failed: %s. Running without WiFi.", esp_err_to_name(err));
        prop_engine_set_link(LINK_DOWN);
        return err;
    }
    prop_engine_set_link(LINK_AP);   /* hotspot live immediately; STA pending */
    start_mdns();                     /* advertise <PROP_HOSTNAME>.local */
    xTaskCreate(rssi_task, "rssi", 4096, NULL, 3, NULL);   /* background RSSI poll */
    ESP_LOGI(NET_TAG, "WiFi APSTA started (sta ssid='%s')", ssid);
    return ESP_OK;
}

int prop_net_scan(prop_ap_t *out, int max)
{
    if (!out || max <= 0) {
        return -1;
    }
    /* WiFi is already started in APSTA mode; trigger a blocking active scan. */
    wifi_scan_config_t scan_cfg = { 0 };   /* all channels, active scan */
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(NET_TAG, "scan start failed: %s", esp_err_to_name(err));
        return -1;
    }
    uint16_t num = 0;
    esp_wifi_scan_get_ap_num(&num);
    if (num == 0) {
        return 0;
    }
    wifi_ap_record_t *recs = calloc(num, sizeof(wifi_ap_record_t));
    if (!recs) {
        esp_wifi_clear_ap_list();
        return -1;
    }
    esp_wifi_scan_get_ap_records(&num, recs);   /* also frees the internal list */

    /* Records come back sorted by RSSI (strongest first); de-dup by SSID. */
    int count = 0;
    for (int i = 0; i < num && count < max; i++) {
        const char *ssid = (const char *)recs[i].ssid;
        if (ssid[0] == '\0') {
            continue;   /* hidden network */
        }
        bool dup = false;
        for (int j = 0; j < count; j++) {
            if (strcmp(out[j].ssid, ssid) == 0) { dup = true; break; }
        }
        if (dup) {
            continue;
        }
        strlcpy(out[count].ssid, ssid, sizeof(out[count].ssid));
        out[count].rssi = recs[i].rssi;
        out[count].secured = (recs[i].authmode != WIFI_AUTH_OPEN &&
                              recs[i].authmode != WIFI_AUTH_OWE);
        count++;
    }
    free(recs);
    ESP_LOGI(NET_TAG, "scan found %d AP(s) (%u raw)", count, num);
    return count;
}

esp_err_t prop_net_set_sta_credentials(const char *ssid, const char *password, bool remember)
{
    if (!ssid) {
        return ESP_ERR_INVALID_ARG;
    }
    if (remember) {
        esp_err_t err = prop_settings_set_str("sta_ssid", ssid);
        if (err == ESP_OK) {
            err = prop_settings_set_str("sta_pass", password ? password : "");
        }
        if (err != ESP_OK) {
            return err;
        }
    }

    /* Reconnect with the new credentials (persisted or session-only). */
    s_retry_num = 0;
    s_sta_state = STA_CONNECTING;
    esp_wifi_disconnect();
    apply_sta_config(ssid, password ? password : "");
    esp_wifi_connect();
    ESP_LOGI(NET_TAG, "STA credentials set -> '%s' (%s), reconnecting",
             ssid, remember ? "saved" : "session-only");
    return ESP_OK;
}

esp_err_t prop_net_forget(void)
{
    prop_settings_set_str("sta_ssid", "");
    prop_settings_set_str("sta_pass", "");
    s_retry_num = MAX_RETRY;        /* don't auto-retry the cleared network */
    s_sta_state = STA_IDLE;
    esp_wifi_disconnect();
    strlcpy(s_ip, "0.0.0.0", sizeof(s_ip));
    prop_engine_set_link(LINK_AP);
    ESP_LOGI(NET_TAG, "STA network forgotten");
    return ESP_OK;
}

prop_sta_state_t prop_net_sta_state(void)
{
    return s_sta_state;
}

void prop_net_get_ip(char *out, size_t out_len)
{
    if (out && out_len) {
        strlcpy(out, s_ip, out_len);
    }
}

int prop_net_get_rssi(void)
{
    return s_rssi;   /* cached by rssi_task; cheap, no SDIO round-trip here */
}

/* Poll RSSI in the background (~1 Hz) so UI code can read it without crossing the
 * SDIO bus on the render path — doing that under the LVGL lock stalls the panel. */
static void rssi_task(void *arg)
{
    (void)arg;
    for (;;) {
        int rssi = 0;
        if (s_sta_state == STA_CONNECTED && esp_wifi_sta_get_rssi(&rssi) == ESP_OK) {
            s_rssi = rssi;
        } else {
            s_rssi = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
