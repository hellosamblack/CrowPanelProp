/* prop_ble — passive BLE scan via the C6's BLE controller + NimBLE host. See prop_ble.h. */
#include "prop_ble.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

/* NimBLE host (controller is remote on the C6, reached over esp_hosted VHCI). */
#include "esp_hosted.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static void start_scan(void);

#define BLE_TAG "PROP_BLE"

static bool s_available;
static uint8_t s_own_addr_type;

/* Bounded contact table, shared between the NimBLE host task (writer) and the UI
 * (reader). Guarded by a short spinlock — the critical sections only touch the
 * fixed array, never NimBLE or the radio. */
static prop_ble_dev_t s_devs[PROP_BLE_MAX];
static int s_count;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---- Company-ID -> prop label -------------------------------------------
 * Author-editable: a short list of common BT SIG Company IDs mapped to brand
 * labels. It's flavour (so the panel can read "CIVILIAN UNIT // APPLE"), not a
 * real OUI database — add entries as you like. */
static const struct { uint16_t id; const char *label; } s_companies[] = {
    { 0x004C, "APPLE"     },
    { 0x0006, "MICROSOFT" },
    { 0x0075, "SAMSUNG"   },
    { 0x00E0, "GOOGLE"    },
    { 0x0087, "GARMIN"    },
    { 0x0002, "INTEL"     },
    { 0x000F, "BROADCOM"  },
    { 0x0059, "NORDIC"    },
    { 0x0157, "HUAMI"     },
    { 0x038F, "XIAOMI"    },
    { 0x00D7, "QUALCOMM"  },
    { 0x009E, "BOSE"      },
    { 0x02E5, "ESPRESSIF" },
};

const char *prop_ble_company_label(uint16_t company_id)
{
    if (company_id == PROP_BLE_NONE) {
        return NULL;
    }
    for (size_t i = 0; i < sizeof(s_companies) / sizeof(s_companies[0]); i++) {
        if (s_companies[i].id == company_id) {
            return s_companies[i].label;
        }
    }
    return NULL;
}

/* BT SIG appearance -> device class, keyed by the category (top 10 bits). The low
 * 6 bits are the sub-type; the category alone is enough for a prop readout. */
const char *prop_ble_appearance_label(uint16_t appearance)
{
    if (appearance == 0) {
        return NULL;
    }
    switch (appearance >> 6) {
        case 1:  return "PHONE";
        case 2:  return "COMPUTER";
        case 3:  return "WATCH";
        case 4:  return "CLOCK";
        case 5:  return "DISPLAY";
        case 6:  return "REMOTE";
        case 7:  return "GLASSES";
        case 8:  return "TAG";
        case 9:  return "KEYRING";
        case 10: return "MEDIA PLAYER";
        case 13: return "HR SENSOR";
        case 14: return "BP MONITOR";
        case 22: return "SENSOR";
        case 37: return "EARBUDS";
        default: return "DEVICE";
    }
}

int8_t prop_ble_calib_txpower(int8_t tx_power)
{
    /* Many devices either omit TX power or advertise a wildly optimistic value
     * (which inflates the distance estimate to hundreds of metres). Trust it only
     * inside a sane window; otherwise fall back to a typical handset level. */
    if (tx_power == PROP_BLE_TXPWR_NONE) return PROP_BLE_TXPWR_TYP;
    if (tx_power < -40 || tx_power > 12)  return PROP_BLE_TXPWR_TYP;
    return tx_power;
}

float prop_ble_distance_m(int8_t rssi, int8_t tx_power)
{
    /* Log-distance path loss: d = 10^((P1m - rssi) / (10 * n)).
     *   P1m  reference RSSI at 1 m, derived from the (calibrated) TX power.
     *   n    path-loss exponent (~2.5 indoors). */
    float p1m = (float)prop_ble_calib_txpower(tx_power) - 7.0f;
    const float n = 2.5f;
    float d = powf(10.0f, (p1m - (float)rssi) / (10.0f * n));
    if (d < 0.1f)                  d = 0.1f;
    if (d > PROP_BLE_RANGE_MAX_M)  d = PROP_BLE_RANGE_MAX_M;   /* scanner's real range */
    return d;
}

/* ---- Contact table ------------------------------------------------------- */

/* Insert or refresh a contact. Called only from the NimBLE host task. */
static void upsert_device(const uint8_t mac[6], int8_t rssi,
                          const char *name, int name_len, uint16_t company_id,
                          int8_t tx_power, uint16_t appearance,
                          const uint8_t *uuid, uint8_t uuid_len)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);

    portENTER_CRITICAL(&s_mux);

    int slot = -1;
    for (int i = 0; i < s_count; i++) {
        if (memcmp(s_devs[i].mac, mac, 6) == 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        if (s_count < PROP_BLE_MAX) {
            slot = s_count++;
            memset(&s_devs[slot], 0, sizeof(s_devs[slot]));
        } else {
            /* Table full: evict the least-recently-seen contact. */
            uint32_t oldest = now;
            slot = 0;
            for (int i = 0; i < s_count; i++) {
                if (s_devs[i].last_seen <= oldest) {
                    oldest = s_devs[i].last_seen;
                    slot = i;
                }
            }
            memset(&s_devs[slot], 0, sizeof(s_devs[slot]));
        }
        memcpy(s_devs[slot].mac, mac, 6);
        s_devs[slot].company_id = PROP_BLE_NONE;
        s_devs[slot].tx_power = PROP_BLE_TXPWR_NONE;
    }

    s_devs[slot].rssi = rssi;
    s_devs[slot].last_seen = now;
    if (company_id != PROP_BLE_NONE) {
        s_devs[slot].company_id = company_id;
    }
    if (tx_power != PROP_BLE_TXPWR_NONE) {
        s_devs[slot].tx_power = tx_power;
    }
    if (appearance != 0) {
        s_devs[slot].appearance = appearance;
    }
    if (uuid && uuid_len) {
        memcpy(s_devs[slot].uuid, uuid, uuid_len);
        s_devs[slot].uuid_len = uuid_len;
    }
    if (name && name_len > 0) {
        int n = name_len < (int)sizeof(s_devs[slot].name) - 1
                ? name_len : (int)sizeof(s_devs[slot].name) - 1;
        memcpy(s_devs[slot].name, name, n);
        s_devs[slot].name[n] = '\0';
    }

    portEXIT_CRITICAL(&s_mux);
}

/* Age out contacts not seen for a while, so the count tracks who's actually here. */
#define BLE_STALE_MS 20000
static void prune_stale(void)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < s_count; ) {
        if (now - s_devs[i].last_seen > BLE_STALE_MS) {
            s_devs[i] = s_devs[--s_count];   /* swap-remove */
        } else {
            i++;
        }
    }
    portEXIT_CRITICAL(&s_mux);
}

int prop_ble_get_devices(prop_ble_dev_t *out, int max)
{
    if (!out || max <= 0) {
        return 0;
    }
    portENTER_CRITICAL(&s_mux);
    int n = s_count < max ? s_count : max;
    memcpy(out, s_devs, n * sizeof(prop_ble_dev_t));
    portEXIT_CRITICAL(&s_mux);

    /* Sort the copy strongest-first (small N, simple insertion sort). */
    for (int i = 1; i < n; i++) {
        prop_ble_dev_t key = out[i];
        int j = i - 1;
        while (j >= 0 && out[j].rssi < key.rssi) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = key;
    }
    return n;
}

void prop_ble_get_summary(int *count, int8_t *strongest, int *named, int *known)
{
    int c = 0, nm = 0, kn = 0;
    int8_t best = 0;
    portENTER_CRITICAL(&s_mux);
    c = s_count;
    for (int i = 0; i < s_count; i++) {
        if (i == 0 || s_devs[i].rssi > best) best = s_devs[i].rssi;
        if (s_devs[i].name[0]) nm++;
        if (s_devs[i].company_id != PROP_BLE_NONE &&
            prop_ble_company_label(s_devs[i].company_id)) kn++;
    }
    portEXIT_CRITICAL(&s_mux);
    if (count)     *count = c;
    if (strongest) *strongest = best;
    if (named)     *named = nm;
    if (known)     *known = kn;
}

bool prop_ble_available(void) { return s_available; }

/* ---- NimBLE GAP scan ----------------------------------------------------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) != 0) {
            return 0;   /* malformed advert — ignore */
        }
        uint16_t company = PROP_BLE_NONE;
        if (fields.mfg_data && fields.mfg_data_len >= 2) {
            company = (uint16_t)fields.mfg_data[0] |
                      ((uint16_t)fields.mfg_data[1] << 8);   /* little-endian */
        }
        int8_t tx_power = fields.tx_pwr_lvl_is_present ? fields.tx_pwr_lvl : PROP_BLE_TXPWR_NONE;
        uint16_t appearance = fields.appearance_is_present ? fields.appearance : 0;

        /* Capture the most-identifying advertised service UUID (LE bytes): prefer a
         * full 128-bit UUID, then 32-bit, then 16-bit. */
        const uint8_t *uuid = NULL; uint8_t uuid_len = 0;
        if (fields.num_uuids128 > 0) {
            uuid = fields.uuids128[0].value;               uuid_len = 16;
        } else if (fields.num_uuids32 > 0) {
            uuid = (const uint8_t *)&fields.uuids32[0].value; uuid_len = 4;
        } else if (fields.num_uuids16 > 0) {
            uuid = (const uint8_t *)&fields.uuids16[0].value; uuid_len = 2;
        }

        upsert_device(event->disc.addr.val, event->disc.rssi,
                      (const char *)fields.name, fields.name_len, company,
                      tx_power, appearance, uuid, uuid_len);
    } else if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        /* Scan windows shouldn't end (we ask for forever), but restart if one does.
         * start_scan latches s_scan_restart_pending on failure; prune_task retries. */
        ESP_LOGW(BLE_TAG, "scan ended (reason %d) — restarting", event->disc_complete.reason);
        start_scan();
    }
    return 0;
}

/* Set when ble_gap_disc failed and the passive scan is down; prune_task keeps
 * retrying start_scan until it sticks. */
static volatile bool s_scan_restart_pending;

static void start_scan(void)
{
    /* Passive (listen-only) continuous scan with duplicates allowed so RSSI keeps
     * refreshing. Passive keeps radio impact low next to WiFi on the shared C6. */
    struct ble_gap_disc_params dp = {
        .passive = 1,
        .filter_duplicates = 0,
        .limited = 0,
    };
    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &dp, gap_event_cb, NULL);
    if (rc == 0 || rc == BLE_HS_EALREADY) {
        s_scan_restart_pending = false;
        ESP_LOGI(BLE_TAG, "passive scan started");
    } else {
        s_scan_restart_pending = true;
        ESP_LOGE(BLE_TAG, "ble_gap_disc start failed: %d — will retry", rc);
    }
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "ensure_addr failed: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "infer addr type failed: %d", rc);
        return;
    }
    start_scan();
}

static void on_reset(int reason)
{
    ESP_LOGW(BLE_TAG, "nimble host reset, reason=%d", reason);
}

static void host_task(void *param)
{
    (void)param;
    ESP_LOGI(BLE_TAG, "NimBLE host task started");
    nimble_port_run();           /* returns only on nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* Background janitor: age out stale contacts (~2 Hz). The host task is event
 * driven, so a tiny separate task keeps the count honest without it. */
static void prune_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        prune_stale();
        if (s_scan_restart_pending) {
            start_scan();   /* passive scan went down; keep retrying until it sticks */
        }
    }
}

esp_err_t prop_ble_init(void)
{
    /* The C6 hosts the BLE controller; the SDIO transport is already up from
     * prop_net_init (esp_wifi_init). Just bring up the controller + NimBLE host. */
    esp_err_t err = esp_hosted_bt_controller_init();
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "bt controller init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_hosted_bt_controller_enable();
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "bt controller enable failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb  = on_sync;     /* fires once the host/controller sync — starts the scan */

    nimble_port_freertos_init(host_task);
    xTaskCreatePinnedToCore(prune_task, "ble_prune", 2560, NULL, 3, NULL, 0);

    s_available = true;
    ESP_LOGI(BLE_TAG, "BLE up (NimBLE host, controller on C6)");
    return ESP_OK;
}
