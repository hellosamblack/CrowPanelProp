/* prop_ftm — WiFi FTM ranging table + backoff engine. See prop_ftm.h. */
#include "prop_ftm.h"
#include "prop_net.h"
#include "prop_coproc.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"   /* MACSTR/MAC2STR */

#define FTM_TAG "PROP_FTM"

/* Re-scan + probe cycle period. A probe can block up to FTM_WAIT_TIMEOUT_MS,
 * so a cycle can run well past this when several APs are due at once — that's
 * fine, this is a low-priority background task. */
#define FTM_CYCLE_MS            8000
#define FTM_WAIT_TIMEOUT_MS     11000  /* > the C6 worker's ~10s internal timeout */
#define FTM_MAX_PROBES_PER_CYCLE 8     /* safety bound; most envs have 0-2 capable APs */
#define FTM_SUCCESS_REPROBE_MS  20000  /* a responsive AP is re-checked fairly often */
#define FTM_BACKOFF_BASE_MS     15000  /* first retry after a failure */
#define FTM_BACKOFF_CAP_MS      (30 * 60 * 1000)   /* never wait longer than this */
#define FTM_STALE_MS            (5 * 60 * 1000)    /* evict a BSSID unseen this long */
#define FTM_FAILS_CLAMP         10     /* consecutive_fails clamp (2^10 * base > cap already) */

static prop_ftm_entry_t s_table[PROP_FTM_TABLE_MAX];
static int s_count;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_available;
static uint16_t s_next_req_id = 1;   /* 0 reserved/avoided for log clarity */

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static uint32_t backoff_for(uint8_t consecutive_fails)
{
    uint32_t ms = FTM_BACKOFF_BASE_MS;
    for (uint8_t i = 0; i < consecutive_fails && ms < FTM_BACKOFF_CAP_MS; i++) {
        ms *= 2;
    }
    return ms > FTM_BACKOFF_CAP_MS ? FTM_BACKOFF_CAP_MS : ms;
}

/* Merge one freshly-scanned AP into the table: update an existing entry, or
 * insert (evicting the stalest entry if full). Must be called under s_mux. */
static void merge_locked(const prop_ap_t *ap, uint32_t now)
{
    int idx = -1;
    for (int i = 0; i < s_count; i++) {
        if (memcmp(s_table[i].bssid, ap->bssid, 6) == 0) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        if (s_count < PROP_FTM_TABLE_MAX) {
            idx = s_count++;
            memset(&s_table[idx], 0, sizeof(s_table[idx]));
            s_table[idx].last_dist_cm = -1;
            memcpy(s_table[idx].bssid, ap->bssid, 6);
        } else {
            /* Table full: evict the entry unseen the longest. */
            int oldest = 0;
            for (int i = 1; i < s_count; i++) {
                if (s_table[i].last_seen_ms < s_table[oldest].last_seen_ms) {
                    oldest = i;
                }
            }
            idx = oldest;
            memset(&s_table[idx], 0, sizeof(s_table[idx]));
            s_table[idx].last_dist_cm = -1;
            memcpy(s_table[idx].bssid, ap->bssid, 6);
        }
    }
    strlcpy(s_table[idx].ssid, ap->ssid, sizeof(s_table[idx].ssid));
    s_table[idx].ftm_capable = ap->ftm;
    s_table[idx].rssi = ap->rssi;
    s_table[idx].channel = ap->channel;
    s_table[idx].last_seen_ms = now;
}

/* Age out entries not seen in a recent scan. Must be called under s_mux. */
static void prune_stale_locked(uint32_t now)
{
    for (int i = 0; i < s_count; ) {
        if (now - s_table[i].last_seen_ms > FTM_STALE_MS) {
            s_table[i] = s_table[--s_count];   /* swap-remove */
        } else {
            i++;
        }
    }
}

/* One FTM probe: send the request, wait for the result, update the entry's
 * status/backoff in place. Blocking (up to FTM_WAIT_TIMEOUT_MS). */
static void probe_one(prop_ftm_entry_t *snap)
{
    uint16_t req_id = s_next_req_id++;
    if (s_next_req_id == 0) {
        s_next_req_id = 1;
    }

    prop_ftm_req_t req = {
        .channel = snap->channel,
        .frm_count = 0,       /* driver default */
        .burst_period = 0,    /* single-shot, ASAP mode */
        .req_id = req_id,
    };
    memcpy(req.bssid, snap->bssid, 6);

    esp_err_t err = prop_coproc_ftm_request(&req);
    prop_ftm_result_t res = { .status = PROP_FTM_TIMEOUT };
    bool got = (err == ESP_OK) && prop_coproc_ftm_wait_result(req_id, FTM_WAIT_TIMEOUT_MS, &res);
    if (!got) {
        res.status = PROP_FTM_TIMEOUT;   /* covers both send failure and no reply */
    }

    prop_ftm_entry_status_t new_status =
        (res.status == PROP_FTM_OK)      ? PROP_FTM_ENTRY_OK :
        (res.status == PROP_FTM_TIMEOUT) ? PROP_FTM_ENTRY_TIMEOUT :
        (res.status == PROP_FTM_BUSY)    ? PROP_FTM_ENTRY_BUSY :
                                            PROP_FTM_ENTRY_NO_REPLY;
    uint32_t now = now_ms();

    portENTER_CRITICAL(&s_mux);
    /* Re-find the entry by BSSID (the table may have shifted under a
     * swap-remove eviction while we were blocked on the probe). */
    int idx = -1;
    for (int i = 0; i < s_count; i++) {
        if (memcmp(s_table[i].bssid, snap->bssid, 6) == 0) { idx = i; break; }
    }
    if (idx >= 0) {
        prop_ftm_entry_t *e = &s_table[idx];
        e->status = new_status;
        if (new_status == PROP_FTM_ENTRY_OK) {
            e->last_dist_cm = res.dist_est_cm;
            e->consecutive_fails = 0;
            e->next_probe_ms = now + FTM_SUCCESS_REPROBE_MS;
        } else {
            if (e->consecutive_fails < FTM_FAILS_CLAMP) {
                e->consecutive_fails++;
            }
            e->next_probe_ms = now + backoff_for(e->consecutive_fails);
        }
    }
    portEXIT_CRITICAL(&s_mux);

    if (new_status == PROP_FTM_ENTRY_OK) {
        ESP_LOGI(FTM_TAG, MACSTR" -> %ld cm", MAC2STR(snap->bssid), (long)res.dist_est_cm);
    } else {
        ESP_LOGD(FTM_TAG, MACSTR" probe status=%d", MAC2STR(snap->bssid), (int)new_status);
    }
}

static void ftm_task(void *arg)
{
    (void)arg;
    for (;;) {
        prop_ap_t scan[PROP_NET_SCAN_MAX];
        int n = prop_net_scan_raw(scan, PROP_NET_SCAN_MAX);
        uint32_t now = now_ms();

        if (n > 0) {
            portENTER_CRITICAL(&s_mux);
            for (int i = 0; i < n; i++) {
                merge_locked(&scan[i], now);
            }
            prune_stale_locked(now);
            portEXIT_CRITICAL(&s_mux);
        }

        /* Snapshot due (capable, past backoff) entries, then probe outside the
         * critical section — a probe blocks for seconds. */
        prop_ftm_entry_t due[FTM_MAX_PROBES_PER_CYCLE];
        int due_n = 0;
        portENTER_CRITICAL(&s_mux);
        for (int i = 0; i < s_count && due_n < FTM_MAX_PROBES_PER_CYCLE; i++) {
            if (s_table[i].ftm_capable && now >= s_table[i].next_probe_ms) {
                due[due_n++] = s_table[i];
            }
        }
        portEXIT_CRITICAL(&s_mux);

        for (int i = 0; i < due_n; i++) {
            probe_one(&due[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(FTM_CYCLE_MS));
    }
}

esp_err_t prop_ftm_init(void)
{
    memset(s_table, 0, sizeof(s_table));
    s_count = 0;
    if (xTaskCreatePinnedToCore(ftm_task, "prop_ftm", 4096, NULL, 3, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_available = true;
    ESP_LOGI(FTM_TAG, "RANGE: FTM ranging task started");
    return ESP_OK;
}

bool prop_ftm_available(void) { return s_available; }

int prop_ftm_get_table(prop_ftm_entry_t *out, int max)
{
    if (!out || max <= 0) {
        return 0;
    }
    portENTER_CRITICAL(&s_mux);
    int n = s_count < max ? s_count : max;
    memcpy(out, s_table, n * sizeof(prop_ftm_entry_t));
    portEXIT_CRITICAL(&s_mux);

    /* Sort the copy: FTM-capable first, then strongest RSSI (small N). */
    for (int i = 1; i < n; i++) {
        prop_ftm_entry_t key = out[i];
        int j = i - 1;
        while (j >= 0 && (out[j].ftm_capable < key.ftm_capable ||
                          (out[j].ftm_capable == key.ftm_capable && out[j].rssi < key.rssi))) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = key;
    }
    return n;
}

void prop_ftm_get_summary(int *tracked, int *capable, int *ranged_ok)
{
    int t = 0, c = 0, r = 0;
    portENTER_CRITICAL(&s_mux);
    t = s_count;
    for (int i = 0; i < s_count; i++) {
        if (s_table[i].ftm_capable) {
            c++;
            if (s_table[i].last_dist_cm >= 0) {
                r++;
            }
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (tracked)   *tracked = t;
    if (capable)   *capable = c;
    if (ranged_ok) *ranged_ok = r;
}
