/* prop_battery — STC8H1K08 battery telemetry over the shared I2C1 bus.
 *
 * See prop_battery.h and the stc8h1k08-battery-i2c-protocol memory for how this
 * was reverse-engineered. Short version: Elecrow's factory source (a different
 * eval board, same chip part number) claims register 0x00 returns a rich
 * 11-byte struct (mV voltage + SOC% + charge-state, chip-computed). Empirically,
 * on THIS board that's wrong — only byte 0 is ever populated; the rest reads
 * back as 0xFF filler. This board's STC8 firmware exposes nothing but a raw
 * 8-bit ADC count of the battery-voltage divider tap. Everything else (real
 * voltage, SOC%, charge/discharge state, time-to-empty/full) is derived here:
 *
 *   - voltage: raw count -> ADC pin mV (assumes 3.3V STC8 ADC reference) ->
 *     battery mV via the board's own R164/R186 divider ratio.
 *   - SOC%: a generic single-cell LiPo open-circuit-voltage curve (not
 *     temperature/age/load compensated — good enough for a prop readout, not
 *     a lab instrument).
 *   - charge state: since there's no CHG-pin readout either, inferred from the
 *     trend (dV/dt) of the derived voltage over several minutes. The 8-bit ADC
 *     is coarse (~18 mV/count after divider back-conversion), so short-window
 *     trend detection is unreliable — this deliberately waits for a long
 *     window before trusting a slope, and simply doesn't report a time
 *     estimate rather than guess from noise.
 */
#include "prop_battery.h"
#include "bsp_i2c.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "BATTERY"

#define STC8_I2C_ADDR       0x2F
#define STC8_REG_BATTERY    0x00

#define POLL_PERIOD_MS       8000

/* raw ADC count -> mV, per the empirical calibration in the memory doc.
 * STC8_ADC_VREF_MV is an assumption (not independently confirmed with a
 * multimeter) but self-consistent: no-battery reads back-computed to an
 * implausible ~0.6V (floating pin), battery-plugged reads back-computed to
 * ~4.0V, exactly where a fresh 3.7V-nominal LiPo should sit. */
#define STC8_ADC_VREF_MV     3300
#define DIV_R_BATTERY_OHM    39000   /* R164, battery side */
#define DIV_R_GROUND_OHM     100000  /* R186, ground side */

/* Below this, treat the reading as "nothing plugged into J3" rather than a
 * real (if catastrophically depleted) cell — no-battery reads sit far below
 * any real single-cell LiPo voltage (~0.6V vs a real cell's 3.0-4.2V). */
#define NO_BATTERY_THRESHOLD_MV  2500

/* Rolling voltage-rate history for charge/discharge trend detection. Coarse
 * 8-bit ADC (~18 mV/count) means a short window is mostly quantization noise,
 * so this deliberately waits a long time before trusting a slope. At
 * POLL_PERIOD_MS=8s, 128 entries spans ~17 minutes. */
#define RATE_HISTORY_LEN     128
#define RATE_MIN_SPAN_S      240    /* need at least 4 min of history before trusting the trend */
#define RATE_NOISE_FLOOR_MV_S (0.5f / 60.0f)  /* mV/s; below this, call the trend "flat" */

#define FULLY_CHARGED_MV     4150   /* flat + at/above this -> report FULLY_CHARGED, not IDLE */

static i2c_master_dev_handle_t s_dev;
static bool                    s_online;
static SemaphoreHandle_t       s_mutex;
static prop_battery_data_t     s_data;

typedef struct {
    int64_t us;
    uint32_t mv;
} rate_sample_t;

static rate_sample_t s_hist[RATE_HISTORY_LEN];
static int           s_hist_count;
static int           s_hist_head;   /* next write index */
static bool           s_last_present = false;  /* forces a reset on first sample */

static void hist_reset(void)
{
    s_hist_count = 0;
    s_hist_head = 0;
}

static void hist_push(int64_t us, uint32_t mv)
{
    s_hist[s_hist_head] = (rate_sample_t){ .us = us, .mv = mv };
    s_hist_head = (s_hist_head + 1) % RATE_HISTORY_LEN;
    if (s_hist_count < RATE_HISTORY_LEN) s_hist_count++;
}

static const rate_sample_t *hist_oldest(void)
{
    if (s_hist_count == 0) return NULL;
    int idx = (s_hist_head - s_hist_count + RATE_HISTORY_LEN) % RATE_HISTORY_LEN;
    return &s_hist[idx];
}

/* mV/s rate of change (positive = charging up, negative = draining), or 0
 * with *valid=false if there isn't enough history yet. */
static float hist_rate_mv_per_s(bool *valid)
{
    *valid = false;
    if (s_hist_count < 2) return 0.0f;
    const rate_sample_t *oldest = hist_oldest();
    int newest_idx = (s_hist_head - 1 + RATE_HISTORY_LEN) % RATE_HISTORY_LEN;
    const rate_sample_t *newest = &s_hist[newest_idx];
    float span_s = (float)(newest->us - oldest->us) / 1000000.0f;
    if (span_s < (float)RATE_MIN_SPAN_S) return 0.0f;
    float d_mv = (float)newest->mv - (float)oldest->mv;
    *valid = true;
    return d_mv / span_s;
}

/* Generic single-cell LiPo open-circuit-voltage -> SOC% curve. Not
 * temperature/age/load compensated; good enough for an on-camera readout. */
static uint8_t voltage_to_pct(uint32_t mv)
{
    static const struct { uint32_t mv; uint8_t pct; } CURVE[] = {
        { 4200, 100 }, { 4150, 95 }, { 4100, 90 }, { 4000, 78 }, { 3900, 63 },
        { 3800, 48 }, { 3700, 33 }, { 3600, 18 }, { 3500, 10 }, { 3400, 5 },
        { 3300, 2 },  { 3000, 0 },
    };
    const int n = sizeof(CURVE) / sizeof(CURVE[0]);
    if (mv >= CURVE[0].mv) return 100;
    if (mv <= CURVE[n - 1].mv) return 0;
    for (int i = 0; i < n - 1; i++) {
        if (mv <= CURVE[i].mv && mv >= CURVE[i + 1].mv) {
            uint32_t span = CURVE[i].mv - CURVE[i + 1].mv;
            uint32_t off  = mv - CURVE[i + 1].mv;
            int pct_span = CURVE[i].pct - CURVE[i + 1].pct;
            return (uint8_t)(CURVE[i + 1].pct + (off * pct_span) / span);
        }
    }
    return 0;
}

static void battery_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));

        uint8_t raw = 0;
        esp_err_t err = i2c_read_reg(s_dev, STC8_REG_BATTERY, &raw, 1);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
            continue;
        }

        uint32_t adc_mv = (uint32_t)raw * STC8_ADC_VREF_MV / 255;
        uint32_t bat_mv = adc_mv * (DIV_R_BATTERY_OHM + DIV_R_GROUND_OHM) / DIV_R_GROUND_OHM;
        bool present = bat_mv >= NO_BATTERY_THRESHOLD_MV;
        ESP_LOGI(TAG, "raw=0x%02X (%u)  adc_mv=%u  bat_mv=%u  present=%d",
                 raw, raw, (unsigned)adc_mv, (unsigned)bat_mv, present);

        int64_t now = esp_timer_get_time();
        if (present != s_last_present) {
            /* Plug/unplug crossover invalidates the old trend. */
            hist_reset();
            s_last_present = present;
        }

        prop_battery_state_t state;
        uint8_t level_pct = 0;
        bool tte_valid = false, ttf_valid = false;
        uint32_t tte_min = 0, ttf_min = 0;

        if (!present) {
            state = PROP_BATT_NOT_PRESENT;
            adc_mv = 0;
            bat_mv = 0;
        } else {
            level_pct = voltage_to_pct(bat_mv);
            hist_push(now, bat_mv);

            bool rate_valid = false;
            float rate_mv_s = hist_rate_mv_per_s(&rate_valid);

            if (rate_valid && rate_mv_s > RATE_NOISE_FLOOR_MV_S) {
                state = PROP_BATT_CHARGING;
                /* Project remaining mV to FULLY_CHARGED_MV at the observed rate,
                 * then convert via the SOC curve's local slope (roughly — good
                 * enough for a ballpark ETA, not a lab figure). */
                if (bat_mv < FULLY_CHARGED_MV) {
                    float remain_mv = (float)FULLY_CHARGED_MV - (float)bat_mv;
                    ttf_min = (uint32_t)((remain_mv / rate_mv_s) / 60.0f);
                    ttf_valid = true;
                }
            } else if (rate_valid && rate_mv_s < -RATE_NOISE_FLOOR_MV_S) {
                state = PROP_BATT_NO_CHARGE;   /* draining: running on battery */
                uint32_t empty_mv = 3000;      /* curve floor */
                if (bat_mv > empty_mv) {
                    float remain_mv = (float)bat_mv - (float)empty_mv;
                    tte_min = (uint32_t)((remain_mv / -rate_mv_s) / 60.0f);
                    tte_valid = true;
                }
            } else if (bat_mv >= FULLY_CHARGED_MV) {
                state = PROP_BATT_FULLY_CHARGED;
            } else {
                state = PROP_BATT_IDLE;   /* flat trend, not at the top of the curve */
            }
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_data.online              = true;
        s_data.valid                = true;   /* a real read landed, even if it says "no battery" */
        s_data.adc_mv               = adc_mv;
        s_data.voltage_mv           = bat_mv;
        s_data.level_pct            = level_pct;
        s_data.state                 = state;
        s_data.led_state             = 0;   /* not exposed on this board's STC8 firmware */
        s_data.time_to_empty_valid   = tte_valid;
        s_data.time_to_empty_min     = tte_min;
        s_data.time_to_full_valid    = ttf_valid;
        s_data.time_to_full_min      = ttf_min;
        xSemaphoreGive(s_mutex);
    }
}

esp_err_t prop_battery_init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    esp_err_t probe = i2c_master_probe(i2c_bus_handle, STC8_I2C_ADDR, 1000);
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "STC8H1K08 not found at 0x%02X (%s) — battery telemetry offline",
                 STC8_I2C_ADDR, esp_err_to_name(probe));
        return ESP_ERR_NOT_FOUND;
    }

    s_dev = i2c_dev_register(STC8_I2C_ADDR);
    if (!s_dev) {
        ESP_LOGE(TAG, "i2c_dev_register failed");
        return ESP_FAIL;
    }

    s_online = true;
    hist_reset();
    xTaskCreate(battery_task, "prop_battery", 3072, NULL, 3, NULL);
    ESP_LOGI(TAG, "STC8H1K08 online at 0x%02X — battery telemetry running", STC8_I2C_ADDR);
    return ESP_OK;
}

bool prop_battery_available(void)
{
    return s_online;
}

void prop_battery_get_data(prop_battery_data_t *out)
{
    if (!out) return;
    if (!s_mutex) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_data;
    xSemaphoreGive(s_mutex);
}
