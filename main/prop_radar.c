/* prop_radar — DFRobot mmWave presence radar (JYSJ "$JYBSS" firmware) on UART2.
 * RX=GPIO54 (radar TX), TX=GPIO53 (radar RX). The sensor streams binary presence
 * frames; its leapMMW CLI also answers config queries (range/sensitivity/latency/
 * version) which we read once at boot. See header for the data model. */
#include "prop_radar.h"

#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "PROP_RADAR"

#define RADAR_UART     UART_NUM_2   /* controller; pins routed via the GPIO matrix */
#define RADAR_RX_GPIO  54           /* RXD2 — radar TX in (radio-module NRST/CS pin) */
#define RADAR_TX_GPIO  53           /* TXD2 — radar RX, for commands (radio IRQ/CE pin) */
#define RADAR_BAUD     115200

/* ---- Cached state (written by the reader task, read by the UI) ----------- */
static volatile bool      s_available;     /* at least one valid frame parsed */
static volatile bool      s_present;        /* latest presence verdict */
static volatile uint32_t  s_frames;         /* total $JYBSS frames parsed */
static volatile uint32_t  s_resets;         /* sensor self-resets observed */
static volatile TickType_t s_change_tick;   /* tick of last presence change */

/* Saved sensor config, read once at boot via the CLI. */
static volatile float s_range_min = -1, s_range_max = -1;  /* metres; -1 = unknown */
static volatile int   s_sens = -1;                          /* 0..9; -1 = unknown */
static volatile float s_lat_delay = -1, s_lat_hold = -1;    /* seconds */
static char s_swv[40], s_hwv[40];

/* Presence history ring (one sample per frame, ~1 Hz) for the timeline strip. */
#define RADAR_HIST 90
static uint8_t       s_hist[RADAR_HIST];
static volatile int  s_hist_head, s_hist_count;

/* Which config query we're awaiting a "Response ..." line for. */
typedef enum { Q_NONE, Q_RANGE, Q_SENS, Q_LAT } pending_t;
static pending_t s_pending;

/* Issue sensorStart. The leading \r absorbs the sensor's first-byte glitch (the
 * first char after the line idles is dropped) and doubles as a CLI line break. */
static void radar_kick(void)
{
    uart_write_bytes(RADAR_UART, "\rsensorStart\r", 13);
}

static void hist_push(bool present)
{
    s_hist[s_hist_head] = present ? 1 : 0;
    s_hist_head = (s_hist_head + 1) % RADAR_HIST;
    if (s_hist_count < RADAR_HIST) {
        s_hist_count++;
    }
}

static void parse_line(const char *line, int len)
{
    /* The sensor resets itself occasionally (prints "reset system"); restart its
     * stream the moment we see that, instead of waiting for the silence watchdog. */
    if (strstr(line, "reset system")) {
        s_resets++;
        ESP_LOGW(TAG, "sensor reset — restarting stream");
        radar_kick();
        return;
    }

    /* Presence frame — may be bare or prefixed with the CLI prompt. */
    const char *f = strstr(line, "$JYBSS");
    if (f && (int)(line + len - f) >= 8) {
        bool present = (f[7] == '1');
        if (!s_available || present != s_present) {
            s_change_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "presence -> %s", present ? "PRESENT" : "clear");
        }
        s_present = present;
        s_available = true;
        s_frames++;
        hist_push(present);
        return;
    }

    /* Config-query plumbing: the CLI echoes the command, then a "Response ..."
     * line, then "Done". Note the awaited query off the echo, fill it off the
     * response. Versions arrive with explicit prefixes. */
    if (strncmp(line, "SoftwareVersion:", 16) == 0) {
        size_t n = strnlen(line + 16, sizeof(s_swv) - 1);
        memcpy(s_swv, line + 16, n);
        s_swv[n] = '\0';
        return;
    }
    if (strncmp(line, "HardwareVersion:", 16) == 0) {
        size_t n = strnlen(line + 16, sizeof(s_hwv) - 1);
        memcpy(s_hwv, line + 16, n);
        s_hwv[n] = '\0';
        return;
    }
    if (strstr(line, "getRange"))            { s_pending = Q_RANGE; return; }
    if (strstr(line, "getSensitivity"))      { s_pending = Q_SENS;  return; }
    if (strstr(line, "getLatency"))          { s_pending = Q_LAT;   return; }
    if (strncmp(line, "Response", 8) == 0) {
        const char *v = line + 8;
        if (s_pending == Q_RANGE) {
            float a, b;
            if (sscanf(v, "%f %f", &a, &b) == 2) { s_range_min = a; s_range_max = b; }
        } else if (s_pending == Q_SENS) {
            int a;
            if (sscanf(v, "%d", &a) == 1) { s_sens = a; }
        } else if (s_pending == Q_LAT) {
            float a, b;
            if (sscanf(v, "%f %f", &a, &b) == 2) { s_lat_delay = a; s_lat_hold = b; }
        }
        s_pending = Q_NONE;
        return;
    }
    /* Anything else (banner / Done / Error) — quiet by default. */
}

/* Read the sensor's saved config once: stop it (config mode), query, resume. The
 * values are flash-saved on the sensor so a single read survives its resets. */
static void radar_read_config(void)
{
    static const char *q[] = {
        "\rsensorStop\r", "\rgetRange\r", "\rgetSensitivity\r",
        "\rgetLatency\r", "\rgetSWV\r", "\rgetHWV\r", "\rsensorStart\r",
    };
    vTaskDelay(pdMS_TO_TICKS(700));   /* let the sensor finish its own boot */
    for (int i = 0; i < (int)(sizeof(q) / sizeof(q[0])); i++) {
        uart_write_bytes(RADAR_UART, q[i], strlen(q[i]));
        vTaskDelay(pdMS_TO_TICKS(350));
    }
    ESP_LOGI(TAG, "config read issued");
}

static void radar_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    char line[160];
    int li = 0;

    radar_read_config();
    TickType_t last_seen = xTaskGetTickCount();
    uint32_t last_frames = 0;

    for (;;) {
        int n = uart_read_bytes(RADAR_UART, buf, sizeof(buf), pdMS_TO_TICKS(200));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (li > 0) {
                    line[li] = '\0';
                    parse_line(line, li);
                    li = 0;
                }
            } else if (li < (int)sizeof(line) - 1) {
                line[li++] = c;
            } else {
                li = 0;   /* overrun: drop the line */
            }
        }

        /* Liveness watchdog: $JYBSS arrives ~1 Hz. If the stream goes silent for
         * 5 s (e.g. a sensor reset we didn't catch), re-issue sensorStart. */
        TickType_t now = xTaskGetTickCount();
        if (s_frames != last_frames) {
            last_frames = s_frames;
            last_seen = now;
        } else if (now - last_seen > pdMS_TO_TICKS(5000)) {
            ESP_LOGW(TAG, "no frames for 5 s — re-issuing sensorStart");
            radar_kick();
            last_seen = now;
        }
    }
}

esp_err_t prop_radar_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = RADAR_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(RADAR_UART, 2048, 256, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return err;
    }
    esp_err_t pe = uart_param_config(RADAR_UART, &cfg);
    esp_err_t sp = uart_set_pin(RADAR_UART, RADAR_TX_GPIO, RADAR_RX_GPIO,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "param_config=%s set_pin=%s", esp_err_to_name(pe), esp_err_to_name(sp));

    if (xTaskCreatePinnedToCore(radar_task, "prop_radar", 4096, NULL, 5, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "mmWave radar: RXD2=GPIO%d TXD2=GPIO%d @ %d 8N1",
             RADAR_RX_GPIO, RADAR_TX_GPIO, RADAR_BAUD);
    return ESP_OK;
}

bool     prop_radar_available(void) { return s_available; }
bool     prop_radar_present(void)   { return s_present; }
uint32_t prop_radar_frames(void)    { return s_frames; }
uint32_t prop_radar_resets(void)    { return s_resets; }

uint32_t prop_radar_dwell_s(void)
{
    if (!s_available) {
        return 0;
    }
    TickType_t d = xTaskGetTickCount() - s_change_tick;
    return (uint32_t)(d * portTICK_PERIOD_MS / 1000);
}

void prop_radar_get_config(float *rmin, float *rmax, int *sens,
                           float *lat_delay, float *lat_hold)
{
    if (rmin)      *rmin = s_range_min;
    if (rmax)      *rmax = s_range_max;
    if (sens)      *sens = s_sens;
    if (lat_delay) *lat_delay = s_lat_delay;
    if (lat_hold)  *lat_hold = s_lat_hold;
}

const char *prop_radar_swv(void) { return s_swv[0] ? s_swv : "?"; }
const char *prop_radar_hwv(void) { return s_hwv[0] ? s_hwv : "?"; }

int prop_radar_history(uint8_t *out, int max)
{
    int count = s_hist_count;
    if (count > max) {
        count = max;
    }
    /* Walk oldest..newest ending at head-1. */
    int start = (s_hist_head - count + RADAR_HIST * 4) % RADAR_HIST;
    for (int i = 0; i < count; i++) {
        out[i] = s_hist[(start + i) % RADAR_HIST];
    }
    return count;
}
