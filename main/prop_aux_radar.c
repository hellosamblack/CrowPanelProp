/* prop_aux_radar — dual auxiliary 24 GHz mmWave radar driver.
 *
 * Sensor A: Seeed MR24HPC1 (UART3, GPIO47 TX / GPIO48 RX, 115200, J2)
 *   Binary protocol: [0x53][0x59][ctrl][cmd][len_h][len_l][data…][CRC][0x54][0x43]
 *   Query-driven — must send a human-status query frame each second.
 *   Presence response: ctrl=0x80 cmd=0x01, data[0]: 0x00=clear, 0x01=present.
 *
 * Sensor B: DFRobot SEN0395 (UART1, GPIO34 TX / GPIO33 RX, 115200, J10)
 *   ASCII lines: "$JYBSS,<par1>,<par2>,<par3>,<par4>*\r\n"
 *   par1 == '0' → clear, par1 == '1' → present.
 *   Sensor is silent until commanded — must send "\rsensorStart\r" at boot.
 *
 * Both sensors are optional.  If no valid frame is received within 5000 ms
 * the sensor state rolls back to AUX_OFFLINE.  See prop_aux_radar.h.
 */

#include "prop_aux_radar.h"

#include <inttypes.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "PROP_AUX_RADAR"

/* ---- Hardware constants -------------------------------------------------- */

#define SEEED_UART       UART_NUM_3
#define SEEED_TX_GPIO    47   /* P4 → sensor, J2/pin2 */
#define SEEED_RX_GPIO    48   /* sensor → P4, J2/pin1 */
#define SEEED_BAUD       115200

#define SEN0395_UART     UART_NUM_1
#define SEN0395_TX_GPIO  34   /* P4 → sensor, J10/pin1 */
#define SEN0395_RX_GPIO  33   /* sensor → P4, J10/pin2 */
#define SEN0395_BAUD     115200

#define RADAR_RX_BUF     512  /* bytes, rx ring buffer per UART */
#define OFFLINE_MS       5000 /* ms without a valid frame → AUX_OFFLINE */
#define TASK_STACK       3072 /* bytes per task */
#define TASK_PRIO        3
#define TASK_CORE        1

/* ---- Cached state -------------------------------------------------------- */

static aux_radar_state_t s_seeed    = AUX_OFFLINE;
static aux_radar_state_t s_sen0395  = AUX_OFFLINE;

/* Last valid-frame timestamps.  0 = never received (sentinel).
 * Stored as ts ? ts : 1 so 0 is unambiguously "never". */
static uint32_t s_seeed_last_ms   = 0;
static uint32_t s_sen0395_last_ms = 0;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ---- Seeed MR24HPC1 24 GHz mmWave parser task ----------------------------
 *
 * Frame layout (from Seeed ESPHome component seeed_mr24hpc1.cpp):
 *   [0]       0x53          header byte 1
 *   [1]       0x59          header byte 2
 *   [2]       ctrl          control word
 *   [3]       cmd           command word
 *   [4]       len_h         data length high byte (always 0x00)
 *   [5]       len_l         data length low byte  (0–32)
 *   [6..N-3]  data          payload (len_l bytes)
 *   [N-2]     CRC           sum of bytes [0..N-3] & 0xFF
 *   [N-1]     0x54          tail byte 1
 *   [N]       0x43          tail byte 2
 *   Total frame length = 9 + len_l bytes.
 *
 * The sensor is query-driven.  We poll at ~1 Hz with a human-status query.
 * Any valid frame (heartbeat, config replies, etc.) resets the offline
 * watchdog.  The cached state only updates on ctrl=0x80 cmd=0x01 reports.
 */

#define MR24_HDR0          0x53u
#define MR24_HDR1          0x59u
#define MR24_TAIL0         0x54u
#define MR24_TAIL1         0x43u
#define MR24_MAX_DATA_LEN  32
#define MR24_MAX_FRAME     (9 + MR24_MAX_DATA_LEN)
#define MR24_BUF_LEN       (MR24_MAX_FRAME * 2)

#define MR24_CTL_HUMAN     0x80u  /* human information control word */
#define MR24_CMD_PRESENCE  0x01u  /* human status: data[0] 0=clear, 1=present */

/* Human-status query frame.
 * CRC = (0x53+0x59+0x80+0x01+0x00+0x01+0x0F) & 0xFF = 0x3D */
static const uint8_t s_seeed_query[] = {
    0x53, 0x59, 0x80, 0x01, 0x00, 0x01, 0x0F, 0x3D, 0x54, 0x43
};

/* CRC covers all bytes except the final three (crc, tail0, tail1). */
static uint8_t mr24_crc(const uint8_t *frame, int total_len)
{
    unsigned int s = 0;
    for (int i = 0; i < total_len - 3; i++) s += frame[i];
    return (uint8_t)(s & 0xFFu);
}

static void seeed_task(void *arg)
{
    (void)arg;

    uint8_t  buf[MR24_BUF_LEN];
    int      buf_len         = 0;
    uint32_t last_query_ms   = 0;
    uint32_t task_start_ms   = now_ms();
    uint32_t total_rx_bytes  = 0;
    bool     no_frame_warned = false;

    for (;;) {
        /* Periodic human-status query (~1 Hz). */
        uint32_t loop_now = now_ms();
        if (loop_now - last_query_ms >= 1000) {
            uart_write_bytes(SEEED_UART, s_seeed_query, sizeof(s_seeed_query));
            last_query_ms = loop_now;
        }

        /* Read available bytes. */
        int space = (int)sizeof(buf) - buf_len;
        if (space > 0) {
            int n = uart_read_bytes(SEEED_UART,
                                    buf + buf_len,
                                    (uint32_t)space,
                                    pdMS_TO_TICKS(50));
            if (n > 0) {
                buf_len += n;
                total_rx_bytes += (uint32_t)n;
            }
        }

        /* Scan for header 0x53 0x59. */
        int hdr_pos = -1;
        for (int i = 0; i <= buf_len - 2; i++) {
            if (buf[i] == MR24_HDR0 && buf[i + 1] == MR24_HDR1) {
                hdr_pos = i;
                break;
            }
        }
        if (hdr_pos < 0) {
            if (buf_len > 0) { buf[0] = buf[buf_len - 1]; buf_len = 1; }
            goto check_timeout;
        }
        if (hdr_pos > 0) {
            memmove(buf, buf + hdr_pos, (size_t)(buf_len - hdr_pos));
            buf_len -= hdr_pos;
        }

        /* Need at least 6 bytes to decode the length field. */
        if (buf_len < 6) goto check_timeout;

        /* len_h must be 0; len_l must be ≤ MR24_MAX_DATA_LEN. */
        if (buf[4] != 0x00 || buf[5] > MR24_MAX_DATA_LEN) {
            memmove(buf, buf + 1, (size_t)(buf_len - 1));
            buf_len--;
            goto check_timeout;
        }
        int data_len  = buf[5];
        int frame_len = 9 + data_len;

        if (buf_len < frame_len) goto check_timeout;

        /* Validate tail. */
        if (buf[frame_len - 2] != MR24_TAIL0 || buf[frame_len - 1] != MR24_TAIL1) {
            memmove(buf, buf + 1, (size_t)(buf_len - 1));
            buf_len--;
            goto check_timeout;
        }

        /* Validate CRC. */
        uint8_t expected = mr24_crc(buf, frame_len);
        if (buf[frame_len - 3] != expected) {
            ESP_LOGW(TAG, "SEEED CRC mismatch (got 0x%02X expected 0x%02X)",
                     buf[frame_len - 3], expected);
            memmove(buf, buf + 1, (size_t)(buf_len - 1));
            buf_len--;
            goto check_timeout;
        }

        /* Valid frame — update watchdog timestamp for any frame type. */
        {
            uint32_t ts = now_ms();
            portENTER_CRITICAL(&s_mux);
            s_seeed_last_ms = ts ? ts : 1;
            portEXIT_CRITICAL(&s_mux);
        }

        /* Update presence state on ctrl=0x80 cmd=0x01 frames. */
        {
            uint8_t ctl = buf[2];
            uint8_t cmd = buf[3];
            if (ctl == MR24_CTL_HUMAN && cmd == MR24_CMD_PRESENCE && data_len >= 1) {
                aux_radar_state_t st = (buf[6] != 0x00) ? AUX_PRESENT : AUX_CLEAR;
                portENTER_CRITICAL(&s_mux);
                s_seeed = st;
                portEXIT_CRITICAL(&s_mux);
                ESP_LOGI(TAG, "SEEED presence=%s",
                         st == AUX_PRESENT ? "PRESENT" : "CLEAR");
            } else {
                ESP_LOGD(TAG, "SEEED frame ctl=0x%02X cmd=0x%02X data_len=%d",
                         ctl, cmd, data_len);
            }
        }

        /* Consume frame. */
        memmove(buf, buf + frame_len, (size_t)(buf_len - frame_len));
        buf_len -= frame_len;

check_timeout:
        {
            portENTER_CRITICAL(&s_mux);
            uint32_t last = s_seeed_last_ms;
            portEXIT_CRITICAL(&s_mux);

            uint32_t now = now_ms();
            if (last != 0 && (now - last) > OFFLINE_MS) {
                portENTER_CRITICAL(&s_mux);
                s_seeed = AUX_OFFLINE;
                portEXIT_CRITICAL(&s_mux);
                ESP_LOGW(TAG, "SEEED offline (no frame for %u ms)", OFFLINE_MS);
            } else if (!no_frame_warned && last == 0 &&
                       (now - task_start_ms) > OFFLINE_MS) {
                no_frame_warned = true;
                ESP_LOGW(TAG, "SEEED no valid frame in %u ms (rx %"PRIu32" bytes) — "
                         "check wiring GPIO%d(TX)/GPIO%d(RX)",
                         OFFLINE_MS, total_rx_bytes, SEEED_TX_GPIO, SEEED_RX_GPIO);
            }
        }
    }
}

/* ---- DFRobot SEN0395 ASCII-frame parser task ----------------------------- */
/*
 * The SEN0395 emits ASCII lines at ~1 Hz:
 *   "$JYBSS,<par1>,<par2>,<par3>,<par4>*\r\n"
 * par1: '0' = clear, '1' = present.
 *
 * The sensor is silent until commanded.  We send "\rsensorStop\r" then
 * "\rsensorStart\r" at task startup (same sequence as the old prop_radar.c).
 * The leading \r absorbs the first-byte glitch on the leapMMW CLI.
 * If the stream goes silent (sensor self-reset), we re-issue sensorStart.
 */

#define SEN0395_LINE_MAX  64

static void sen0395_kick(void)
{
    uart_write_bytes(SEN0395_UART, "\rsensorStart\r", 13);
}

static void sen0395_task(void *arg)
{
    (void)arg;

    /* Send sensorStop then sensorStart before entering the read loop. */
    vTaskDelay(pdMS_TO_TICKS(700));
    uart_write_bytes(SEN0395_UART, "\rsensorStop\r",  12);
    vTaskDelay(pdMS_TO_TICKS(350));
    sen0395_kick();
    ESP_LOGI(TAG, "SEN0395 sensorStart sent");

    char     line[SEN0395_LINE_MAX + 1];
    int      line_len        = 0;
    uint32_t task_start_ms   = now_ms();   /* measured after init sequence */
    uint32_t total_rx_bytes  = 0;
    bool     no_frame_warned = false;

    for (;;) {
        uint8_t byte;
        int n = uart_read_bytes(SEN0395_UART, &byte, 1, pdMS_TO_TICKS(50));
        if (n <= 0) {
            goto check_timeout_sen;
        }
        total_rx_bytes++;

        if (byte == '\r') {
            goto check_timeout_sen;
        }

        if (byte == '\n') {
            line[line_len] = '\0';

            if (line_len >= 8 && strncmp(line, "$JYBSS,", 7) == 0) {
                char par1 = line[7];
                aux_radar_state_t state = (par1 == '1') ? AUX_PRESENT : AUX_CLEAR;

                uint32_t ts = now_ms();
                portENTER_CRITICAL(&s_mux);
                s_sen0395         = state;
                s_sen0395_last_ms = ts ? ts : 1;
                portEXIT_CRITICAL(&s_mux);

                ESP_LOGI(TAG, "SEN0395 frame ok, par1='%c' presence=%d",
                         par1, (int)state);
            }

            line_len = 0;
            goto check_timeout_sen;
        }

        if (line_len < SEN0395_LINE_MAX) {
            line[line_len++] = (char)byte;
        }

check_timeout_sen:
        {
            portENTER_CRITICAL(&s_mux);
            uint32_t last = s_sen0395_last_ms;
            portEXIT_CRITICAL(&s_mux);

            uint32_t now = now_ms();
            if (last != 0 && (now - last) > OFFLINE_MS) {
                portENTER_CRITICAL(&s_mux);
                s_sen0395     = AUX_OFFLINE;
                s_sen0395_last_ms = 0;
                portEXIT_CRITICAL(&s_mux);
                ESP_LOGW(TAG, "SEN0395 offline — re-issuing sensorStart");
                sen0395_kick();
                task_start_ms   = now;
                no_frame_warned = false;
            } else if (!no_frame_warned && last == 0 &&
                       (now - task_start_ms) > OFFLINE_MS) {
                no_frame_warned = true;
                ESP_LOGW(TAG, "SEN0395 no valid frame in %u ms (rx %"PRIu32" bytes) — "
                         "check wiring GPIO%d(TX)/GPIO%d(RX)",
                         OFFLINE_MS, total_rx_bytes, SEN0395_TX_GPIO, SEN0395_RX_GPIO);
            }
        }
    }
}

/* ---- Public API ---------------------------------------------------------- */

esp_err_t prop_aux_radar_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = SEEED_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t first_err = ESP_OK;
    bool seeed_ok   = false;
    bool sen0395_ok = false;

    /* ---- Sensor A: Seeed MR24HPC1 on UART3 ---- */
    esp_err_t err;
    err = uart_driver_install(SEEED_UART, RADAR_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SEEED uart_driver_install: %s", esp_err_to_name(err));
        if (first_err == ESP_OK) first_err = err;
    } else {
        err = uart_param_config(SEEED_UART, &cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "SEEED uart_param_config: %s", esp_err_to_name(err));
            uart_driver_delete(SEEED_UART);
            if (first_err == ESP_OK) first_err = err;
        } else {
            err = uart_set_pin(SEEED_UART,
                               SEEED_TX_GPIO, SEEED_RX_GPIO,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "SEEED uart_set_pin: %s", esp_err_to_name(err));
                uart_driver_delete(SEEED_UART);
                if (first_err == ESP_OK) first_err = err;
            } else {
                BaseType_t r = xTaskCreatePinnedToCore(seeed_task, "prop_seeed",
                                                       TASK_STACK, NULL,
                                                       TASK_PRIO, NULL, TASK_CORE);
                if (r != pdPASS) {
                    ESP_LOGW(TAG, "SEEED xTaskCreatePinnedToCore failed");
                    uart_driver_delete(SEEED_UART);
                    if (first_err == ESP_OK) first_err = ESP_ERR_NO_MEM;
                } else {
                    seeed_ok = true;
                    ESP_LOGI(TAG, "MR24HPC1 UART3 ok (GPIO%d/GPIO%d @ %d 8N1)",
                             SEEED_TX_GPIO, SEEED_RX_GPIO, SEEED_BAUD);
                }
            }
        }
    }

    /* ---- Sensor B: DFRobot SEN0395 on UART1 ---- */
    err = uart_driver_install(SEN0395_UART, RADAR_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SEN0395 uart_driver_install: %s", esp_err_to_name(err));
        if (first_err == ESP_OK) first_err = err;
    } else {
        err = uart_param_config(SEN0395_UART, &cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "SEN0395 uart_param_config: %s", esp_err_to_name(err));
            uart_driver_delete(SEN0395_UART);
            if (first_err == ESP_OK) first_err = err;
        } else {
            /* Route UART1 to GPIO25 (TX) and GPIO27 (RX) via the GPIO matrix. */
            err = uart_set_pin(SEN0395_UART,
                               SEN0395_TX_GPIO, SEN0395_RX_GPIO,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "SEN0395 uart_set_pin: %s", esp_err_to_name(err));
                uart_driver_delete(SEN0395_UART);
                if (first_err == ESP_OK) first_err = err;
            } else {
                BaseType_t r = xTaskCreatePinnedToCore(sen0395_task, "prop_sen0395",
                                                       TASK_STACK, NULL,
                                                       TASK_PRIO, NULL, TASK_CORE);
                if (r != pdPASS) {
                    ESP_LOGW(TAG, "SEN0395 xTaskCreatePinnedToCore failed");
                    uart_driver_delete(SEN0395_UART);
                    if (first_err == ESP_OK) first_err = ESP_ERR_NO_MEM;
                } else {
                    sen0395_ok = true;
                    ESP_LOGI(TAG, "SEN0395 UART1 ok (GPIO%d/GPIO%d @ %d 8N1)",
                             SEN0395_TX_GPIO, SEN0395_RX_GPIO, SEN0395_BAUD);
                }
            }
        }
    }

    /* Return ESP_OK if at least one sensor came up; return the first error only
     * when both sensors failed to initialise. */
    if (seeed_ok || sen0395_ok) {
        return ESP_OK;
    }
    return first_err;
}

aux_radar_state_t prop_aux_radar_seeed(void)
{
    portENTER_CRITICAL(&s_mux);
    aux_radar_state_t st = s_seeed;
    portEXIT_CRITICAL(&s_mux);
    return st;
}

aux_radar_state_t prop_aux_radar_sen0395(void)
{
    portENTER_CRITICAL(&s_mux);
    aux_radar_state_t st = s_sen0395;
    portEXIT_CRITICAL(&s_mux);
    return st;
}
