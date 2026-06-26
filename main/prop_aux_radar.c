/* prop_aux_radar — dual auxiliary 24 GHz mmWave radar driver.
 *
 * Sensor A: Seeed 101991030 (UART3, GPIO34 TX / GPIO33 RX, 115200, J10)
 *   Binary HiLink-family frames: 0xAA 0xFF header … 0x55 0xCC tail.
 *   Byte at header+4: 0x00 = clear, non-zero = present.
 *
 * Sensor B: DFRobot SEN0395 (UART1, GPIO25 TX / GPIO27 RX, 115200, J7)
 *   ASCII lines: "$JYBSS,<par1>,<par2>,<par3>,<par4>*"
 *   par1 == '0' → clear, par1 == '1' → present.
 *
 * Both sensors are optional.  If no valid frame is received within 5000 ms
 * the sensor state rolls back to AUX_OFFLINE.  See prop_aux_radar.h.
 */

#include "prop_aux_radar.h"

#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "PROP_AUX_RADAR"

/* ---- Hardware constants -------------------------------------------------- */

#define SEEED_UART       UART_NUM_3
#define SEEED_TX_GPIO    34   /* P4 → sensor */
#define SEEED_RX_GPIO    33   /* sensor → P4 */
#define SEEED_BAUD       115200

#define SEN0395_UART     UART_NUM_1
#define SEN0395_TX_GPIO  25   /* P4 → sensor */
#define SEN0395_RX_GPIO  27   /* sensor → P4 */
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

/* ---- Seeed binary-frame parser task -------------------------------------- */
/*
 * Frame structure (HiLink family):
 *   [0] 0xAA  [1] 0xFF  — header magic
 *   [2..N-3]  — data bytes (variable length)
 *   [N-2] 0x55  [N-1] 0xCC  — tail magic
 *
 * Presence flag: byte at offset 4 (0x00 = clear, non-zero = present).
 * We scan for the 2-byte header, then search up to 30 bytes ahead for the
 * 2-byte tail — tolerant of frames we don't have a complete spec for.
 */

#define SEEED_HDR0       0xAAu
#define SEEED_HDR1       0xFFu
#define SEEED_TAIL0      0x55u
#define SEEED_TAIL1      0xCCu
#define SEEED_MAX_FRAME  32   /* header + at most 30 data bytes + tail */
#define SEEED_BUF_LEN    (SEEED_MAX_FRAME * 2)
#define PRESENCE_OFFSET  4    /* byte at header+4 carries presence flag */

static void seeed_task(void *arg)
{
    (void)arg;

    uint8_t buf[SEEED_BUF_LEN];
    int     buf_len = 0;

    for (;;) {
        /* Read available bytes into the tail of buf. */
        int space = (int)sizeof(buf) - buf_len;
        if (space > 0) {
            int n = uart_read_bytes(SEEED_UART,
                                    buf + buf_len,
                                    (uint32_t)space,
                                    pdMS_TO_TICKS(50));
            if (n > 0) {
                buf_len += n;
            }
        }

        /* Scan for the 2-byte header (0xAA 0xFF). */
        int hdr_pos = -1;
        for (int i = 0; i <= buf_len - 2; i++) {
            if (buf[i] == SEEED_HDR0 && buf[i + 1] == SEEED_HDR1) {
                hdr_pos = i;
                break;
            }
        }

        if (hdr_pos < 0) {
            /* No header found; keep last byte in case it is the first header byte. */
            if (buf_len > 0) {
                buf[0]  = buf[buf_len - 1];
                buf_len = 1;
            }
            goto check_timeout;
        }

        /* Discard bytes before the header. */
        if (hdr_pos > 0) {
            memmove(buf, buf + hdr_pos, (size_t)(buf_len - hdr_pos));
            buf_len -= hdr_pos;
        }

        /* Need at least header (2) + presence byte + 2 tail bytes = 5 bytes to parse. */
        if (buf_len < 5) {
            goto check_timeout;
        }

        /* Scan for tail (0x55 0xCC) within the next 30 bytes after the header.
         * The tail cannot start before offset 2 (header occupies [0..1]). */
        int tail_pos = -1;
        int scan_end = buf_len - 1;
        if (scan_end > (int)(2 + 30)) {
            scan_end = 2 + 30;
        }
        for (int i = 2; i < scan_end; i++) {
            if (buf[i] == SEEED_TAIL0 && buf[i + 1] == SEEED_TAIL1) {
                tail_pos = i;
                break;
            }
        }

        if (tail_pos < 0) {
            /* Tail not yet arrived (or not a real frame).
             * If we've already buffered the max possible frame and still no tail,
             * the header byte is junk — drop it and resync. */
            if (buf_len >= (int)(2 + 30 + 2)) {
                memmove(buf, buf + 1, (size_t)(buf_len - 1));
                buf_len--;
            }
            goto check_timeout;
        }

        /* We have a complete frame: buf[0..tail_pos+1].
         * Extract presence flag. */
        aux_radar_state_t state = AUX_CLEAR;
        if (tail_pos > PRESENCE_OFFSET) {
            /* buf[PRESENCE_OFFSET] is the presence byte (header is at [0..1]). */
            state = (buf[PRESENCE_OFFSET] != 0x00) ? AUX_PRESENT : AUX_CLEAR;
        }

        uint32_t ts = now_ms();
        portENTER_CRITICAL(&s_mux);
        s_seeed         = state;
        s_seeed_last_ms = ts ? ts : 1;
        portEXIT_CRITICAL(&s_mux);

        ESP_LOGD(TAG, "SEEED frame ok, presence=%d", (int)state);

        /* Consume the frame from the buffer. */
        int frame_end = tail_pos + 2;
        memmove(buf, buf + frame_end, (size_t)(buf_len - frame_end));
        buf_len -= frame_end;

check_timeout:
        /* Check if the sensor has gone silent. */
        portENTER_CRITICAL(&s_mux);
        uint32_t last = s_seeed_last_ms;
        portEXIT_CRITICAL(&s_mux);

        if (last != 0 && (now_ms() - last) > OFFLINE_MS) {
            portENTER_CRITICAL(&s_mux);
            s_seeed = AUX_OFFLINE;
            portEXIT_CRITICAL(&s_mux);
            ESP_LOGW(TAG, "SEEED offline (no frame for %u ms)", OFFLINE_MS);
        }
    }
}

/* ---- DFRobot SEN0395 ASCII-frame parser task ----------------------------- */
/*
 * The SEN0395 emits ASCII lines at ~1 Hz:
 *   "$JYBSS,<par1>,<par2>,<par3>,<par4>*\r\n"
 * par1: '0' = clear, '1' = present.
 * We accumulate bytes into a line buffer (max 64 bytes), look for '\n',
 * then check if the line starts with "$JYBSS," and read offset 7 for par1.
 */

#define SEN0395_LINE_MAX  64

static void sen0395_task(void *arg)
{
    (void)arg;

    char line[SEN0395_LINE_MAX + 1];
    int  line_len = 0;

    for (;;) {
        uint8_t byte;
        int n = uart_read_bytes(SEN0395_UART, &byte, 1, pdMS_TO_TICKS(50));
        if (n <= 0) {
            goto check_timeout_sen;
        }

        if (byte == '\r') {
            /* Ignore carriage return. */
            goto check_timeout_sen;
        }

        if (byte == '\n') {
            /* End of line — try to parse. */
            line[line_len] = '\0';

            if (line_len >= 8 && strncmp(line, "$JYBSS,", 7) == 0) {
                char par1 = line[7];
                aux_radar_state_t state = (par1 == '1') ? AUX_PRESENT : AUX_CLEAR;

                uint32_t ts = now_ms();
                portENTER_CRITICAL(&s_mux);
                s_sen0395         = state;
                s_sen0395_last_ms = ts ? ts : 1;
                portEXIT_CRITICAL(&s_mux);

                ESP_LOGD(TAG, "SEN0395 frame ok, par1='%c' presence=%d",
                         par1, (int)state);
            }

            line_len = 0;
            goto check_timeout_sen;
        }

        /* Accumulate; drop bytes that overflow the line buffer (resync on next '\n'). */
        if (line_len < SEN0395_LINE_MAX) {
            line[line_len++] = (char)byte;
        }
        goto check_timeout_sen;

check_timeout_sen:
        /* Check if the sensor has gone silent. */
        portENTER_CRITICAL(&s_mux);
        uint32_t last = s_sen0395_last_ms;
        portEXIT_CRITICAL(&s_mux);

        if (last != 0 && (now_ms() - last) > OFFLINE_MS) {
            portENTER_CRITICAL(&s_mux);
            s_sen0395 = AUX_OFFLINE;
            portEXIT_CRITICAL(&s_mux);
            ESP_LOGW(TAG, "SEN0395 offline (no frame for %u ms)", OFFLINE_MS);
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

    /* ---- Sensor A: Seeed 101991030 on UART3 ---- */
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
                    ESP_LOGI(TAG, "SEEED UART3 ok (GPIO%d/GPIO%d @ %d 8N1)",
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
