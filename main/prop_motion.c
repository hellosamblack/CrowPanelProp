/* prop_motion — HiLink LD2450 24 GHz multi-target mmWave radar on UART2.
 * TX=GPIO53 (P4→LD2450), RX=GPIO54 (LD2450→P4), 256000 8N1.
 * See prop_motion.h for the public API and hardware/protocol notes. */
#include "prop_motion.h"

#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "PROP_MOTION"

#define MOTION_UART      UART_NUM_2
#define MOTION_TX_GPIO   53   /* P4 TXD2 → LD2450 RX */
#define MOTION_RX_GPIO   54   /* P4 RXD2 ← LD2450 TX */
#define MOTION_BAUD      256000

/* LD2450 frame constants */
#define FRAME_LEN        30   /* total bytes per frame */
#define HEADER_LEN       4    /* AA FF 03 00 */
#define TARGET_DATA_LEN  24   /* 3 targets × 8 bytes */
#define TAIL_LEN         2    /* 55 CC */

static const uint8_t FRAME_HEADER[HEADER_LEN] = { 0xAA, 0xFF, 0x03, 0x00 };
static const uint8_t FRAME_TAIL[TAIL_LEN]     = { 0x55, 0xCC };

/* ---- Cached state (written by reader task, read by UI) ------------------- */
static bool              s_available;
static prop_motion_target_t s_targets[PROP_MOTION_MAX_TARGETS];
static int               s_target_count;
static uint32_t          s_last_seen_ms;   /* ms timestamp of last valid frame */
static portMUX_TYPE      s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ---- Frame parser -------------------------------------------------------- */

/* The LD2450 encodes signed coordinates/speed as SIGN-MAGNITUDE, not two's
 * complement: bit 15 is the sign (1 = positive, 0 = negative); bits 0..14 are
 * the magnitude. Decoding the raw word as a plain int16 makes a real +913 mm
 * (0x8391) read as -31855 (≈ -32 m) — the classic LD2450 parsing bug. */
static inline int16_t ld2450_signmag(const uint8_t *lo)
{
    uint16_t raw = (uint16_t)lo[0] | ((uint16_t)lo[1] << 8);
    int16_t mag = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? mag : (int16_t)(-mag);
}

/* Parse 8-byte target block into a prop_motion_target_t.
 * Returns true when the target slot is active (any non-zero field). */
static bool parse_target(const uint8_t *p, prop_motion_target_t *t)
{
    int16_t  x   = ld2450_signmag(p + 0);
    int16_t  y   = ld2450_signmag(p + 2);
    int16_t  spd = ld2450_signmag(p + 4);   /* raw units: cm/s (protocol PDF), not mm/s */
    uint16_t dr  = (uint16_t)p[6] | ((uint16_t)p[7] << 8);

    t->x_mm        = x;
    t->y_mm        = y;
    t->speed_mm_s  = (int16_t)(spd * 10);   /* cm/s -> mm/s so the field name is honest */
    t->dist_res_mm = dr;

    /* Inactive slots are all-zero; a real target has a non-zero coordinate. */
    return (x != 0 || y != 0);
}

/* ---- Background reader task --------------------------------------------- */

static void motion_task(void *arg)
{
    (void)arg;

    /* Working buffer: large enough to scan for a header and buffer one full
     * frame even if bytes dribble in across multiple reads. */
    uint8_t buf[FRAME_LEN * 2];
    int     buf_len = 0;    /* bytes currently held in buf */

    for (;;) {
        /* Drain as many bytes as are available into the tail of buf. */
        int space = (int)sizeof(buf) - buf_len;
        if (space > 0) {
            int n = uart_read_bytes(MOTION_UART,
                                    buf + buf_len,
                                    (uint32_t)space,
                                    pdMS_TO_TICKS(50));
            if (n > 0) {
                buf_len += n;
            }
        }

        /* Scan for the 4-byte header. */
        int hdr_pos = -1;
        for (int i = 0; i <= buf_len - HEADER_LEN && buf_len >= HEADER_LEN; i++) {
            if (memcmp(buf + i, FRAME_HEADER, HEADER_LEN) == 0) {
                hdr_pos = i;
                break;
            }
        }

        if (hdr_pos < 0) {
            /* No header yet.  Keep the last 3 bytes in case the header spans
             * two reads (header is 4 bytes; overlap = header_len - 1 = 3). */
            if (buf_len >= HEADER_LEN - 1) {
                int keep = HEADER_LEN - 1;
                memmove(buf, buf + buf_len - keep, (size_t)keep);
                buf_len = keep;
            }
            continue;
        }

        /* Discard any bytes before the header. */
        if (hdr_pos > 0) {
            memmove(buf, buf + hdr_pos, (size_t)(buf_len - hdr_pos));
            buf_len -= hdr_pos;
        }

        /* Do we have the full frame yet? */
        if (buf_len < FRAME_LEN) {
            /* Need more bytes; loop back to read more. */
            continue;
        }

        /* Verify tail. */
        const uint8_t *tail = buf + HEADER_LEN + TARGET_DATA_LEN;
        if (tail[0] != FRAME_TAIL[0] || tail[1] != FRAME_TAIL[1]) {
            /* Bad tail: discard header byte and resync. */
            ESP_LOGD(TAG, "bad tail %02X %02X — resyncing", tail[0], tail[1]);
            memmove(buf, buf + 1, (size_t)(buf_len - 1));
            buf_len--;
            continue;
        }

        /* Parse the three target slots. */
        prop_motion_target_t targets[PROP_MOTION_MAX_TARGETS];
        int count = 0;
        for (int i = 0; i < PROP_MOTION_MAX_TARGETS; i++) {
            prop_motion_target_t t;
            if (parse_target(buf + HEADER_LEN + i * 8, &t)) {
                targets[count++] = t;
            }
        }

        /* Commit to the shared cache. */
        uint32_t ts = now_ms();
        portENTER_CRITICAL(&s_mux);
        s_target_count = count;
        memcpy(s_targets, targets, (size_t)count * sizeof(prop_motion_target_t));
        s_last_seen_ms = ts ? ts : 1;   /* 0 is the "never received" sentinel */
        portEXIT_CRITICAL(&s_mux);

        ESP_LOGD(TAG, "frame ok, %d targets", count);

        /* Consume the frame from the buffer and continue. */
        memmove(buf, buf + FRAME_LEN, (size_t)(buf_len - FRAME_LEN));
        buf_len -= FRAME_LEN;
    }
}

/* ---- Public API ---------------------------------------------------------- */

esp_err_t prop_motion_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = MOTION_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* rx_buffer_size=1024, tx_buffer_size=0 (sensor is read-only in passive mode),
     * event_queue_size=0, no event queue, no interrupt flags. */
    esp_err_t err = uart_driver_install(MOTION_UART, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    esp_err_t pe = uart_param_config(MOTION_UART, &cfg);
    if (pe != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(pe));
        uart_driver_delete(MOTION_UART);
        return pe;
    }

    esp_err_t sp = uart_set_pin(MOTION_UART,
                                MOTION_TX_GPIO, MOTION_RX_GPIO,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (sp != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(sp));
        uart_driver_delete(MOTION_UART);
        return sp;
    }

    /* Task: 4096-byte stack, priority 4, pinned to core 1. */
    BaseType_t r = xTaskCreatePinnedToCore(motion_task, "prop_motion",
                                           4096, NULL, 4, NULL, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        uart_driver_delete(MOTION_UART);
        return ESP_ERR_NO_MEM;
    }

    s_available = true;
    ESP_LOGI(TAG, "PROP_MOTION UART2 ok (GPIO%d/GPIO%d @ %d 8N1)",
             MOTION_TX_GPIO, MOTION_RX_GPIO, MOTION_BAUD);
    return ESP_OK;
}

bool prop_motion_available(void) { return s_available; }

int prop_motion_get_targets(prop_motion_target_t *out, int max)
{
    if (!out || max <= 0) {
        return 0;
    }
    portENTER_CRITICAL(&s_mux);
    int n = s_target_count < max ? s_target_count : max;
    memcpy(out, s_targets, (size_t)n * sizeof(prop_motion_target_t));
    portEXIT_CRITICAL(&s_mux);
    return n;
}

uint32_t prop_motion_ms_since_frame(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t last = s_last_seen_ms;
    portEXIT_CRITICAL(&s_mux);

    if (last == 0) {
        return UINT32_MAX;
    }
    uint32_t elapsed = now_ms() - last;
    return elapsed;
}
