/* prop_motion — HiLink LD2450 24 GHz multi-target mmWave radar on UART2.
 * TX=GPIO53 (P4→LD2450), RX=GPIO54 (LD2450→P4), 256000 8N1.
 * See prop_motion.h for the public API and hardware/protocol notes. */
#include "prop_motion.h"

#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

/* ---- Config-command channel (shared with motion_task) --------------------
 * Separate frame envelope from the data-output frames below: FD FC FB FA ...
 * 04 03 02 01, length-prefixed, with the command word echoed | 0x0100 in the
 * ACK. motion_task is the sole UART reader, so it recognizes both frame
 * types; when a caller is waiting (s_cfg_waiting), a matching ACK is copied
 * into s_cfg_pending and s_cfg_done_sem is given. Byte offsets verified
 * against the protocol PDF's worked examples — see
 * .claude/skills/sensor-datasheets/references/ld2450.md for the command
 * table. ---- */

#define CFG_HEADER_LEN 4
#define CFG_TAIL_LEN   4
#define CFG_MAX_ALEN   32   /* real max is 30 (zone-filter query ACK); anything
                            * bigger looks like corrupt/garbage, not a real frame */
#define CFG_RESP_MAX   26   /* largest real ACK payload after word+status: the
                            * zone-filter query (2B type + 24B of 3 zones) */
static const uint8_t CFG_HEADER[CFG_HEADER_LEN] = { 0xFD, 0xFC, 0xFB, 0xFA };
static const uint8_t CFG_TAIL[CFG_TAIL_LEN]     = { 0x04, 0x03, 0x02, 0x01 };

typedef struct {
    uint16_t word;                  /* command word we're waiting the ACK for */
    uint16_t status;                /* 0 = success, else failure/none-yet */
    uint8_t  resp[CFG_RESP_MAX];    /* ACK payload after word+status, if any */
    uint16_t resp_len;
} ld2450_cfg_pending_t;

static SemaphoreHandle_t     s_cfg_mutex;      /* serializes concurrent callers */
static SemaphoreHandle_t     s_cfg_done_sem;   /* given by motion_task on a matching ACK */
static ld2450_cfg_pending_t  s_cfg_pending;    /* written by motion_task, read by the waiter */
static volatile bool         s_cfg_waiting;    /* true while a caller is blocked in ld2450_cfg_cmd */
static uint32_t              s_current_baud = MOTION_BAUD;   /* tracks the module's UART rate across set_baud */

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
     * frame even if bytes dribble in across multiple reads. Also large enough
     * for the biggest config-command ACK (zone-filter query, 40 bytes total). */
    uint8_t buf[FRAME_LEN * 2];
    int     buf_len = 0;    /* bytes currently held in buf */

    for (;;) {
        /* Drain as many bytes as are available into the tail of buf. */
        int space = (int)sizeof(buf) - buf_len;
        if (space > 0) {
            int n = uart_read_bytes(MOTION_UART, buf + buf_len, (uint32_t)space, pdMS_TO_TICKS(50));
            if (n > 0) buf_len += n;
        }

        /* Find whichever recognized header appears first: the data-frame
         * header (AA FF 03 00) or the config-command ACK header (FD FC FB
         * FA). Config ACKs can in principle arrive between data frames if a
         * command is issued while normal streaming is running. */
        int  hdr_pos = -1;
        bool is_cfg  = false;
        for (int i = 0; i <= buf_len - HEADER_LEN && buf_len >= HEADER_LEN; i++) {
            if (memcmp(buf + i, FRAME_HEADER, HEADER_LEN) == 0) { hdr_pos = i; is_cfg = false; break; }
            if (memcmp(buf + i, CFG_HEADER, CFG_HEADER_LEN) == 0) { hdr_pos = i; is_cfg = true; break; }
        }

        if (hdr_pos < 0) {
            /* No header yet. Keep the last 3 bytes in case a header spans
             * two reads (both headers are 4 bytes; overlap = 3). */
            if (buf_len >= HEADER_LEN - 1) {
                int keep = HEADER_LEN - 1;
                memmove(buf, buf + buf_len - keep, (size_t)keep);
                buf_len = keep;
            }
            continue;
        }

        if (hdr_pos > 0) {
            memmove(buf, buf + hdr_pos, (size_t)(buf_len - hdr_pos));
            buf_len -= hdr_pos;
        }

        if (is_cfg) {
            if (buf_len < CFG_HEADER_LEN + 2) continue;   /* need the length field yet */
            uint16_t alen = (uint16_t)buf[CFG_HEADER_LEN] | ((uint16_t)buf[CFG_HEADER_LEN + 1] << 8);
            if (alen > CFG_MAX_ALEN) {
                /* Bogus length -- resync past this header byte. */
                memmove(buf, buf + 1, (size_t)(buf_len - 1)); buf_len--; continue;
            }
            int total = CFG_HEADER_LEN + 2 + alen + CFG_TAIL_LEN;
            if (buf_len < total) continue;   /* frame incomplete; need more bytes */
            if (memcmp(buf + total - CFG_TAIL_LEN, CFG_TAIL, CFG_TAIL_LEN) != 0) {
                memmove(buf, buf + 1, (size_t)(buf_len - 1)); buf_len--; continue;
            }

            uint16_t ack_word = (uint16_t)buf[CFG_HEADER_LEN + 2] | ((uint16_t)buf[CFG_HEADER_LEN + 3] << 8);
            if (s_cfg_waiting && ack_word == (uint16_t)(s_cfg_pending.word | 0x0100)) {
                const uint8_t *dptr = buf + CFG_HEADER_LEN + 2 + 2;   /* just past the echoed word */
                uint16_t dlen = (uint16_t)(alen - 2);
                s_cfg_pending.status = (dlen >= 2) ? ((uint16_t)dptr[0] | ((uint16_t)dptr[1] << 8)) : 0xFFFF;
                uint16_t extra = (dlen > 2) ? (uint16_t)(dlen - 2) : 0;
                if (extra > sizeof(s_cfg_pending.resp)) extra = sizeof(s_cfg_pending.resp);
                memcpy(s_cfg_pending.resp, dptr + 2, extra);
                s_cfg_pending.resp_len = extra;
                xSemaphoreGive(s_cfg_done_sem);
            }
            /* else: unmatched/stray ACK -- ignore, no one is waiting for it. */

            memmove(buf, buf + total, (size_t)(buf_len - total));
            buf_len -= total;
            continue;
        }

        /* ---- data-frame path ---- */
        if (buf_len < FRAME_LEN) continue;

        const uint8_t *tail = buf + HEADER_LEN + TARGET_DATA_LEN;
        if (tail[0] != FRAME_TAIL[0] || tail[1] != FRAME_TAIL[1]) {
            ESP_LOGD(TAG, "bad tail %02X %02X — resyncing", tail[0], tail[1]);
            memmove(buf, buf + 1, (size_t)(buf_len - 1));
            buf_len--;
            continue;
        }

        prop_motion_target_t targets[PROP_MOTION_MAX_TARGETS];
        int count = 0;
        for (int i = 0; i < PROP_MOTION_MAX_TARGETS; i++) {
            prop_motion_target_t t;
            if (parse_target(buf + HEADER_LEN + i * 8, &t)) {
                targets[count++] = t;
            }
        }

        uint32_t ts = now_ms();
        portENTER_CRITICAL(&s_mux);
        s_target_count = count;
        memcpy(s_targets, targets, (size_t)count * sizeof(prop_motion_target_t));
        s_last_seen_ms = ts ? ts : 1;   /* 0 is the "never received" sentinel */
        portEXIT_CRITICAL(&s_mux);

        ESP_LOGD(TAG, "frame ok, %d targets", count);

        memmove(buf, buf + FRAME_LEN, (size_t)(buf_len - FRAME_LEN));
        buf_len -= FRAME_LEN;
    }
}

/* ---- Config-command exchange --------------------------------------------- */

/* Send one config-command frame and block (up to timeout_ms) for motion_task
 * to hand back its ACK. Serializes concurrent callers on s_cfg_mutex -- only
 * one command exchange is ever in flight. Returns true iff the module ACK'd
 * with status == 0 (success); resp_out/resp_len_out (if non-NULL) receive any
 * payload the ACK carried beyond the status word. */
static bool ld2450_cfg_cmd(uint16_t word, const uint8_t *val, uint16_t val_len,
                           uint8_t *resp_out, uint16_t resp_out_max, uint16_t *resp_len_out,
                           int timeout_ms)
{
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);

    uint8_t frame[40];   /* largest real command is zone-set: 4+2+2+26+4 = 38 bytes */
    uint16_t plen = (uint16_t)(2 + val_len);   /* command word + value */
    int n = 0;
    memcpy(frame + n, CFG_HEADER, CFG_HEADER_LEN); n += CFG_HEADER_LEN;
    frame[n++] = (uint8_t)(plen & 0xFF);
    frame[n++] = (uint8_t)(plen >> 8);
    frame[n++] = (uint8_t)(word & 0xFF);
    frame[n++] = (uint8_t)(word >> 8);
    if (val_len) { memcpy(frame + n, val, val_len); n += val_len; }
    memcpy(frame + n, CFG_TAIL, CFG_TAIL_LEN); n += CFG_TAIL_LEN;

    s_cfg_pending.word     = word;
    s_cfg_pending.status   = 0xFFFF;
    s_cfg_pending.resp_len = 0;
    xSemaphoreTake(s_cfg_done_sem, 0);   /* drain any stale signal */
    s_cfg_waiting = true;

    uart_write_bytes(MOTION_UART, frame, n);
    bool got_ack = (xSemaphoreTake(s_cfg_done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
    s_cfg_waiting = false;

    bool ok = got_ack && s_cfg_pending.status == 0x0000;
    if (ok && resp_out && resp_len_out) {
        uint16_t nc = s_cfg_pending.resp_len < resp_out_max ? s_cfg_pending.resp_len : resp_out_max;
        memcpy(resp_out, s_cfg_pending.resp, nc);
        *resp_len_out = nc;
    } else if (resp_len_out) {
        *resp_len_out = 0;
    }

    xSemaphoreGive(s_cfg_mutex);
    return ok;
}

bool prop_motion_cfg_get_mode(prop_motion_track_mode_t *out)
{
    if (!out) return false;
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t resp[4]; uint16_t resp_len = 0;
    bool ok = ld2450_cfg_cmd(0x0091, NULL, 0, resp, sizeof(resp), &resp_len, 300) && resp_len >= 2;
    if (ok) *out = (prop_motion_track_mode_t)((uint16_t)resp[0] | ((uint16_t)resp[1] << 8));
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    return ok;
}

bool prop_motion_cfg_set_mode(prop_motion_track_mode_t mode)
{
    if (mode != PROP_MOTION_TRACK_SINGLE && mode != PROP_MOTION_TRACK_MULTI) return false;
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint16_t word = (mode == PROP_MOTION_TRACK_SINGLE) ? 0x0080 : 0x0090;
    bool ok = ld2450_cfg_cmd(word, NULL, 0, NULL, 0, NULL, 300);
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    return ok;
}

bool prop_motion_cfg_get_fw_version(char *out, size_t out_len)
{
    if (!out || out_len == 0) return false;
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t resp[10]; uint16_t resp_len = 0;   /* fw_type(2) + major(2) + minor(4) = 8 */
    bool ok = ld2450_cfg_cmd(0x00A0, NULL, 0, resp, sizeof(resp), &resp_len, 300) && resp_len >= 8;
    if (ok) {
        /* Best-effort pretty-print inferred from the PDF's one worked example
         * (major/minor bytes reversed, printed as hex-digit pairs: raw
         * [02,01,16,24,06,22] -> "V1.02.22062416"). Trust the raw bytes over
         * this string if it ever looks wrong on a real module. */
        snprintf(out, out_len, "V%u.%02X.%02X%02X%02X%02X",
                 resp[3], resp[2], resp[7], resp[6], resp[5], resp[4]);
    }
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    return ok;
}

bool prop_motion_cfg_get_mac(uint8_t mac_out[6])
{
    if (!mac_out) return false;
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t resp[8]; uint16_t resp_len = 0;
    /* The protocol PDF (V1.03) documents a type byte + 3-byte MAC, but real
     * fw V2.04 hardware returns a plain 6-byte MAC, big-endian, no prefix
     * (verified on the bench: raw ACK payload len=6). With Bluetooth off the
     * module reports the placeholder 08:05:04:03:02:01 instead of its real
     * MAC — same behavior ESPHome documents for the LD2410 sibling. */
    bool ok = ld2450_cfg_cmd(0x00A5, (const uint8_t[]){0x01, 0x00}, 2, resp, sizeof(resp), &resp_len, 300)
              && resp_len >= 6;
    if (ok) memcpy(mac_out, resp, 6);
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    return ok;
}

bool prop_motion_cfg_set_bt(bool on)
{
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t val[2] = { 0x00, (uint8_t)(on ? 0x01 : 0x00) };   /* LE: on=0x0100, off=0x0000 */
    bool ok = ld2450_cfg_cmd(0x00A4, val, 2, NULL, 0, NULL, 300);
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    if (ok) ESP_LOGI(TAG, "LD2450 bluetooth %s", on ? "enabled" : "disabled");
    return ok;
}

bool prop_motion_cfg_get_zone(prop_motion_zone_mode_t *mode, prop_motion_zone_t zones[3])
{
    if (!mode || !zones) return false;
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t resp[CFG_RESP_MAX]; uint16_t resp_len = 0;
    /* resp layout: type(2) + 3 zones x 4 int16 (x1,y1,x2,y2), plain LE two's
     * complement -- NOT sign-magnitude, unlike the real-time data frames. */
    bool ok = ld2450_cfg_cmd(0x00C1, NULL, 0, resp, sizeof(resp), &resp_len, 300) && resp_len >= 26;
    if (ok) {
        *mode = (prop_motion_zone_mode_t)((uint16_t)resp[0] | ((uint16_t)resp[1] << 8));
        for (int i = 0; i < 3; i++) {
            const uint8_t *z = resp + 2 + i * 8;
            zones[i].x1_mm = (int16_t)((uint16_t)z[0] | ((uint16_t)z[1] << 8));
            zones[i].y1_mm = (int16_t)((uint16_t)z[2] | ((uint16_t)z[3] << 8));
            zones[i].x2_mm = (int16_t)((uint16_t)z[4] | ((uint16_t)z[5] << 8));
            zones[i].y2_mm = (int16_t)((uint16_t)z[6] | ((uint16_t)z[7] << 8));
        }
    }
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    return ok;
}

bool prop_motion_cfg_set_zone(prop_motion_zone_mode_t mode, const prop_motion_zone_t zones[3])
{
    if (!zones || (mode != PROP_MOTION_ZONE_OFF && mode != PROP_MOTION_ZONE_INCLUDE &&
                   mode != PROP_MOTION_ZONE_EXCLUDE)) return false;
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t val[26];
    val[0] = (uint8_t)mode; val[1] = 0x00;
    for (int i = 0; i < 3; i++) {
        uint8_t *z = val + 2 + i * 8;
        z[0] = (uint8_t)(zones[i].x1_mm & 0xFF); z[1] = (uint8_t)((uint16_t)zones[i].x1_mm >> 8);
        z[2] = (uint8_t)(zones[i].y1_mm & 0xFF); z[3] = (uint8_t)((uint16_t)zones[i].y1_mm >> 8);
        z[4] = (uint8_t)(zones[i].x2_mm & 0xFF); z[5] = (uint8_t)((uint16_t)zones[i].x2_mm >> 8);
        z[6] = (uint8_t)(zones[i].y2_mm & 0xFF); z[7] = (uint8_t)((uint16_t)zones[i].y2_mm >> 8);
    }
    bool ok = ld2450_cfg_cmd(0x00C2, val, sizeof(val), NULL, 0, NULL, 300);
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    return ok;
}

/* How long to wait for data streaming to resume after a restart, at whichever
 * baud we expect the module to come back at. Shared with the baud-change path. */
#define LD2450_RESYNC_TIMEOUT_MS 5000

/* Poll prop_motion_ms_since_frame() until a FRESH data frame lands. motion_task
 * parses it on its own -- no separate read path needed here. Used after a
 * restart (and after a baud change) to confirm the module is back. */
static bool ld2450_wait_for_data_frame(int timeout_ms)
{
    int64_t start    = esp_timer_get_time();
    int64_t deadline = start + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(100));
        /* Only count a frame that landed AFTER this wait began — a frame
         * received just before the restart/baud switch is still "recent" by
         * age but proves nothing about the module having come back. */
        uint32_t since   = prop_motion_ms_since_frame();
        uint32_t elapsed = (uint32_t)((esp_timer_get_time() - start) / 1000);
        if (since < elapsed) return true;
    }
    return false;
}

/* Baud index table (protocol PDF Table 6). Returns 0 if bps isn't one of the
 * module's 8 supported rates. */
static uint16_t ld2450_baud_index(uint32_t bps)
{
    switch (bps) {
        case 9600:   return 0x0001;
        case 19200:  return 0x0002;
        case 38400:  return 0x0003;
        case 57600:  return 0x0004;
        case 115200: return 0x0005;
        case 230400: return 0x0006;
        case 256000: return 0x0007;
        case 460800: return 0x0008;
        default:     return 0;
    }
}

/* Sends "restart module" (inside its own Enable-Config bracket), then follows
 * the module through its reboot at expect_baud: switches the LOCAL UART to
 * that rate and waits for data frames to resume. Falls back to the PREVIOUS
 * baud rate if nothing arrives in time -- we know exactly what we asked for,
 * so a clean two-way fallback is possible. If the module is silent at BOTH
 * rates, there is no further automatic recovery: it needs physical attention
 * (power-cycle, or check with a bench USB-serial adapter). */
static bool ld2450_restart_and_resync(uint32_t expect_baud)
{
    uint32_t prev_baud = s_current_baud;

    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) {
        ESP_LOGW(TAG, "LD2450 enable-config (for restart) failed — leaving baud unchanged");
        return false;
    }
    if (!ld2450_cfg_cmd(0x00A3, NULL, 0, NULL, 0, NULL, 300)) {
        ESP_LOGW(TAG, "LD2450 restart command not ACK'd — leaving baud unchanged");
        return false;
    }

    uart_set_baudrate(MOTION_UART, expect_baud);
    s_current_baud = expect_baud;
    uart_flush_input(MOTION_UART);

    if (ld2450_wait_for_data_frame(LD2450_RESYNC_TIMEOUT_MS)) {
        ESP_LOGI(TAG, "LD2450 resumed streaming at %lu baud", (unsigned long)expect_baud);
        return true;
    }

    ESP_LOGW(TAG, "LD2450 silent at %lu baud after restart — falling back to %lu",
             (unsigned long)expect_baud, (unsigned long)prev_baud);
    uart_set_baudrate(MOTION_UART, prev_baud);
    s_current_baud = prev_baud;
    uart_flush_input(MOTION_UART);

    if (ld2450_wait_for_data_frame(LD2450_RESYNC_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "LD2450 baud change did not take effect; module still at %lu — "
                      "config was likely rejected or the restart failed silently",
                 (unsigned long)prev_baud);
    } else {
        ESP_LOGE(TAG, "LD2450 silent at both %lu and %lu baud — module needs physical "
                      "attention (power-cycle, or check with a bench USB-serial adapter)",
                 (unsigned long)expect_baud, (unsigned long)prev_baud);
    }
    return false;
}

bool prop_motion_cfg_set_baud(uint32_t new_baud_bps)
{
    uint16_t idx = ld2450_baud_index(new_baud_bps);
    if (idx == 0) {
        ESP_LOGW(TAG, "LD2450: %lu is not one of the 8 supported baud rates", (unsigned long)new_baud_bps);
        return false;
    }
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t val[2] = { (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8) };
    bool set_ok = ld2450_cfg_cmd(0x00A1, val, 2, NULL, 0, NULL, 300);
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    if (!set_ok) {
        ESP_LOGW(TAG, "LD2450 set-baud command not ACK'd — module unchanged");
        return false;
    }
    return ld2450_restart_and_resync(new_baud_bps);
}

bool prop_motion_cfg_factory_reset(void)
{
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    bool ok = ld2450_cfg_cmd(0x00A2, NULL, 0, NULL, 0, NULL, 300);
    ld2450_cfg_cmd(0x00FE, NULL, 0, NULL, 0, NULL, 300);
    if (!ok) return false;
    ESP_LOGI(TAG, "LD2450 factory-reset accepted; restarting to apply "
                  "(baud reverts to 256000, BT to on, tracking to multi, zone filter off)");
    return ld2450_restart_and_resync(256000);
}

bool prop_motion_cfg_restart(void)
{
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    if (!ld2450_cfg_cmd(0x00A3, NULL, 0, NULL, 0, NULL, 300)) return false;
    ESP_LOGI(TAG, "LD2450 restart requested; waiting for the module to come back up");
    bool back = ld2450_wait_for_data_frame(LD2450_RESYNC_TIMEOUT_MS);
    if (!back) ESP_LOGE(TAG, "LD2450 did not resume streaming after restart");
    return back;
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

    /* rx_buffer_size=1024, tx_buffer_size=0 (config commands are tiny; with no
     * TX ring uart_write_bytes sends synchronously, which is fine here),
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

    s_cfg_mutex    = xSemaphoreCreateMutex();
    s_cfg_done_sem = xSemaphoreCreateBinary();
    s_current_baud = MOTION_BAUD;

    /* Task: 4096-byte stack, priority 4, pinned to core 0 (core 1 = LVGL). */
    BaseType_t r = xTaskCreatePinnedToCore(motion_task, "prop_motion",
                                           4096, NULL, 4, NULL, 0);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        uart_driver_delete(MOTION_UART);
        return ESP_ERR_NO_MEM;
    }

    s_available = true;
    ESP_LOGI(TAG, "PROP_MOTION UART2 ok (GPIO%d/GPIO%d @ %d 8N1)",
             MOTION_TX_GPIO, MOTION_RX_GPIO, MOTION_BAUD);

    /* Best-effort: disable the module's onboard Bluetooth (on by default;
     * unused by this project, and removing an always-on 24GHz-adjacent BT
     * radio rules it out as an RF-interference variable for the C6's own
     * BLE/CSI work). motion_task is already running and routes this
     * command's ACK back to us. */
    if (!prop_motion_cfg_set_bt(false)) {
        ESP_LOGW(TAG, "LD2450 bluetooth-off at boot failed/timed out — leaving module default");
    }

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
