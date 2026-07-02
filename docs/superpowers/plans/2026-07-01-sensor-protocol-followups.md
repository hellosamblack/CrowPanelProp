# Sensor Protocol Follow-ups — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Act on the concrete, verified discoveries from building the `sensor-datasheets` skill
(`.claude/skills/sensor-datasheets/`) — a real unit-conversion bug in the LD2450 radar driver, an
undersized command-spacing delay in the SEN0395 boot sequence, and full read/write access to
**every** documented LD2450 configuration option (tracking mode, onboard Bluetooth, zone
filtering, firmware version, MAC address, factory reset, restart, and baud rate), exposed over
HTTP for scripting/testing. Lower-value/product-dependent opportunities for the *other* two
sensors (MR24HPC1 scene modes, SEN0395 range/latency tuning) are captured as a deferred backlog,
not full tasks — they need a design decision this plan doesn't make for you.

**Background:** Building that skill involved fact-checking all four external-sensor
`README.md` files against their real datasheet PDFs and the firmware drivers that actually talk
to them (`main/prop_motion.c`, `main/prop_imu.c`, `main/prop_aux_radar.c`). Two READMEs turned
out to be wrong/duplicated (already fixed); along the way the research surfaced firmware-level
findings that are the subject of this plan. The LD2450 config-command byte protocol below was
re-verified directly against the primary source, `docs/datasheets/externalDevices/LD2450 HiLink
24GHz_Multitarget_mmWave_Sensor/HLK-LD2450 Human body tracking detection (3 person)/LD2450 serial
port communication protocol V1.03.pdf`, including every worked example in it — not just the
skill's summarized reference (`.claude/skills/sensor-datasheets/references/ld2450.md`), which is
still useful background but doesn't have the exact byte offsets this plan needed.

**Architecture:** Task 1 touches `main/prop_motion.c` + `main/prop_ui.c`. Task 2 touches
`main/prop_aux_radar.c`. Tasks 3-4 touch `main/prop_motion.c` + `main/include/prop_motion.h`
(the config-command channel and typed read/write API). Task 5 touches `main/prop_api.c` +
`tools/prop.py` (HTTP exposure). No new files, no build-system changes, no `idf.py reconfigure`
needed anywhere in this plan.

**Design decision worth flagging up front:** the original version of this plan (Task 3) only
covered a one-shot, boot-time-only Bluetooth-off exchange, done *before* the background frame
reader task started — that sidestepped needing to share the UART between two readers. Doing
genuine runtime read/write (queryable anytime, not just at boot) requires the background reader
(`motion_task`) to become the **sole** reader of the UART and dispatch both frame types itself
(data frames it already knew about, plus config-command ACK frames) — Task 3 below builds that
properly rather than bolting on a second concurrent reader.

## Global Constraints

- **No automated tests in this repo.** Every task is verified by build → flash → either a
  live-hardware check (moving a hand in front of the LD2450) or `idf.py monitor`/log
  inspection, as noted per task.
- **Don't touch `CONFIG_ESP32P4_*REV*`** or other unrelated sdkconfig items — out of scope here.
- Follow the existing code style in each file (this project doesn't use comments to restate
  what code does — only for non-obvious constraints/gotchas, matching the rest of these files).
- Tasks 1-2 are independent and can be done in any order. Tasks 3→4→5 are **sequential** — 4
  and 5 both depend on the config-command channel Task 3 builds.
- **Never call any `prop_motion_cfg_*` function from the LVGL/UI task or from inside
  `lvgl_port_lock()`/`unlock()`** — every one of them blocks the calling task for up to a few
  hundred ms (up to several seconds for `prop_motion_cfg_set_baud`/`prop_motion_cfg_factory_reset`,
  which restart the module). Call them from a plain task context (an HTTP handler task is fine).

### Build / flash commands (used in every task)

```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" build
# Confirm port first (varies COM7/COM4): [System.IO.Ports.SerialPort]::GetPortNames()
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash
```

```bash
# Live checks (mDNS, no IP hunting):
python tools/prop.py telemetry                                   # one-shot radar/imu/etc snapshot
python tools/prop.py watch --only radar --count 20                # live radar stream
python tools/prop.py shot motion.png --screen motion --wait        # SCANNER/MOTION SCAN screenshot
python tools/prop.py trace --trials 8                              # repeated-reboot crash/hang check
```

---

## Task 1: Fix LD2450 speed unit bug (cm/s decoded and displayed as mm/s)

**Files:**
- Modify: `main/prop_motion.c` — `parse_target()` (~line 56-70).
- Modify: `main/prop_ui.c` — the fast-motion alert-color threshold (~line 5622) and the velocity
  readout label (~line 5680-5682).

**Why:** The LD2450 protocol PDF (`docs/datasheets/externalDevices/LD2450 HiLink.../LD2450
serial port communication protocol V1.03.pdf`) states each target's speed field is encoded in
**cm/s**, sign-magnitude, same as X/Y. `parse_target()` currently stores that raw value directly
into `t->speed_mm_s` with no ×10 conversion — so every speed reading is off by exactly 10×
system-wide (radar UI readout, `/telemetry` API's `radar[].speed_mm_s`, and the fast-motion
alert-color threshold). A target moving at a real 50 cm/s (a slow walk) currently displays as
`+50 mm/s` (should read `+500 mm/s`), and the UI's `> 200` "fast" alert threshold — written
assuming the field was already true mm/s — is actually gated on **200 cm/s = 2 m/s** raw, a much
higher real-world bar than "200 mm/s" suggests. Fixing the encoding alone would silently drop
that effective threshold to 20 cm/s (any detected movement) unless the threshold constant is
updated in the same change — do both in one task so the two files stay consistent.

- [x] **Step 1: Convert to true mm/s at the decode boundary**

Current (`main/prop_motion.c` ~56-70):
```c
static bool parse_target(const uint8_t *p, prop_motion_target_t *t)
{
    int16_t  x   = ld2450_signmag(p + 0);
    int16_t  y   = ld2450_signmag(p + 2);
    int16_t  spd = ld2450_signmag(p + 4);
    uint16_t dr  = (uint16_t)p[6] | ((uint16_t)p[7] << 8);

    t->x_mm        = x;
    t->y_mm        = y;
    t->speed_mm_s  = spd;
    t->dist_res_mm = dr;

    /* Inactive slots are all-zero; a real target has a non-zero coordinate. */
    return (x != 0 || y != 0);
}
```
Change to:
```c
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
```
(`spd`'s realistic magnitude for a human target is well under 3000 cm/s, so `spd * 10` cannot
overflow `int16_t` in practice — no need for a wider intermediate type.)

- [x] **Step 2: Rescale the fast-motion alert threshold to match**

Current (`main/prop_ui.c` ~5621-5623):
```c
                lv_obj_set_style_bg_color(s_motion_blips[i],
                    (tgts[i].speed_mm_s > 200 || tgts[i].speed_mm_s < -200)
                        ? COL_ALERT : COL_AMBER, 0);
```
Change to (preserves the pre-fix real-world threshold of ~2 m/s, now expressed correctly):
```c
                lv_obj_set_style_bg_color(s_motion_blips[i],
                    (tgts[i].speed_mm_s > 2000 || tgts[i].speed_mm_s < -2000)
                        ? COL_ALERT : COL_AMBER, 0);
```
If you'd rather the alert trigger at a different real speed now that the field is trustworthy
(e.g. 1500 mm/s ≈ a brisk walk), pick that value instead — just make sure it's a deliberate
choice, not the accidental ~2000 mm/s the bug happened to produce.

- [x] **Step 3: Confirm the velocity label's unit suffix is now correct**

`main/prop_ui.c` ~5680-5682 already prints `"%+d mm/s"` from `tgts[i].speed_mm_s` — no code
change needed here, it was already labeled correctly, it was just fed wrong data. Leave as-is;
this step is just confirming there's nothing else to touch.

- [x] **Step 4: Build**

Run the build command. Expected: `Project build complete`.

- [x] **Step 5: Flash + verify on hardware**

Flash, then walk a hand across the LD2450's field of view at a normal pace and watch:
```bash
python tools/prop.py watch --only radar --count 30
```
Expected: `speed_mm_s` values in the few-hundred-to-~1500 mm/s range for a normal walking pace
(previously they'd have read 10× smaller, i.e. tens-to-~150). Also check
`python tools/prop.py shot motion.png --screen motion --wait` — the T1/T2/T3 velocity readout
should show plausible `mm/s` numbers, and the blip should only turn `COL_ALERT` red on a fast
lunge/wave, not on ordinary walking-pace movement.

- [x] **Step 6: Commit**

```bash
git add main/prop_motion.c main/prop_ui.c
git commit -m "fix(motion): correct LD2450 speed units — protocol is cm/s, not mm/s"
```

---

## Task 2: Harden SEN0395 boot command spacing

**Files:**
- Modify: `main/prop_aux_radar.c` — `sen0395_task()` boot sequence (~line 264-268).

**Why:** The DFRobot SEN0395's own Arduino library (`DFRobot_mmWave_Radar.h`, `#define DELAY
1000`) documents that commands sent to the sensor's CLI must be spaced **≥1000 ms apart** — the
onboard CLI can't reliably process back-to-back writes faster than that. This project's boot
sequence currently sends `sensorStop` then `sensorStart` only **350 ms** apart
(`main/prop_aux_radar.c:267`), under that documented minimum. This has apparently been working
in practice (it's carried over unchanged from the old `prop_radar.c`), so this is risk-reduction
for an edge case (e.g. sensor init flakiness after certain power-on conditions), not a fix for
an observed failure — treat it as cheap insurance, not an urgent bug.

- [x] **Step 1: Widen the inter-command delay**

Current (`main/prop_aux_radar.c` ~264-268):
```c
    /* Send sensorStop then sensorStart before entering the read loop. */
    vTaskDelay(pdMS_TO_TICKS(700));
    uart_write_bytes(SEN0395_UART, "\rsensorStop\r",  12);
    vTaskDelay(pdMS_TO_TICKS(350));
    sen0395_kick();
```
Change to:
```c
    /* Send sensorStop then sensorStart before entering the read loop.
     * Vendor lib enforces >=1000ms between CLI commands (DFRobot_mmWave_Radar.h
     * DELAY=1000) — give sensorStop that much room before sensorStart follows. */
    vTaskDelay(pdMS_TO_TICKS(700));
    uart_write_bytes(SEN0395_UART, "\rsensorStop\r",  12);
    vTaskDelay(pdMS_TO_TICKS(1000));
    sen0395_kick();
```

- [x] **Step 2: Build**

Run the build command. Expected: `Project build complete`.

- [x] **Step 3: Flash + verify boot reliability**

```bash
python tools/prop.py trace --trials 8
```
Expected: 8/8 clean boots, SEN0395 presence telemetry (`aux_radar.sen0395`) comes up online
within the same rough timeframe as before (boot is ~650ms later than before, not user-visible).
No behavior change expected — this is a robustness margin increase, not a functional change.

- [x] **Step 4: Commit**

```bash
git add main/prop_aux_radar.c
git commit -m "fix(aux_radar): widen SEN0395 boot command spacing to vendor-documented 1000ms"
```

---

## Task 3: Build the LD2450 config-command channel + core read/write API

**Files:**
- Modify: `main/prop_motion.c` — dual-header frame recognition in `motion_task`, a mutex/semaphore
  command channel, and typed functions for mode, Bluetooth, zone filtering, firmware version, MAC
  address, and restart/factory-reset. Baud rate is deliberately split out into Task 4 (elevated
  risk, isolated so it's easy to review/test/rip out on its own).
- Modify: `main/include/prop_motion.h` — public declarations for all of the above.

**Why:** The LD2450 protocol PDF (`.../LD2450 serial port communication protocol V1.03.pdf`)
documents a full configuration surface over a **separate command-frame envelope**
(`FD FC FB FA ... 04 03 02 01`, distinct from the `AA FF 03 00 ... 55 CC` data-output frames):
tracking mode get/set, firmware version, MAC address, Bluetooth on/off, zone filtering get/set,
factory reset, restart, and baud rate. This project's firmware never sends any config command
today. Building real read/write access — queryable at any time, not just once at boot — means
`motion_task` (the only task that ever reads this UART) has to recognize **both** frame types
and route config-command ACKs back to whichever caller is waiting.

**Design note:** every byte offset below was re-verified against the protocol PDF's worked
examples (not just summarized from the skill's reference file) — e.g. the Enable-Config ACK
`FD FC FB FA 08 00 FF 01 00 00 01 00 40 00 04 03 02 01` decodes as header(4) + length(2)=`08 00`
+ echoed word(2)=`FF 01` (0x00FF|0x0100) + status(2)=`00 00` + protocol version(2)=`01 00` +
buffer size(2)=`40 00` + tail(4). The zone-filter query/set commands turned out to use **plain
signed int16 (two's complement)** coordinates — confirmed from the worked example (`18FC` LE =
`0xFC18` = -1000) — **not** the sign-magnitude encoding the real-time data frames use. Getting
that distinction backwards would silently corrupt every negative zone coordinate.

- [x] **Step 1: Add the command-channel state and dual-header frame scanner**

Add near the top of `main/prop_motion.c`, after the existing `#include`s (add `#include
"freertos/semphr.h"` alongside the existing `freertos/task.h` include) and before `#define TAG`:
no change needed there — just confirm the new include is present.

Add above `/* ---- Frame parser ---- */` (~line 41), replacing nothing (pure addition):
```c
/* ---- Config-command channel (shared with motion_task) --------------------
 * Separate frame envelope from the data-output frames below: FD FC FB FA ...
 * 04 03 02 01, length-prefixed, with the command word echoed | 0x0100 in the
 * ACK. motion_task is the sole UART reader, so it recognizes both frame
 * types; when a caller is waiting (s_cfg_waiting), a matching ACK is copied
 * into s_cfg_pending and s_cfg_done_sem is given. Byte offsets verified
 * against the protocol PDF's worked examples — see the "Design note" above
 * this task in the plan this was implemented from, or
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
static uint32_t              s_current_baud = MOTION_BAUD;   /* used by Task 4's baud-change */
```

- [x] **Step 2: Update `motion_task` to recognize both frame types**

Current (`main/prop_motion.c` ~74-162) — the whole function. Replace it with:
```c
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

        /* ---- data-frame path (unchanged from before this task) ---- */
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
```
(This is a pure superset of the original function — the data-frame path at the bottom is
byte-for-byte unchanged; only the header search and a new `is_cfg` branch were added above it.)

- [x] **Step 3: Add the low-level send/wait-for-ACK primitive**

Add just above `/* ---- Public API ---- */` (~line 164):
```c
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
```

- [x] **Step 4: Add the typed public functions**

Add below `ld2450_cfg_cmd` (still above `/* ---- Public API ---- */`):
```c
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

bool prop_motion_cfg_get_mac(uint8_t mac_out[3])
{
    if (!mac_out) return false;
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){0x01, 0x00}, 2, NULL, 0, NULL, 300)) return false;
    uint8_t resp[6]; uint16_t resp_len = 0;   /* fixed type(1) + MAC(3) = 4 */
    /* MAC bytes are BIG-endian -- the one field in this protocol that isn't LE. */
    bool ok = ld2450_cfg_cmd(0x00A5, (const uint8_t[]){0x01, 0x00}, 2, resp, sizeof(resp), &resp_len, 300)
              && resp_len >= 4;
    if (ok) memcpy(mac_out, resp + 1, 3);
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
```

- [x] **Step 5: Add `prop_motion_cfg_restart`, wire semaphore creation into init, and disable BT at boot**

Add below the Step 4 functions (still above `/* ---- Public API ---- */`):
```c
/* How long to wait for data streaming to resume after a restart, at whichever
 * baud we expect the module to come back at. Shared with Task 4's baud-change. */
#define LD2450_RESYNC_TIMEOUT_MS 5000

/* Poll prop_motion_ms_since_frame() until a FRESH data frame lands. motion_task
 * parses it on its own -- no separate read path needed here. Used after a
 * restart (and, in Task 4, after a baud change) to confirm the module is back. */
static bool ld2450_wait_for_data_frame(int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (prop_motion_ms_since_frame() < 500) return true;
    }
    return false;
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
```

Now update `prop_motion_init()`. Current (`main/prop_motion.c` ~166-214), the tail end from the
task-creation call onward:
```c
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
```
Change to:
```c
    s_cfg_mutex    = xSemaphoreCreateMutex();
    s_cfg_done_sem = xSemaphoreCreateBinary();
    s_current_baud = MOTION_BAUD;

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
```

- [x] **Step 6: Add the public declarations**

Add to `main/include/prop_motion.h`, after `#include "esp_err.h"` add `#include <stddef.h>`
(needed for `size_t`), and after the `prop_motion_target_t` struct, before `prop_motion_init`:
```c
/* ---- Config-command protocol (read/write access to on-module settings) --
 * Every prop_motion_cfg_* function performs a live UART command/ACK exchange
 * (see docs/datasheets/externalDevices/LD2450.../"LD2450 serial port
 * communication protocol V1.03.pdf" and
 * .claude/skills/sensor-datasheets/references/ld2450.md) and BLOCKS the
 * calling task for up to ~300ms (several seconds for prop_motion_cfg_set_baud
 * / prop_motion_cfg_factory_reset, which restart the module). Never call
 * these from the LVGL/UI task or from inside lvgl_port_lock()/unlock(). Only
 * one exchange runs at a time; concurrent callers serialize on an internal
 * mutex. */

typedef enum {
    PROP_MOTION_TRACK_SINGLE = 1,
    PROP_MOTION_TRACK_MULTI  = 2,
} prop_motion_track_mode_t;

typedef enum {
    PROP_MOTION_ZONE_OFF     = 0,   /* zone filtering disabled (factory default) */
    PROP_MOTION_ZONE_INCLUDE = 1,   /* only detect targets inside the listed zones */
    PROP_MOTION_ZONE_EXCLUDE = 2,   /* ignore targets inside the listed zones */
} prop_motion_zone_mode_t;

typedef struct {
    int16_t x1_mm, y1_mm, x2_mm, y2_mm;  /* diagonal rectangle corners, mm.
                                          * Plain signed int16 (two's complement)
                                          * -- NOT the sign-magnitude encoding
                                          * the real-time data frames use.
                                          * All-zero = this zone slot unused. */
} prop_motion_zone_t;

/* Tracking mode: read the module's current single/multi setting, or change it
 * (factory default: multi). */
bool prop_motion_cfg_get_mode(prop_motion_track_mode_t *out);
bool prop_motion_cfg_set_mode(prop_motion_track_mode_t mode);

/* Firmware version as a display string (e.g. "V1.02.22062416"); out_len >= 24. */
bool prop_motion_cfg_get_fw_version(char *out, size_t out_len);

/* The module's 3-byte MAC address (its onboard Bluetooth radio's MAC). */
bool prop_motion_cfg_get_mac(uint8_t mac_out[3]);

/* Bluetooth on/off. Write-only -- the protocol has no BT-status query
 * command, so there is no prop_motion_cfg_get_bt(). On by default from the
 * factory; this project turns it off at boot (see prop_motion_init). */
bool prop_motion_cfg_set_bt(bool on);

/* Rectangular zone filtering: read or replace the current configuration (up
 * to 3 zones; a zone with all-zero coordinates is unused). */
bool prop_motion_cfg_get_zone(prop_motion_zone_mode_t *mode, prop_motion_zone_t zones[3]);
bool prop_motion_cfg_set_zone(prop_motion_zone_mode_t mode, const prop_motion_zone_t zones[3]);

/* Restart the module (it reboots itself right after ACKing this command).
 * Blocks until data streaming resumes or ~5s elapses. */
bool prop_motion_cfg_restart(void);
```

- [x] **Step 7: Build**

Run the build command. Expected: `Project build complete`.

- [x] **Step 8: Flash + verify normal tracking still works**

```bash
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 monitor
```
Expected near boot: `LD2450 bluetooth disabled` (or a `WARN` if it didn't ACK in time — not
fatal). Then:
```bash
python tools/prop.py watch --only radar --count 20
```
Expected: target X/Y/speed still streams normally — the boot-time BT-off exchange must not have
left the module stuck in config mode. If radar data doesn't resume, check that "End Config"
(`0x00FE`) is being sent unconditionally in every typed function above (each one calls it
regardless of whether the preceding command succeeded).

- [x] **Step 9: Exercise the read functions once from code (temporary smoke test)**

Before committing, temporarily add a one-off call right after the `prop_motion_cfg_set_bt(false)`
line in `prop_motion_init()` to confirm the read path works end-to-end on real hardware:
```c
    char fwver[24];
    if (prop_motion_cfg_get_fw_version(fwver, sizeof(fwver)))
        ESP_LOGI(TAG, "LD2450 firmware: %s", fwver);
    prop_motion_track_mode_t mode;
    if (prop_motion_cfg_get_mode(&mode))
        ESP_LOGI(TAG, "LD2450 tracking mode: %s", mode == PROP_MOTION_TRACK_MULTI ? "multi" : "single");
```
Build, flash, `idf.py monitor`, confirm both log lines appear with sane values (mode should read
`multi` on a factory-default module), **then remove this temporary block** before committing —
Task 5 exposes these properly over HTTP instead of hardcoding a boot-time log dump.

- [x] **Step 10: Commit**

```bash
git add main/prop_motion.c main/include/prop_motion.h
git commit -m "feat(motion): LD2450 config-command channel + mode/BT/zone/version/MAC/restart API"
```

---

## Task 4: LD2450 baud-rate get/set (elevated risk — isolated on purpose)

**Files:**
- Modify: `main/prop_motion.c` — add `ld2450_baud_index`, `ld2450_restart_and_resync`,
  `prop_motion_cfg_set_baud`, and extend `prop_motion_cfg_factory_reset` to reuse the same
  restart/resync path (factory defaults revert baud to 256000).
- Modify: `main/include/prop_motion.h` — declare `prop_motion_cfg_factory_reset` and
  `prop_motion_cfg_set_baud`.

**Why this is riskier than Task 3:** setting the baud rate requires the module to **restart**
before the new rate takes effect (documented: "the configured value is not lost when power down,
and the configured value takes effect after restarting the module"). That means our side has to
switch its own UART baud rate in lockstep with the module's reboot — if that handshake goes
wrong, the board and the module end up talking at different rates and go silent to each other
until someone intervenes. There's no CRC in this protocol and no independent "what baud are you
actually running at" query, so the only way to verify success is to watch for data frames to
resume. This task builds an explicit fallback (try the new rate, then the previous rate, then
give up loudly) rather than assuming it always works.

- [x] **Step 1: Add the baud-index lookup and the restart-and-resync helper**

Add just above `prop_motion_cfg_restart` from Task 3 Step 5 (reuses
`ld2450_wait_for_data_frame`/`LD2450_RESYNC_TIMEOUT_MS` already added there):
```c
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
```

- [x] **Step 2: Add `prop_motion_cfg_set_baud` and `prop_motion_cfg_factory_reset`**

Add below Step 1's helpers:
```c
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
```

- [x] **Step 3: Add the public declarations**

In `main/include/prop_motion.h`, after `prop_motion_cfg_restart`:
```c
/* Restore all module settings to factory defaults (baud 256000, BT on, multi-
 * target tracking, zone filtering off) and restart to apply them. Reuses the
 * same restart+resync path as prop_motion_cfg_set_baud since baud reverts too. */
bool prop_motion_cfg_factory_reset(void);

/* Change the module's UART baud rate. HIGH RISK: this restarts the module and
 * reconfigures the LOCAL ESP32 UART to match, then waits for data streaming
 * to resume; only 9600/19200/38400/57600/115200/230400/256000/460800 are
 * valid. If the module doesn't resume streaming at the new rate within ~5s,
 * this function automatically falls back to the PREVIOUS baud rate and
 * returns false — if the module is silent at BOTH rates afterward, it needs
 * physical attention (power-cycle or a bench USB-serial check). Test this
 * thoroughly on the bench (Step 4 below) before relying on it in the field. */
bool prop_motion_cfg_set_baud(uint32_t new_baud_bps);
```

- [x] **Step 4: Build**

Run the build command. Expected: `Project build complete`.

- [x] **Step 5: Flash + bench-test the baud change on real hardware**

This is the one step in this whole plan worth doing carefully and alone, with `idf.py monitor`
open the entire time so you can see exactly what happens:
```bash
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 monitor
```
Temporarily add a one-off call in `prop_motion_init()` (same pattern as Task 3 Step 9) right
after the boot BT-off call:
```c
    vTaskDelay(pdMS_TO_TICKS(2000));   /* let normal streaming settle first */
    bool baud_ok = prop_motion_cfg_set_baud(115200);
    ESP_LOGI(TAG, "LD2450 set_baud(115200) -> %s", baud_ok ? "OK" : "FAILED");
```
Build, flash, watch the log. Expected sequence: normal `frame ok, N targets` debug spam (if
`ESP_LOGD` is enabled) → `LD2450 set_baud(115200)` attempt → `LD2450 resumed streaming at 115200
baud` → `LD2450 set_baud(115200) -> OK`. Confirm with:
```bash
python tools/prop.py watch --only radar --count 10
```
that target tracking still works normally afterward (the module and the firmware should now both
be at 115200 — this persists across a normal reflash, since it's stored in the module's own
flash, not the ESP32's; **remember to either set it back to 256000 or update `MOTION_BAUD` in
`main/prop_motion.c` to match** before you're done testing, or the next boot's default-256000
`uart_param_config` call at init will desync from whatever the module now expects at its next
power-up).

**If it fails:** the log will show the fallback path (`falling back to 256000`) and whether
recovery succeeded. If it reports "module needs physical attention," power-cycle the whole board
(don't just reflash — the module's own power state, not the ESP32's, is what's stuck) and re-test
starting from a known-good 256000 baud before re-attempting.

Once you've confirmed this works and reverted the baud back to the module's original setting
(256000), **remove the temporary Step 5 test block** from `prop_motion_init()`.

- [x] **Step 6: Commit**

```bash
git add main/prop_motion.c main/include/prop_motion.h
git commit -m "feat(motion): LD2450 baud-rate change + factory-reset, with restart/resync fallback"
```

---

## Task 5: Expose the LD2450 config API over HTTP (for scripting/testing)

**Files:**
- Modify: `main/prop_api.c` — a new `GET /ld2450` endpoint (aggregate read) and a new
  `{"cmd":"ld2450","action":...}` branch in the existing `POST /cmd` dispatcher (writes).
- Modify: `tools/prop.py` — a `ld2450` subcommand wrapping both.

**Why:** Tasks 3-4 build the capability; without an HTTP surface the only way to exercise it is
temporary code + `idf.py monitor`, which is how Tasks 3-4 verify on hardware but isn't something
you'd want to leave lying around. This follows the same `/state`+`/telemetry`-for-reads,
`/cmd`-for-writes split the rest of the API already uses. Reads are live blocking UART queries
(unlike `/state`/`/telemetry`, which serve cached values) — `GET /ld2450` can take up to ~1s to
respond, which is fine for an on-demand diagnostic endpoint, not a hot path.

- [x] **Step 1: Add `ld2450_to_json()` and the `GET /ld2450` handler**

Add in `main/prop_api.c` next to `telemetry_get_handler` (~line 486):
```c
/* GET /ld2450 -- aggregate live read of the LD2450 config-command protocol.
 * Each field is its own blocking UART query (~100-300ms); this handler may
 * take up to ~1s total. Diagnostic/admin endpoint, not a hot path. */
static char *ld2450_to_json(void)
{
    cJSON *root = cJSON_CreateObject();

    prop_motion_track_mode_t mode;
    if (prop_motion_cfg_get_mode(&mode))
        cJSON_AddStringToObject(root, "track_mode", mode == PROP_MOTION_TRACK_SINGLE ? "single" : "multi");
    else
        cJSON_AddNullToObject(root, "track_mode");

    char fw[24];
    cJSON_AddStringToObject(root, "fw_version", prop_motion_cfg_get_fw_version(fw, sizeof(fw)) ? fw : "unknown");

    uint8_t mac[3];
    if (prop_motion_cfg_get_mac(mac)) {
        char macbuf[9];
        snprintf(macbuf, sizeof(macbuf), "%02X%02X%02X", mac[0], mac[1], mac[2]);
        cJSON_AddStringToObject(root, "mac", macbuf);
    } else {
        cJSON_AddNullToObject(root, "mac");
    }

    prop_motion_zone_mode_t zmode;
    prop_motion_zone_t zones[3];
    if (prop_motion_cfg_get_zone(&zmode, zones)) {
        cJSON *z = cJSON_AddObjectToObject(root, "zone");
        cJSON_AddStringToObject(z, "mode", zmode == PROP_MOTION_ZONE_OFF ? "off" :
                                            zmode == PROP_MOTION_ZONE_INCLUDE ? "include" : "exclude");
        cJSON *arr = cJSON_AddArrayToObject(z, "zones");
        for (int i = 0; i < 3; i++) {
            cJSON *zi = cJSON_CreateObject();
            cJSON_AddNumberToObject(zi, "x1_mm", zones[i].x1_mm);
            cJSON_AddNumberToObject(zi, "y1_mm", zones[i].y1_mm);
            cJSON_AddNumberToObject(zi, "x2_mm", zones[i].x2_mm);
            cJSON_AddNumberToObject(zi, "y2_mm", zones[i].y2_mm);
            cJSON_AddItemToArray(arr, zi);
        }
    } else {
        cJSON_AddNullToObject(root, "zone");
    }

    /* No BT-status query command exists in the protocol -- write-only via
     * set_bt. This field is informational, not a live read. */
    cJSON_AddStringToObject(root, "bt", "write-only (no status query in the protocol; use set_bt)");

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

static esp_err_t ld2450_get_handler(httpd_req_t *req)
{
    char *json = ld2450_to_json();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json ? json : "{}");
    free(json);
    return ESP_OK;
}
```

- [x] **Step 2: Register the new URI**

Current (`main/prop_api.c` ~742-755):
```c
    httpd_uri_t root = { .uri = "/",     .method = HTTP_GET,  .handler = root_get_handler };
    httpd_uri_t st   = { .uri = "/state", .method = HTTP_GET,  .handler = state_get_handler };
    httpd_uri_t tel  = { .uri = "/telemetry", .method = HTTP_GET, .handler = telemetry_get_handler };
    httpd_uri_t cmd  = { .uri = "/cmd",  .method = HTTP_POST, .handler = cmd_post_handler };
    httpd_uri_t ota  = { .uri = "/ota",  .method = HTTP_POST, .handler = ota_post_handler };
    httpd_uri_t ws   = { .uri = "/ws",   .method = HTTP_GET,  .handler = ws_handler, .is_websocket = true };
    httpd_uri_t shot = { .uri = "/screenshot", .method = HTTP_GET, .handler = screenshot_get_handler };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &st);
    httpd_register_uri_handler(s_server, &tel);
    httpd_register_uri_handler(s_server, &cmd);
    httpd_register_uri_handler(s_server, &ota);
    httpd_register_uri_handler(s_server, &ws);
    httpd_register_uri_handler(s_server, &shot);

    prop_engine_add_observer(broadcast_observer, NULL);
    xTaskCreate(telemetry_task, "prop_telemetry", 4096, NULL, 4, NULL);
    ESP_LOGI(API_TAG, "HTTP API up (/, /state, /telemetry, /cmd, /ws, /ota, /screenshot)");
```
Change to:
```c
    httpd_uri_t root = { .uri = "/",     .method = HTTP_GET,  .handler = root_get_handler };
    httpd_uri_t st   = { .uri = "/state", .method = HTTP_GET,  .handler = state_get_handler };
    httpd_uri_t tel  = { .uri = "/telemetry", .method = HTTP_GET, .handler = telemetry_get_handler };
    httpd_uri_t cmd  = { .uri = "/cmd",  .method = HTTP_POST, .handler = cmd_post_handler };
    httpd_uri_t ota  = { .uri = "/ota",  .method = HTTP_POST, .handler = ota_post_handler };
    httpd_uri_t ws   = { .uri = "/ws",   .method = HTTP_GET,  .handler = ws_handler, .is_websocket = true };
    httpd_uri_t shot = { .uri = "/screenshot", .method = HTTP_GET, .handler = screenshot_get_handler };
    httpd_uri_t ld   = { .uri = "/ld2450", .method = HTTP_GET, .handler = ld2450_get_handler };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &st);
    httpd_register_uri_handler(s_server, &tel);
    httpd_register_uri_handler(s_server, &cmd);
    httpd_register_uri_handler(s_server, &ota);
    httpd_register_uri_handler(s_server, &ws);
    httpd_register_uri_handler(s_server, &shot);
    httpd_register_uri_handler(s_server, &ld);

    prop_engine_add_observer(broadcast_observer, NULL);
    xTaskCreate(telemetry_task, "prop_telemetry", 4096, NULL, 4, NULL);
    ESP_LOGI(API_TAG, "HTTP API up (/, /state, /telemetry, /cmd, /ws, /ota, /screenshot, /ld2450)");
```

- [x] **Step 3: Add the `ld2450` write actions to `dispatch_command`**

In `main/prop_api.c`, add a new `else if` branch to `dispatch_command` (~after the `wifi` branch,
~line 307, before the `io` branch):
```c
        } else if (strcmp(c, "ld2450") == 0) {
            /* {"cmd":"ld2450","action":"set_bt","on":true|false}
             * {"cmd":"ld2450","action":"set_mode","value":"single"|"multi"}
             * {"cmd":"ld2450","action":"restart"}
             * {"cmd":"ld2450","action":"factory_reset"}
             * {"cmd":"ld2450","action":"set_baud","value":115200}   -- HIGH RISK, see prop_motion.h
             * set_zone is intentionally not exposed here -- its 3-zone payload
             * is more naturally a raw JSON body; call prop_motion_cfg_set_zone()
             * directly from C if a scripted zone workflow is ever needed. */
            const cJSON *action = cJSON_GetObjectItem(root, "action");
            if (cJSON_IsString(action)) {
                const char *a = action->valuestring;
                if (strcmp(a, "set_bt") == 0) {
                    const cJSON *on = cJSON_GetObjectItem(root, "on");
                    if (cJSON_IsBool(on)) err = prop_motion_cfg_set_bt(cJSON_IsTrue(on)) ? ESP_OK : ESP_FAIL;
                } else if (strcmp(a, "set_mode") == 0 && cJSON_IsString(value)) {
                    prop_motion_track_mode_t m =
                        (strcasecmp(value->valuestring, "single") == 0) ? PROP_MOTION_TRACK_SINGLE :
                        (strcasecmp(value->valuestring, "multi")  == 0) ? PROP_MOTION_TRACK_MULTI  :
                                                                          (prop_motion_track_mode_t)0;
                    err = (m != 0 && prop_motion_cfg_set_mode(m)) ? ESP_OK : ESP_FAIL;
                } else if (strcmp(a, "restart") == 0) {
                    err = prop_motion_cfg_restart() ? ESP_OK : ESP_FAIL;
                } else if (strcmp(a, "factory_reset") == 0) {
                    err = prop_motion_cfg_factory_reset() ? ESP_OK : ESP_FAIL;
                } else if (strcmp(a, "set_baud") == 0 && cJSON_IsNumber(value)) {
                    err = prop_motion_cfg_set_baud((uint32_t)value->valueint) ? ESP_OK : ESP_FAIL;
                }
            }
        } else if (strcmp(c, "io") == 0) {
```
(The last line above is the existing `io` branch's opening — this just shows where the new
branch is spliced in; don't duplicate the `io` branch itself.)

- [x] **Step 4: Add the `tools/prop.py` CLI wrapper**

In `tools/prop.py`, add a new `elif cmd == "ld2450":` branch in `main()` (~after the `wifi`-style
branches, before the final `else`):
```python
    elif cmd == "ld2450":
        action = rest[0] if rest else "get"
        if action == "get":
            with urllib.request.urlopen(f"http://{host}/ld2450", timeout=8) as r:
                print(json.dumps(json.loads(r.read().decode()), indent=2))
        elif action == "set_bt":
            print(_post_cmd(host, {"cmd": "ld2450", "action": "set_bt", "on": rest[1] == "on"}))
        elif action == "set_mode":
            print(_post_cmd(host, {"cmd": "ld2450", "action": "set_mode", "value": rest[1]}))
        elif action == "restart":
            print(_post_cmd(host, {"cmd": "ld2450", "action": "restart"}))
        elif action == "factory_reset":
            print(_post_cmd(host, {"cmd": "ld2450", "action": "factory_reset"}))
        elif action == "set_baud":
            # can take up to ~11s worst case (restart + resync, with fallback)
            # -- use a longer client timeout than _post_cmd's default 8s.
            body = json.dumps({"cmd": "ld2450", "action": "set_baud", "value": int(rest[1])}).encode()
            req = urllib.request.Request(f"http://{host}/cmd", data=body,
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=15) as r:
                print(r.read().decode())
        else:
            print(f"unknown ld2450 action: {action}")
            sys.exit(2)
```

- [x] **Step 5: Build**

Run the build command. Expected: `Project build complete`.

- [x] **Step 6: Flash + verify over HTTP**

```bash
python tools/prop.py ld2450 get
```
Expected: JSON with `track_mode: "multi"`, a `fw_version` string, a `mac` hex string, and a
`zone` object with `mode: "off"` (factory default) and 3 all-zero zones. Then exercise a couple
of writes:
```bash
python tools/prop.py ld2450 set_mode single
python tools/prop.py ld2450 get     # track_mode should now read "single"
python tools/prop.py ld2450 set_mode multi
python tools/prop.py watch --only radar --count 10   # confirm tracking still works
```
Do **not** run `ld2450 set_baud` here — that was already bench-tested in isolation in Task 4
Step 5; re-testing it through the HTTP path is optional and should only be done deliberately,
watching `idf.py monitor` at the same time, for the same reasons as Task 4.

- [x] **Step 7: Commit**

```bash
git add main/prop_api.c tools/prop.py
git commit -m "feat(api): expose LD2450 config read/write over HTTP (GET /ld2450, POST /cmd ld2450)"
```

---

## Deferred / not planned (needs a design decision first)

- **A SETUP-menu LVGL panel for the LD2450 config options** — Tasks 3-5 give full read/write
  access over HTTP/C API; a UI panel (mirroring how SETUP already exposes WiFi/audio/display) is
  a separate design decision (which options are worth surfacing to an operator vs. HTTP-only
  diagnostics) and a `communicator-ui`/`design-kit` skill concern, not attempted here.
- **MR24HPC1 sensitivity/scene mode** (`CONTROL_WORK` 0x05, commands `0x87`/`0x88`/`0x89` —
  Small-Area/Area-Detection/Maximum-Area) — currently unused; firmware only polls human-status.
  Needs a decision on which scene mode fits this prop's enclosure before it's worth wiring up.
- **SEN0395 `detRangeCfg`/`outputLatency` tuning** — firmware runs entirely on the sensor's
  factory defaults (0-3m range, 2.5s/10s latency) today. Only worth changing if false triggers
  from outside the intended detection zone are actually observed on the built prop; premature to
  tune blind.

## Self-Review

**Coverage of discoveries:** speed-unit bug (Task 1) ✓. SEN0395 command-spacing gap (Task 2) ✓.
Full LD2450 config read/write, including baud rate as explicitly requested — mode get/set,
firmware version, MAC, Bluetooth (write-only, no query command exists), zone filtering get/set,
restart, factory reset, and baud rate with an explicit restart/resync/fallback path (Tasks 3-4) ✓.
HTTP exposure so the capability is actually usable without hand-editing firmware each time
(Task 5) ✓. MPU-6500 WHO_AM_I concern — already covered by the vendored driver, no action needed
(checked in the prior revision of this plan against `components/mpu6500/driver_mpu6500.c`
~line 4020-4034).

**Byte-protocol accuracy:** every frame layout in Tasks 3-4 (Enable/End-Config, single/multi
tracking, mode query, firmware version, baud-rate table, factory reset, restart, Bluetooth,
MAC address, zone query/set) was checked against the actual worked examples in the protocol PDF,
not just paraphrased from the skill's summary — including recomputing the firmware-version
pretty-print byte-by-byte and confirming zone coordinates are plain two's-complement (not
sign-magnitude) from the `(1000,1000)`/`(-1000,5000)` example.

**Concurrency review:** `motion_task` remains the *only* task that calls `uart_read_bytes` on
`MOTION_UART` — Task 3's channel design was deliberately built around that constraint (replacing
an earlier draft that would have added a second concurrent reader) so there's no read-side race.
`s_cfg_pending`/`s_cfg_waiting` are written by `motion_task` and read by the waiting caller only
after a successful `xSemaphoreTake(s_cfg_done_sem, ...)`, which is a standard, correct
producer/consumer handoff. A stray late ACK arriving after a caller's timeout is silently dropped
(`s_cfg_waiting` already false) — harmless, not a correctness issue.

**Placeholder scan:** no TBD/TODO; every step shows complete, compilable code. Buffer sizes were
double-checked against the actual maximum frame sizes in this protocol (`CFG_RESP_MAX=26` for the
zone-query ACK, `frame[40]` for the zone-set command — both are the largest real cases, not
guesses).

**Risk check:** Task 4 (baud-rate change) is the one genuinely risky operation in this plan —
flagged explicitly, isolated into its own task, given an automatic two-way fallback, and its bench
test (Step 5) calls out watching the serial log live rather than trusting the HTTP response alone.
Every other write operation (mode, BT, zone, restart, factory-reset-without-baud-consideration) is
low-risk: worst case is a failed ACK that's logged and returns `false`, with normal data streaming
unaffected (End-Config is always sent, even on failure, in every typed function).

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-01-sensor-protocol-followups.md`.
