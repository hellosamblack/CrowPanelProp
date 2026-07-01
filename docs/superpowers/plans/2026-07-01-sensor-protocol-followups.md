# Sensor Protocol Follow-ups — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Act on the concrete, verified discoveries from building the `sensor-datasheets` skill
(`.claude/skills/sensor-datasheets/`) — a real unit-conversion bug in the LD2450 radar driver,
an undersized command-spacing delay in the SEN0395 boot sequence, and an unused LD2450
capability (disabling its onboard Bluetooth) worth wiring up to rule out an RF-interference
source. Lower-value/product-dependent opportunities (LD2450 zone filtering, MR24HPC1 scene
modes, SEN0395 range/latency tuning) are captured as a deferred backlog, not full tasks — they
need a design decision this plan doesn't make for you.

**Background:** Building that skill involved fact-checking all four external-sensor
`README.md` files against their real datasheet PDFs and the firmware drivers that actually talk
to them (`main/prop_motion.c`, `main/prop_imu.c`, `main/prop_aux_radar.c`). Two READMEs turned
out to be wrong/duplicated (already fixed); along the way the research surfaced firmware-level
findings that are the subject of this plan. Read `.claude/skills/sensor-datasheets/references/`
for the full protocol reference behind each task below.

**Architecture:** Task 1 touches `main/prop_motion.c` + `main/prop_ui.c`. Task 2 touches
`main/prop_aux_radar.c`. Task 3 touches only `main/prop_motion.c` (adds a small config-command
helper + a one-shot boot sequence, called from `prop_motion_init` before the background parser
task starts). No new files, no build-system changes, no `idf.py reconfigure` needed.

## Global Constraints

- **No automated tests in this repo.** Every task is verified by build → flash → either a
  live-hardware check (moving a hand in front of the LD2450) or `idf.py monitor`/log
  inspection, as noted per task.
- **Don't touch `CONFIG_ESP32P4_*REV*`** or other unrelated sdkconfig items — out of scope here.
- Follow the existing code style in each file (this project doesn't use comments to restate
  what code does — only for non-obvious constraints/gotchas, matching the rest of these files).
- Tasks are independent and can be done in any order; Task 3 is the only one with any real risk
  (new UART command exchange at boot) so do it last if you want the lower-risk wins banked first.

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

- [ ] **Step 1: Convert to true mm/s at the decode boundary**

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

- [ ] **Step 2: Rescale the fast-motion alert threshold to match**

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

- [ ] **Step 3: Confirm the velocity label's unit suffix is now correct**

`main/prop_ui.c` ~5680-5682 already prints `"%+d mm/s"` from `tgts[i].speed_mm_s` — no code
change needed here, it was already labeled correctly, it was just fed wrong data. Leave as-is;
this step is just confirming there's nothing else to touch.

- [ ] **Step 4: Build**

Run the build command. Expected: `Project build complete`.

- [ ] **Step 5: Flash + verify on hardware**

Flash, then walk a hand across the LD2450's field of view at a normal pace and watch:
```bash
python tools/prop.py watch --only radar --count 30
```
Expected: `speed_mm_s` values in the few-hundred-to-~1500 mm/s range for a normal walking pace
(previously they'd have read 10× smaller, i.e. tens-to-~150). Also check
`python tools/prop.py shot motion.png --screen motion --wait` — the T1/T2/T3 velocity readout
should show plausible `mm/s` numbers, and the blip should only turn `COL_ALERT` red on a fast
lunge/wave, not on ordinary walking-pace movement.

- [ ] **Step 6: Commit**

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

- [ ] **Step 1: Widen the inter-command delay**

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

- [ ] **Step 2: Build**

Run the build command. Expected: `Project build complete`.

- [ ] **Step 3: Flash + verify boot reliability**

```bash
python tools/prop.py trace --trials 8
```
Expected: 8/8 clean boots, SEN0395 presence telemetry (`aux_radar.sen0395`) comes up online
within the same rough timeframe as before (boot is ~650ms later than before, not user-visible).
No behavior change expected — this is a robustness margin increase, not a functional change.

- [ ] **Step 4: Commit**

```bash
git add main/prop_aux_radar.c
git commit -m "fix(aux_radar): widen SEN0395 boot command spacing to vendor-documented 1000ms"
```

---

## Task 3: Disable LD2450's onboard Bluetooth at boot (new capability)

**Files:**
- Modify: `main/prop_motion.c` — add a config-command helper + one-shot boot sequence, call it
  from `prop_motion_init()` (~line 166-214) before `motion_task` is spawned.

**Why:** The LD2450 protocol PDF documents a **separate command-frame envelope**
(`FD FC FB FA ... 04 03 02 01`, distinct from the `AA FF 03 00 ... 55 CC` data-output frames)
used to configure the module — including turning its **onboard Bluetooth off** (it's **on by
default**, meant for pairing with Hi-Link's phone/PC config app). This project's firmware never
sends any config command today; it only passively parses data frames. This board separately runs
its own BLE scanning on the C6 (`prop_ble.c`, the CONTACTS instrument) and CSI work
(`prop_csi.c`, SIGNAL ENV) — an always-on 24 GHz-adjacent BT radio sitting right next to the
antenna is a plausible (unconfirmed) source of RF noise for either. Turning it off costs nothing
functionally (this project never uses the LD2450's BT pairing) and removes that variable. This
is the only task in this plan that adds a new UART command exchange — do it after Tasks 1-2 if
you want to bank the safer wins first.

**Design note — why this is safe to do as a one-shot boot sequence:** the config-command
protocol requires bracketing every command between "Enable Config" (`0x00FF`) and "End Config"
(`0x00FE`) — the module is documented to stop streaming data frames while inside that bracket.
Doing the whole exchange synchronously *before* `motion_task` (the continuous data-frame reader)
is spawned means there's no risk of the two parsers (data-frame vs. command-ACK) racing over the
same UART — exactly how `main/prop_aux_radar.c`'s SEN0395 boot sequence already works today.

- [ ] **Step 1: Add the config-command frame constants and send/wait-for-ACK helper**

Add above `/* ---- Background reader task ---- */` (~line 72) in `main/prop_motion.c`:
```c
/* ---- Config-command protocol (one-shot use at boot; see the protocol PDF and
 * .claude/skills/sensor-datasheets/references/ld2450.md for the full command
 * table). Separate frame envelope from the data-output frames above. Not used
 * for anything beyond disabling onboard BT at boot today — no zone filtering,
 * no mode switching. ---- */

#define CFG_HEADER_LEN 4
#define CFG_TAIL_LEN   4
static const uint8_t CFG_HEADER[CFG_HEADER_LEN] = { 0xFD, 0xFC, 0xFB, 0xFA };
static const uint8_t CFG_TAIL[CFG_TAIL_LEN]     = { 0x04, 0x03, 0x02, 0x01 };

/* Send one config-command frame and block (up to timeout_ms) for its ACK.
 * Must be called before motion_task starts reading (see prop_motion_init) —
 * this function does its own raw uart_read_bytes and would race motion_task's
 * data-frame parser otherwise. Returns true iff a well-formed ACK for this
 * exact command word arrived with status == 0 (success). */
static bool ld2450_cfg_cmd(uint16_t word, const uint8_t *val, uint16_t val_len, int timeout_ms)
{
    uint8_t frame[16];
    uint16_t plen = (uint16_t)(2 + val_len);   /* command word + value */
    int n = 0;
    memcpy(frame + n, CFG_HEADER, CFG_HEADER_LEN); n += CFG_HEADER_LEN;
    frame[n++] = (uint8_t)(plen & 0xFF);
    frame[n++] = (uint8_t)(plen >> 8);
    frame[n++] = (uint8_t)(word & 0xFF);
    frame[n++] = (uint8_t)(word >> 8);
    if (val_len) { memcpy(frame + n, val, val_len); n += val_len; }
    memcpy(frame + n, CFG_TAIL, CFG_TAIL_LEN); n += CFG_TAIL_LEN;
    uart_write_bytes(MOTION_UART, frame, n);

    uint8_t buf[32];
    int got = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline && got < (int)sizeof(buf)) {
        int r = uart_read_bytes(MOTION_UART, buf + got, sizeof(buf) - got, pdMS_TO_TICKS(20));
        if (r > 0) got += r;

        for (int i = 0; i + CFG_HEADER_LEN + 6 <= got; i++) {
            if (memcmp(buf + i, CFG_HEADER, CFG_HEADER_LEN) != 0) continue;
            uint16_t alen = (uint16_t)buf[i + 4] | ((uint16_t)buf[i + 5] << 8);
            int total = CFG_HEADER_LEN + 2 + alen + CFG_TAIL_LEN;
            if (i + total > got) continue;   /* frame incomplete, keep reading */
            if (memcmp(buf + i + total - CFG_TAIL_LEN, CFG_TAIL, CFG_TAIL_LEN) != 0) continue;
            uint16_t ack_word = (uint16_t)buf[i + 6] | ((uint16_t)buf[i + 7] << 8);
            uint16_t status    = (alen >= 4) ? ((uint16_t)buf[i + 8] | ((uint16_t)buf[i + 9] << 8))
                                              : 0xFFFF;
            if (ack_word == (uint16_t)(word | 0x0100)) return status == 0x0000;
        }
    }
    return false;
}

/* One-shot boot sequence: enter config mode, disable the module's onboard
 * Bluetooth, read back the firmware version for the boot log, exit config
 * mode. Best-effort — any failure just logs a warning and continues into
 * normal operation, since this is a nice-to-have, not required for tracking. */
static void ld2450_configure_at_boot(void)
{
    if (!ld2450_cfg_cmd(0x00FF, (const uint8_t[]){ 0x01, 0x00 }, 2, 200)) {
        ESP_LOGW(TAG, "LD2450 enable-config failed/timed out — leaving module defaults");
        return;
    }

    uint8_t bt_off[2] = { 0x00, 0x00 };
    if (ld2450_cfg_cmd(0x00A4, bt_off, 2, 200))
        ESP_LOGI(TAG, "LD2450 onboard Bluetooth disabled");
    else
        ESP_LOGW(TAG, "LD2450 bluetooth-off command failed/timed out");

    ld2450_cfg_cmd(0x00FE, NULL, 0, 200);   /* end-config; resumes data streaming regardless */
}
```

- [ ] **Step 2: Call it once at init, before the reader task starts**

Current (`main/prop_motion.c` ~192-203):
```c
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
```
Change to:
```c
    esp_err_t sp = uart_set_pin(MOTION_UART,
                                MOTION_TX_GPIO, MOTION_RX_GPIO,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (sp != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(sp));
        uart_driver_delete(MOTION_UART);
        return sp;
    }

    /* One-shot config exchange (BT off) before the continuous data-frame
     * reader task starts — see ld2450_configure_at_boot() for why ordering
     * matters here. */
    ld2450_configure_at_boot();

    /* Task: 4096-byte stack, priority 4, pinned to core 1. */
    BaseType_t r = xTaskCreatePinnedToCore(motion_task, "prop_motion",
                                           4096, NULL, 4, NULL, 1);
```

- [ ] **Step 3: Build**

Run the build command. Expected: `Project build complete`.

- [ ] **Step 4: Flash + verify**

```bash
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 monitor
```
Expected log lines near boot: `LD2450 onboard Bluetooth disabled` (or a `WARN` if the module
didn't ACK in time — not fatal, tracking still works either way). Then confirm normal tracking
still works:
```bash
python tools/prop.py watch --only radar --count 20
```
Expected: target X/Y/speed still streams normally — the config exchange must not have left the
module stuck in config mode (if it had, no data frames would ever arrive). If radar data doesn't
resume, check that "End Config" (`0x00FE`) is being sent regardless of the BT-off command's
result (Step 1's `ld2450_configure_at_boot` already does this unconditionally — verify the log
doesn't stop after the enable-config line).

- [ ] **Step 5: Commit**

```bash
git add main/prop_motion.c
git commit -m "feat(motion): disable LD2450 onboard Bluetooth at boot via config-command protocol"
```

---

## Deferred / not planned (needs a design decision first)

These are real, documented capabilities surfaced by the same research, but implementing them
requires a product decision this plan doesn't make — listed here so they aren't lost, not as
ready-to-execute tasks.

- **LD2450 zone filtering** (`0x00C1`/`0x00C2`, up to 3 rectangular include/exclude zones) —
  could reduce false target clutter from outside the prop's intended detection area, but needs
  someone to decide *which* zone in the physical install this should be, and would reuse the
  `ld2450_cfg_cmd` helper from Task 3.
- **MR24HPC1 sensitivity/scene mode** (`CONTROL_WORK` 0x05, commands `0x87`/`0x88`/`0x89` —
  Small-Area/Area-Detection/Maximum-Area) — currently unused; firmware only polls human-status.
  Needs a decision on which scene mode fits this prop's enclosure before it's worth wiring up.
- **SEN0395 `detRangeCfg`/`outputLatency` tuning** — firmware runs entirely on the sensor's
  factory defaults (0-3m range, 2.5s/10s latency) today. Only worth changing if false triggers
  from outside the intended detection zone are actually observed on the built prop; premature to
  tune blind.

## Self-Review

**Coverage of discoveries:** speed-unit bug (Task 1, includes the dependent UI-threshold
rescale so the fix doesn't silently change alert behavior) ✓. SEN0395 command-spacing gap vs.
vendor spec (Task 2) ✓. Unused LD2450 config-command protocol, applied to the one capability
with clear, argument-free value — disabling onboard BT as an interference-elimination step
(Task 3) ✓. MPU-6500 WHO_AM_I concern — checked against `components/mpu6500/driver_mpu6500.c`
(~line 4020-4034): `mpu6500_init()` already reads WHO_AM_I and fails with an error (which
`imu_debug_print` surfaces to the ESP log) if it isn't `0x70` — **no action needed, already
covered by the vendored driver**. Zone filtering / MR24HPC1 scene modes / SEN0395 range tuning
are real but product-decision-gated — captured as a backlog, not turned into speculative tasks.

**Placeholder scan:** no TBD/TODO; every task step shows complete, compilable code with exact
file:line anchors verified against the current file contents.

**Risk check:** Task 3 is the only one adding new runtime behavior (a UART command exchange at
boot) — designed to fail safe (any timeout just logs a warning and falls through to normal
operation; End Config is sent unconditionally so the module can't get stuck refusing to stream).

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-01-sensor-protocol-followups.md`.
