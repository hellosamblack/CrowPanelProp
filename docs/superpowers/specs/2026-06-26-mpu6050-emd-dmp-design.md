# MPU-6050 eMD DMP upgrade — design

**Date:** 2026-06-26
**Branch:** feature/alien-motion-tracker
**Status:** approved design → implementation

## Goal

Replace the current MPU-6050 driver (a port of Jeff Rowberg's i2cdevlib
*MotionApps 2.0*) with a port of InvenSense's **eMD (Embedded Motion Driver) 5.1**
feature set, as implemented by the LibDriver `mpu6050` library. The MotionApps20
path only exposes a read-only quaternion+raw FIFO packet (~30% of the chip). eMD
unlocks the DMP feature-flag mechanism: tap gestures, pedometer, screen
orientation events, and on-chip gyro auto-calibration, plus the register-level
extras (die temperature, motion/free-fall interrupt, self-test + bias).

The MPU-6050 holds **one** DMP firmware image at a time, so MotionApps20 and eMD
are mutually exclusive — this is a full replacement, not an addition.

## Source material (vendored, MIT)

- LibDriver mpu6050: `datasheets/externalDevices/MPU-6050 Accelerometer and Gyro/libdriver mpu6050/`
  - `src/driver_mpu6050.{c,h}` — core driver: register map, DMP memory I/O,
    feature/firmware load, FIFO + gesture parsing, tap/orient callbacks, pedometer.
  - `src/driver_mpu6050_code.h` — eMD DMP firmware blob.
  - `example/driver_mpu6050_dmp.c` — reference init sequence (template only).

## Approach (chosen: A — vendor core, thin wrapper)

Vendor the LibDriver core **unmodified** and own only the glue. Rejected:
B (hand-port everything into one file — high transcription-bug risk on the fiddly
gesture/FIFO code) and C (hybrid — inherits the hard parts' risk anyway).

## File layout

```
components/mpu6050/
  CMakeLists.txt                 # idf_component_register, REQUIRES driver/esp_common
  driver_mpu6050.c   (vendored)
  driver_mpu6050.h   (vendored)
  driver_mpu6050_code.h (vendored)
main/
  prop_imu.c   (rewritten adapter)
  prop_imu_iic.c (NEW — bsp_i2c link functions)
main/include/
  prop_imu.h   (additive API changes)
```

A proper ESP-IDF component avoids the `main/CMakeLists.txt` `GLOB main/*.c` pitfall
and keeps the vendored code self-contained in the project tree.

## Components and responsibilities

### `components/mpu6050/` (vendored core)
- **What:** platform-agnostic MPU-6050 + eMD DMP driver via a handle + function-pointer
  link layer (`DRIVER_MPU6050_LINK_*`).
- **Use:** attach link funcs, `mpu6050_init`, `mpu6050_dmp_load_firmware`,
  `mpu6050_dmp_set_feature`, `mpu6050_dmp_read`, callbacks.
- **Depends on:** only the link functions we provide. No direct ESP-IDF calls.

### `main/prop_imu_iic.c` (link layer)
- **What:** the 6 link functions LibDriver needs — `iic_init/deinit/read/write`,
  `delay_ms`, `debug_print`.
- **How:** `iic_init` registers the MPU-6050 (addr 0x68) on the shared `bsp_i2c`
  bus via `i2c_dev_register` and stores the device handle; `read`/`write` map the
  (addr,reg,buf,len) calls onto `i2c_read_reg`/`i2c_write_reg` on that handle;
  `delay_ms` → `vTaskDelay`; `debug_print` → `ESP_LOGx(TAG,…)`.
- **Depends on:** `bsp_i2c`, FreeRTOS, `esp_log`.

### `main/prop_imu.c` (adapter — rewritten)
- **What:** owns the singleton lifecycle, the background poll task, the mutex-protected
  cache, and the public API. The *only* file that changes the prop's behavior.
- **Init sequence** (adapted from `driver_mpu6050_dmp.c`):
  1. attach link funcs, `mpu6050_init`, wake.
  2. `mpu6050_self_test` → `dmp_gyro_accel_raw_offset_convert` → `dmp_set_accel_bias`/`set_gyro_bias`.
  3. enable temp sensor; config range (accel ±2g, gyro ±2000dps), DLPF, rate.
  4. `dmp_load_firmware`; `dmp_set_orientation` (identity mount matrix — adjust if §Risk shows axis flip).
  5. `dmp_set_feature(6X_QUAT | TAP | ORIENT | PEDOMETER | GYRO_CAL | SEND_RAW_ACCEL | SEND_CAL_GYRO)`.
  6. register tap + orient callbacks; configure tap thresholds (libdriver defaults).
  7. program motion + free-fall threshold/duration registers; enable motion & free-fall interrupts (read via INT_STATUS polling — no INT GPIO).
  8. `dmp_set_enable(true)`, `force_fifo_reset`.
- **Poll task** (20 ms, no INT pin): `mpu6050_dmp_read` for quat/YPR/accel/cal-gyro;
  read `temp`, `pedometer_step_count`, and `INT_STATUS` (motion/free-fall bits).
  Tap/orient arrive via callbacks invoked inside `mpu6050_dmp_read`'s FIFO walk;
  callbacks store latest tap/event into the cache. All cache writes under the mutex.
- **Non-fatal:** if the sensor doesn't ACK at init, `online=false`, task not started,
  all getters return zero/false — prop never hangs (unchanged contract).

## Public API (`prop_imu.h`) — additive

Existing functions keep their exact contract (gimbal + VITALS code untouched):
`prop_imu_init`, `prop_imu_get_data`, `prop_imu_available`,
`prop_imu_get_orientation` (degrees), `prop_imu_get_raw`.

`prop_imu_data_t` gains:
```c
float    temp_c;            /* die temperature, °C */
uint32_t step_count;        /* cumulative pedometer steps */
int16_t  cgx, cgy, cgz;     /* calibrated gyro (eMD SEND_CAL_GYRO) */
```

New event getters (latched, cleared on read):
```c
typedef struct { uint8_t count; uint8_t direction; bool present; } prop_imu_tap_t;
typedef enum { PROP_IMU_EVT_NONE=0, PROP_IMU_EVT_MOTION, PROP_IMU_EVT_FREEFALL } prop_imu_event_t;

prop_imu_tap_t   prop_imu_get_tap(void);          /* returns + clears latest tap */
prop_imu_event_t prop_imu_get_motion_event(void); /* returns + clears latest motion/freefall */
```

## Behavior wiring

- **Tap** (consumed in `prop_engine` tick): double-tap (count ≥ 2) →
  `prop_audio_play(PA_SIGNAL)` + `prop_engine_set_scene(SCENE_SIGNAL_ACQUIRED)`.
  Single tap → `prop_audio_play(PA_TAB)` soft ack. Tap direction surfaced as a
  brief VITALS line.
- **Pedometer** (VITALS via `ui_observer`): row "OPERATOR / ON FOOT" →
  `NNNN steps · ~NNN m` (steps × 0.75 m).
- **Die temp** (VITALS): row "CORE TEMP" → `%.1f °C` from `temp_c`.
- **Free-fall / motion** (consumed in `prop_engine` tick from `prop_imu_get_motion_event`):
  FREEFALL → `SCENE_ALERT` + alert LED + `prop_audio_play(PA_ALERT)` ("UNIT DROPPED");
  MOTION → lightweight LED/flag only (no scene change, avoids thrash). Latched so each
  event fires exactly once.
- **Gyro auto-cal:** always-on via `GYRO_CAL` feature flag; no UI. Reduces gimbal yaw drift.

## Risk & verification

- **Primary risk:** the rewritten quaternion path altering gimbal behavior. The
  motion-scanner artificial-horizon and VITALS YPR bars depend on
  `prop_imu_get_orientation()` sign/axis conventions.
  **Mitigation:** eMD pitch/roll/yaw derive from the same 6X quaternion, so the
  contract should hold. After the swap, verify live via the screenshot loop
  (`python tools/prop.py shot g.png --screen motion --wait`) at three poses —
  flat, nose-up, roll-left — and confirm the horizon moves the same direction as
  before. Any flip is corrected in the **adapter** (axis/sign or the DMP mount
  matrix), never in the vendored core.
- **Build:** new component + new `main/*.c` → `idf.py reconfigure` before `build`
  (CLAUDE.md GLOB note). Confirm `components/mpu6050` is picked up.
- **No automated tests in repo** — verification is on-hardware (COM7) via the
  build/flash/screenshot loop. State checked via `GET /state` and VITALS screenshots.
- **Safety:** preserve non-fatal sensor-absent behavior; never call I2C under the
  LVGL lock (poll task is independent; UI reads cache only).

## MOTION SCAN screen → all-sensor dashboard (`build_motion_panel`)

Rework the MOTION SCAN panel (`PK_MOTION`) into a single *Aliens motion-tracker*
dashboard that surfaces every sensor at once, matching
`resources/inspiration/alienMotionScanner/`. Visual rules:

- **Headliners (large, center):**
  - **Radar sweep fan** — existing rotating sweep + target blips, driven by the
    aux radar (SEN0395 mmWave / Seeed) target list. Keep the fan/arc look.
  - **Gimbal** — existing artificial-horizon line from `prop_imu_get_orientation()`.
- **Direction ring (NEW):** a 4-quadrant ring around the dot at the **base apex** of
  the radar fan (the dark quadrant ring in the reference). When the operator is
  moving, the quadrant matching the movement direction lights bright; others stay
  dim. Direction derived from IMU horizontal accel vector (`ax`,`ay`) → heading →
  N/E/S/W quadrant; magnitude gates the "moving" state (deadband near rest). This
  is **in addition to** the gimbal.
- **Analog signals → progress bars** (bottom-left stack, reference style):
  CSI movement (`prop_csi_get_motion` movement_milli vs threshold), CSI RF
  turbulence (`prop_csi_get_rf`), mic level (`prop_mic_get_db`), WiFi RSSI
  (`prop_net_get_rssi`), BLE strongest (`prop_ble_get_summary`), IMU accel
  magnitude. Numeric readouts (pedometer steps, core temp) shown as values like
  the reference "0.6579".
- **Booleans → dim/bright 3-letter all-caps words** parked in a corner (the
  "ATT/SUS/DEC" style): each is `COL_MUTE`/`COL_AMBER` when active, `COL_DIM` when
  inactive. Set: `SEN`/`SEE` (aux radar present), `MOV` (CSI motion), `LIV` (CSI
  live vs synthetic), `IMU` (online), `FFL` (free-fall latched), `MIC` (PDM up),
  `BLE` (available), `NET` (STA linked), `GEI` (geiger mode). Camera-legibility
  rule: active words use `COL_MUTE`+bold, never `COL_DIM`, per CLAUDE.md.

All readouts update in `ui_observer` under the existing `PK_MOTION` guard; no new
task. New widget pointers added to the `s_motion_*` block and NULLed in
`close_panel`. The direction ring is four `lv_obj`/arc segments (or a small canvas)
re-tinted per frame — cheap, one panel alive at a time.

## Out of scope (YAGNI)

- Screen auto-rotation from orient events (panel is fixed-mount); orient is captured
  but only logged, not acted on.
- I2C-master/aux-sensor passthrough, FSYNC, low-power cycle/wake modes.
- 3X (accel-only) quaternion mode.
