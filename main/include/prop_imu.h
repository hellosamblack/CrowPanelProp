#ifndef _PROP_IMU_H_
#define _PROP_IMU_H_

/* prop_imu — MPU-6500 eMD DMP driver (LibDriver core) on the shared I2C bus.
 *
 * The installed IMU is an MPU-6500 (WHO_AM_I 0x70), not an MPU-6050. The DMP
 * on-chip processor produces a fused 6-axis quaternion + YPR + raw accel +
 * calibrated gyro via FIFO, plus tap gestures, a pedometer, and on-chip gyro
 * auto-calibration. The driver polls the FIFO every 40 ms and caches the result
 * under a mutex. Wraps the vendored LibDriver mpu6500 eMD core in
 * components/mpu6500. (Was a MotionApps 2.0 / MPU-6050 port.)
 *
 * Non-fatal: if the sensor does not ACK (not wired / powered off),
 * prop_imu_available() stays false and callers get zero / false — the prop
 * never hangs.
 *
 * I2C bus: shared bsp_i2c bus (I2C_NUM_0, GPIO45/46). MPU-6500 addr 0x68
 * does not conflict with the GT911 touch controller (addr 0x14/0x5D).
 * Config:  accel ±2g (16384 LSB/g), gyro ±2000 dps (16.4 LSB/dps).
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Full DMP output packet, produced at up to 200 Hz. */
typedef struct {
    float   qw, qx, qy, qz;       /* unit quaternion from DMP */
    float   yaw, pitch, roll;      /* radians, derived from quaternion + gravity */
    int16_t ax, ay, az;            /* raw accel (±2g, 16384 LSB/g) */
    int16_t gx, gy, gz;            /* calibrated gyro (eMD SEND_CAL_GYRO; ±2000 dps) */
    float   temp_c;                /* die temperature, °C (eMD) */
    uint32_t step_count;           /* cumulative pedometer steps (eMD) */
    bool    valid;                  /* true once first DMP packet arrives */
    bool    online;                 /* true when sensor was found at init */
} prop_imu_data_t;

/* Tap gesture (eMD). `direction` is mpu6500_dmp_tap_t: 1..6 = X/Y/Z up/down. */
typedef struct {
    uint8_t count;
    uint8_t direction;
    bool    present;               /* true if a tap occurred since last read */
} prop_imu_tap_t;

/* Motion / free-fall event (latched until read). */
typedef enum {
    PROP_IMU_EVT_NONE = 0,
    PROP_IMU_EVT_MOTION,
    PROP_IMU_EVT_FREEFALL,
} prop_imu_event_t;

/* Bring up the MPU-6500 DMP and start the background FIFO reader task.
 * Returns ESP_ERR_NOT_FOUND if the device doesn't ACK (sensor not wired).
 * Non-fatal: prop_imu_available() stays false. */
esp_err_t prop_imu_init(void);

/* Copy latest DMP packet into *out. Safe from any task. */
void prop_imu_get_data(prop_imu_data_t *out);

/* ── Compatibility API ─────────────────────────────────────────────────────
 * These functions match the complementary-filter API used by prop_ui's
 * motion-scan panel so the UI doesn't need modification. */

bool prop_imu_available(void);

/* Pitch/roll/yaw in DEGREES (not radians). Returns false until first packet. */
bool prop_imu_get_orientation(float *pitch, float *roll, float *yaw);

/* Return + clear the latest tap gesture (present=false if none since last call). */
prop_imu_tap_t prop_imu_get_tap(void);

/* Return + clear the latest motion / free-fall event. */
prop_imu_event_t prop_imu_get_motion_event(void);

#endif /* _PROP_IMU_H_ */
