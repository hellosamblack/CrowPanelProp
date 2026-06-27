#ifndef _PROP_IMU_H_
#define _PROP_IMU_H_

/* prop_imu — MPU-6050 DMP MotionApps 2.0 driver on the shared I2C bus.
 *
 * The DMP on-chip processor produces fused quaternion + YPR + raw accel/gyro
 * at up to 200 Hz via a 42-byte FIFO packet. The driver polls the FIFO every
 * 20 ms and caches the result under a mutex.
 *
 * Non-fatal: if the sensor does not ACK (not wired / powered off),
 * prop_imu_available() stays false and callers get zero / false — the prop
 * never hangs.
 *
 * I2C bus: shared bsp_i2c bus (I2C_NUM_0, GPIO45/46). MPU-6050 addr 0x68
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
    int16_t gx, gy, gz;            /* raw gyro  (±2000 dps, 16.4 LSB/dps) */
    bool    valid;                  /* true once first DMP packet arrives */
    bool    online;                 /* true when sensor was found at init */
} prop_imu_data_t;

/* Bring up the MPU-6050 DMP and start the background FIFO reader task.
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

/* Accel in milli-g [X,Y,Z]; gyro in milli-°/s [X,Y,Z]. Arrays may be NULL. */
void prop_imu_get_raw(int32_t accel_milli_g[3], int32_t gyro_milli_dps[3]);

#endif /* _PROP_IMU_H_ */
