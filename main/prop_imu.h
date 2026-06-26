#pragma once
/* prop_imu — MPU-6050 IMU driver with DMP MotionApps 2.0 firmware.
 *
 * Wiring (J7 header, all pins adjacent):
 *   GPIO27 (J7/A19) → SCL    GPIO28 (J7/B20) → SDA
 *   GPIO25 (J7/A17) → INT    3.3V → VCC   GND → GND
 *
 * Exposes quaternion, YPR angles, and raw accel from the on-chip DMP.
 * Non-fatal: prop_imu_init() returns ESP_ERR_NOT_FOUND if the sensor is
 * absent; the rest of the prop runs normally, all getters return 0/false.
 */
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float qw, qx, qy, qz;   /* normalized quaternion (DMP output) */
    float yaw;               /* about Z-axis, radians, -π..+π */
    float pitch;             /* nose up/down, radians, -π/2..+π/2 */
    float roll;              /* tilt left/right, radians, -π/2..+π/2 */
    int16_t ax, ay, az;      /* raw accel (±2 g, 16384 LSB/g) */
    int16_t gx, gy, gz;      /* raw gyro  (±2000 dps) */
    bool valid;              /* true once the first DMP packet has arrived */
    bool online;             /* true when sensor initialized OK */
} prop_imu_data_t;

/* Initialize the MPU-6050 on I2C_NUM_1 (GPIO27/28), load DMP firmware,
 * and launch the background task.  Returns ESP_ERR_NOT_FOUND when the
 * sensor does not ACK its I2C address (hardware not connected). */
esp_err_t prop_imu_init(void);

/* Copy the latest DMP data into *out (thread-safe). */
void prop_imu_get_data(prop_imu_data_t *out);
