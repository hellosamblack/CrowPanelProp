#ifndef _PROP_IMU_H_
#define _PROP_IMU_H_

/* prop_imu — InvenSense MPU-6050 6-axis IMU driver on the existing I2C bus.
 *
 * The MPU-6050 provides 3-axis accelerometer and 3-axis gyroscope data via
 * I2C (address 0x68, AD0 low). A complementary filter fuses accel and gyro
 * readings at ~50 Hz to produce low-drift pitch and roll estimates. Yaw is
 * integrated from the gyro Z axis only (will drift over time — acceptable for
 * a prop display).
 *
 * Non-fatal: if the sensor does not ACK (not wired / powered off),
 * prop_imu_available() stays false and callers get zero / false — the prop
 * never hangs.
 *
 * I2C bus: uses the global i2c_bus_handle initialised by bsp_i2c.
 * Config:  accel ±2g, gyro ±250°/s.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Bring up the MPU-6050 on the existing I2C bus and start the orientation task.
 * Returns ESP_ERR_NOT_FOUND if the device doesn't ACK (sensor not wired).
 * Non-fatal: prop_imu_available() stays false. */
esp_err_t prop_imu_init(void);

bool prop_imu_available(void);

/* Copy latest pitch/roll/yaw (degrees) into out-params. Any pointer may be NULL.
 * Returns false if init failed or no data yet. Cheap cached read under spinlock. */
bool prop_imu_get_orientation(float *pitch, float *roll, float *yaw);

/* Copy raw accel (in g, ×1000 fixed-point integer for spinlock safety) and gyro
 * (in °/s ×1000). Arrays must hold 3 elements [X,Y,Z]. May be NULL. */
void prop_imu_get_raw(int32_t accel_milli_g[3], int32_t gyro_milli_dps[3]);

#endif /* _PROP_IMU_H_ */
