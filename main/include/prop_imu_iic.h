#ifndef _PROP_IMU_IIC_H_
#define _PROP_IMU_IIC_H_

#include <stdint.h>

/* LibDriver mpu6500 link functions bridging the eMD core to the shared bsp_i2c
 * bus. Attach these to the handle via the DRIVER_MPU6500_LINK_* macros.
 * Single device (MPU-6500 @ 0x68); the `addr` argument from LibDriver is ignored
 * because bsp_i2c addresses the registered device handle directly.
 * All return 0 on success, 1 on failure (LibDriver convention). */
uint8_t prop_imu_iic_init(void);
uint8_t prop_imu_iic_deinit(void);
uint8_t prop_imu_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t prop_imu_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* _PROP_IMU_IIC_H_ */
