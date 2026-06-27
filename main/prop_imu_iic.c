/* prop_imu_iic — LibDriver mpu6050 link layer over the shared bsp_i2c bus.
 *
 * Single device (MPU-6050 @ 0x68 / AD0 low). The `addr` argument LibDriver
 * passes is ignored — bsp_i2c works on the registered device handle. The
 * MPU-6050 shares the bus with the GT911 touch controller (addr 0x14/0x5D)
 * with no conflict.
 */
#include "prop_imu_iic.h"
#include "bsp_i2c.h"
#include "esp_log.h"

#define IIC_TAG "IMU_IIC"
#define IMU_I2C_ADDR 0x68   /* 7-bit address (AD0 low) */

static i2c_master_dev_handle_t s_dev;

uint8_t prop_imu_iic_init(void)
{
    if (s_dev) return 0;
    s_dev = i2c_dev_register(IMU_I2C_ADDR);
    return s_dev ? 0 : 1;
}

uint8_t prop_imu_iic_deinit(void)
{
    /* bsp_i2c owns the bus lifetime; just drop our reference. */
    s_dev = NULL;
    return 0;
}

uint8_t prop_imu_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)addr;
    if (!s_dev) return 1;
    /* LibDriver's dmp_read issues a 0-length FIFO read when fewer than one full
     * packet is buffered. That's a benign no-op on STM32/RPi, but the ESP-IDF
     * new I2C master rejects size-0 transfers ("buffer or size invalid"). Treat
     * it as success so the DMP read loop continues cleanly. */
    if (len == 0) return 0;
    return i2c_read_reg(s_dev, reg, buf, len) == ESP_OK ? 0 : 1;
}

/* LibDriver writes `len` bytes starting at register `reg`. bsp_i2c's
 * i2c_write_reg writes a single byte, so build a [reg, buf...] frame and use
 * the raw i2c_write (matches how the DMP firmware blocks are written). */
uint8_t prop_imu_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)addr;
    if (!s_dev) return 1;
    if (len == 0) return 0;   /* no-op (see prop_imu_iic_read) */

    uint8_t tx[257];
    if (len > sizeof(tx) - 1) {
        ESP_LOGE(IIC_TAG, "write len %u too large", (unsigned)len);
        return 1;
    }
    tx[0] = reg;
    for (uint16_t i = 0; i < len; i++) {
        tx[1 + i] = buf[i];
    }
    return i2c_write(s_dev, tx, (size_t)(len + 1)) == ESP_OK ? 0 : 1;
}
