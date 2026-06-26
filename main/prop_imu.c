/* prop_imu — InvenSense MPU-6050 6-axis IMU driver + complementary filter.
 * Sensor address 0x68 (AD0 low) on the I2C bus initialised by bsp_i2c.
 * See prop_imu.h for the public API and hardware/filter notes. */

#include "prop_imu.h"

#include <math.h>
#include <string.h>
#include "bsp_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "PROP_IMU"

/* ---- MPU-6050 register map (subset) ------------------------------------- */
#define MPU_ADDR          0x68   /* I2C address (AD0 low) */
#define REG_PWR_MGMT_1    0x6B   /* write 0x00 to wake */
#define REG_GYRO_CONFIG   0x1B   /* write 0x00 for ±250°/s */
#define REG_ACCEL_CONFIG  0x1C   /* write 0x00 for ±2g */
#define REG_ACCEL_XOUT_H  0x3B   /* 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L */
#define REG_GYRO_XOUT_H   0x43   /* 6 bytes: GX_H GX_L GY_H GY_L GZ_H GZ_L */

/* Scale factors for the chosen full-scale ranges. */
#define ACCEL_LSB_PER_G   16384.0f   /* ±2g  */
#define GYRO_LSB_PER_DPS    131.0f   /* ±250°/s */

/* Complementary filter constants. */
#define ALPHA             0.96f       /* gyro weight */
#define DT                0.020f      /* 20 ms loop period */

/* ---- Module-scope state ------------------------------------------------- */
static bool                   s_available;
static i2c_master_dev_handle_t s_dev;

/* Orientation (degrees), computed by the filter. */
static float s_pitch;
static float s_roll;
static float s_yaw;

/* Fixed-point cache: ×1000 integer so the spinlock critical section never
 * touches floats (avoids undefined behaviour on targets without hardware FP
 * in a critical section, and matches the public API contract). */
static int32_t s_accel_mg[3];    /* accel in milli-g   [X,Y,Z] */
static int32_t s_gyro_mdps[3];   /* gyro  in m°/s      [X,Y,Z] */

/* Guards all of the above shared state. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* True once the complementary filter has been seeded from the first accel
 * sample (so callers don't read un-initialised pitch/roll). */
static bool s_has_data;

/* ---- Helpers ------------------------------------------------------------ */

/* Combine two big-endian bytes into a signed 16-bit value. */
static inline int16_t be16(const uint8_t *b)
{
    return (int16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
}

/* ---- Background orientation task ---------------------------------------- */

static void imu_task(void *arg)
{
    (void)arg;

    uint8_t buf[6];

    for (;;) {
        /* --- Read raw accelerometer (6 bytes from 0x3B) --- */
        esp_err_t err = i2c_read_reg(s_dev, REG_ACCEL_XOUT_H, buf, 6);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "accel read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        int16_t raw_ax = be16(buf + 0);
        int16_t raw_ay = be16(buf + 2);
        int16_t raw_az = be16(buf + 4);

        /* --- Read raw gyroscope (6 bytes from 0x43) --- */
        err = i2c_read_reg(s_dev, REG_GYRO_XOUT_H, buf, 6);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "gyro read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        int16_t raw_gx = be16(buf + 0);
        int16_t raw_gy = be16(buf + 2);
        int16_t raw_gz = be16(buf + 4);

        /* --- Convert to physical units --- */
        float ax = (float)raw_ax / ACCEL_LSB_PER_G;   /* g */
        float ay = (float)raw_ay / ACCEL_LSB_PER_G;
        float az = (float)raw_az / ACCEL_LSB_PER_G;

        float gx_dps = (float)raw_gx / GYRO_LSB_PER_DPS;  /* °/s */
        float gy_dps = (float)raw_gy / GYRO_LSB_PER_DPS;
        float gz_dps = (float)raw_gz / GYRO_LSB_PER_DPS;

        /* --- Accel-derived pitch / roll (noisy but drift-free) --- */
        float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);
        float accel_roll  = atan2f( ay, az)                        * (180.0f / (float)M_PI);

        /* --- Complementary filter --- */
        float new_pitch, new_roll, new_yaw;

        portENTER_CRITICAL(&s_mux);
        bool seeded = s_has_data;
        float prev_pitch = s_pitch;
        float prev_roll  = s_roll;
        float prev_yaw   = s_yaw;
        portEXIT_CRITICAL(&s_mux);

        if (!seeded) {
            /* First sample: seed from accel; yaw starts at 0. */
            new_pitch = accel_pitch;
            new_roll  = accel_roll;
            new_yaw   = 0.0f;
        } else {
            new_pitch = ALPHA * (prev_pitch + gx_dps * DT) + (1.0f - ALPHA) * accel_pitch;
            new_roll  = ALPHA * (prev_roll  + gy_dps * DT) + (1.0f - ALPHA) * accel_roll;
            new_yaw   = prev_yaw + gz_dps * DT;   /* gyro-only; will drift */
        }

        /* Clamp yaw to [−180, 180] to keep display values sane. */
        if (new_yaw >  180.0f) new_yaw -= 360.0f;
        if (new_yaw < -180.0f) new_yaw += 360.0f;

        /* --- Commit to shared cache (fixed-point integers only under lock) --- */
        int32_t accel_mg[3]  = {
            (int32_t)(ax     * 1000.0f),
            (int32_t)(ay     * 1000.0f),
            (int32_t)(az     * 1000.0f),
        };
        int32_t gyro_mdps[3] = {
            (int32_t)(gx_dps * 1000.0f),
            (int32_t)(gy_dps * 1000.0f),
            (int32_t)(gz_dps * 1000.0f),
        };

        /* Store pitch/roll/yaw as fixed-point ×1000 integers to keep
         * float ops out of the critical section. */
        int32_t pitch_milli = (int32_t)(new_pitch * 1000.0f);
        int32_t roll_milli  = (int32_t)(new_roll  * 1000.0f);
        int32_t yaw_milli   = (int32_t)(new_yaw   * 1000.0f);

        portENTER_CRITICAL(&s_mux);
        memcpy(s_accel_mg,  accel_mg,  sizeof(s_accel_mg));
        memcpy(s_gyro_mdps, gyro_mdps, sizeof(s_gyro_mdps));
        /* Reconstruct floats from the integers so the stored values
         * are consistent with what the getter will return. */
        s_pitch    = (float)pitch_milli * 0.001f;
        s_roll     = (float)roll_milli  * 0.001f;
        s_yaw      = (float)yaw_milli   * 0.001f;
        s_has_data = true;
        portEXIT_CRITICAL(&s_mux);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---- Public API ---------------------------------------------------------- */

esp_err_t prop_imu_init(void)
{
    /* Register the MPU-6050 device on the shared I2C bus. */
    s_dev = i2c_dev_register(MPU_ADDR);
    if (s_dev == NULL) {
        ESP_LOGE(TAG, "i2c_dev_register(0x%02X) failed — no handle returned", MPU_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    /* Wake the device (PWR_MGMT_1 = 0x00). On power-up it is in sleep mode. */
    esp_err_t err = i2c_write_reg(s_dev, REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wake write failed — sensor not ACKing (0x%02X): %s",
                 MPU_ADDR, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }

    /* Gyro: ±250°/s full scale. */
    err = i2c_write_reg(s_dev, REG_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GYRO_CONFIG write failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Accel: ±2g full scale. */
    err = i2c_write_reg(s_dev, REG_ACCEL_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ACCEL_CONFIG write failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Start the background orientation task. */
    BaseType_t r = xTaskCreatePinnedToCore(imu_task, "prop_imu",
                                           3072, NULL, 3, NULL, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        return ESP_ERR_NO_MEM;
    }

    s_available = true;
    ESP_LOGI(TAG, "MPU-6050 ok, addr 0x68");
    return ESP_OK;
}

bool prop_imu_available(void)
{
    return s_available;
}

bool prop_imu_get_orientation(float *pitch, float *roll, float *yaw)
{
    if (!s_available) {
        return false;
    }

    portENTER_CRITICAL(&s_mux);
    bool ready = s_has_data;
    float p = s_pitch;
    float r = s_roll;
    float y = s_yaw;
    portEXIT_CRITICAL(&s_mux);

    if (!ready) {
        return false;
    }
    if (pitch) *pitch = p;
    if (roll)  *roll  = r;
    if (yaw)   *yaw   = y;
    return true;
}

void prop_imu_get_raw(int32_t accel_milli_g[3], int32_t gyro_milli_dps[3])
{
    portENTER_CRITICAL(&s_mux);
    if (accel_milli_g)  memcpy(accel_milli_g,  s_accel_mg,  3 * sizeof(int32_t));
    if (gyro_milli_dps) memcpy(gyro_milli_dps, s_gyro_mdps, 3 * sizeof(int32_t));
    portEXIT_CRITICAL(&s_mux);
}
