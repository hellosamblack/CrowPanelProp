/* prop_imu — MPU-6050 eMD DMP driver (LibDriver core + bsp_i2c link).
 *
 * Source: LibDriver mpu6050 (MIT), vendored unmodified in components/mpu6050.
 * This adapter owns the singleton lifecycle, a 20 ms FIFO poll task, a
 * mutex-protected cache, and the public API. Tap/orient arrive via callbacks
 * invoked inside mpu6050_dmp_read's FIFO walk. Free-fall is derived in software
 * from low-g accel magnitude (LibDriver does not expose the FF interrupt).
 *
 * Shared bsp_i2c bus (I2C_NUM_0, GPIO45/46). MPU-6050 @ 0x68 does not conflict
 * with the GT911 touch controller (0x14/0x5D). Config: accel ±2g (16384 LSB/g),
 * gyro ±2000 dps. Non-fatal: if the sensor doesn't ACK, online stays false.
 */
#include "prop_imu.h"
#include "prop_imu_iic.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver_mpu6050.h"

#define TAG "IMU"

static mpu6050_handle_t  s_handle;
static prop_imu_data_t   s_data;
static SemaphoreHandle_t s_mutex;

/* Latest gesture/event, latched until read. */
static prop_imu_tap_t    s_tap;
static prop_imu_event_t  s_event;

/* ── LibDriver link helpers ──────────────────────────────────────────────── */
static void imu_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
}

static void imu_debug_print(const char *const fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

/* ── DMP callbacks (called from mpu6050_dmp_read, i.e. the poll task) ─────── */
static void tap_cb(uint8_t count, uint8_t direction)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_tap.count = count;
    s_tap.direction = direction;
    s_tap.present = true;
    xSemaphoreGive(s_mutex);
}
static void orient_cb(uint8_t orientation) { (void)orientation; /* captured, not acted on */ }
static void recv_cb(uint8_t type)          { (void)type; }

/* ── Init ─────────────────────────────────────────────────────────────────── */
static int dmp_bringup(void)
{
    int32_t gyro_off_raw[3], accel_off_raw[3], gyro_off[3], accel_off[3];
    int8_t  orient[9] = { 1, 0, 0,
                          0, 1, 0,
                          0, 0, 1 };

    DRIVER_MPU6050_LINK_INIT(&s_handle, mpu6050_handle_t);
    DRIVER_MPU6050_LINK_IIC_INIT(&s_handle, prop_imu_iic_init);
    DRIVER_MPU6050_LINK_IIC_DEINIT(&s_handle, prop_imu_iic_deinit);
    DRIVER_MPU6050_LINK_IIC_READ(&s_handle, prop_imu_iic_read);
    DRIVER_MPU6050_LINK_IIC_WRITE(&s_handle, prop_imu_iic_write);
    DRIVER_MPU6050_LINK_DELAY_MS(&s_handle, imu_delay_ms);
    DRIVER_MPU6050_LINK_DEBUG_PRINT(&s_handle, imu_debug_print);
    DRIVER_MPU6050_LINK_RECEIVE_CALLBACK(&s_handle, recv_cb);

    if (mpu6050_set_addr_pin(&s_handle, MPU6050_ADDRESS_AD0_LOW) != 0) return -1;
    if (mpu6050_init(&s_handle) != 0) return -1;
    imu_delay_ms(100);
    if (mpu6050_set_sleep(&s_handle, MPU6050_BOOL_FALSE) != 0) goto fail;

    /* self-test → bias (improves orientation, reduces drift) */
    if (mpu6050_self_test(&s_handle, gyro_off_raw, accel_off_raw) != 0) goto fail;

    if (mpu6050_set_clock_source(&s_handle, MPU6050_CLOCK_SOURCE_PLL_X_GYRO) != 0) goto fail;
    if (mpu6050_set_sample_rate_divider(&s_handle, (1000 / 50) - 1) != 0) goto fail;   /* 50 Hz */
    if (mpu6050_set_accelerometer_range(&s_handle, MPU6050_ACCELEROMETER_RANGE_2G) != 0) goto fail;
    if (mpu6050_set_gyroscope_range(&s_handle, MPU6050_GYROSCOPE_RANGE_2000DPS) != 0) goto fail;
    if (mpu6050_set_low_pass_filter(&s_handle, MPU6050_LOW_PASS_FILTER_3) != 0) goto fail;
    if (mpu6050_set_temperature_sensor(&s_handle, MPU6050_BOOL_TRUE) != 0) goto fail;

    /* wake all six axes */
    const mpu6050_source_t axes[6] = {
        MPU6050_SOURCE_ACC_X, MPU6050_SOURCE_ACC_Y, MPU6050_SOURCE_ACC_Z,
        MPU6050_SOURCE_GYRO_X, MPU6050_SOURCE_GYRO_Y, MPU6050_SOURCE_GYRO_Z
    };
    for (int i = 0; i < 6; i++) {
        if (mpu6050_set_standby_mode(&s_handle, axes[i], MPU6050_BOOL_FALSE) != 0) goto fail;
    }

    /* FIFO + interrupts (motion enabled for the MOTION event; no INT GPIO — we
     * poll INT_STATUS in the task). */
    if (mpu6050_set_fifo(&s_handle, MPU6050_BOOL_TRUE) != 0) goto fail;
    if (mpu6050_set_interrupt_level(&s_handle, MPU6050_PIN_LEVEL_LOW) != 0) goto fail;
    if (mpu6050_set_interrupt_pin_type(&s_handle, MPU6050_PIN_TYPE_PUSH_PULL) != 0) goto fail;
    {
        uint8_t reg;
        if (mpu6050_motion_threshold_convert_to_register(&s_handle, 200, &reg) == 0) {
            mpu6050_set_motion_threshold(&s_handle, reg);
        }
        if (mpu6050_motion_duration_convert_to_register(&s_handle, 200, &reg) == 0) {
            mpu6050_set_motion_duration(&s_handle, reg);
        }
    }
    mpu6050_set_force_accel_sample(&s_handle, MPU6050_BOOL_TRUE);
    mpu6050_set_interrupt(&s_handle, MPU6050_INTERRUPT_MOTION, MPU6050_BOOL_TRUE);
    mpu6050_set_interrupt(&s_handle, MPU6050_INTERRUPT_FIFO_OVERFLOW, MPU6050_BOOL_TRUE);
    mpu6050_set_interrupt_latch(&s_handle, MPU6050_BOOL_TRUE);
    mpu6050_set_interrupt_read_clear(&s_handle, MPU6050_BOOL_TRUE);

    /* DMP firmware + features */
    if (mpu6050_dmp_load_firmware(&s_handle) != 0) goto fail;

    /* enable tap on all three axes (note: AXIS_X=7, AXIS_Y=6, AXIS_Z=5) */
    mpu6050_dmp_set_tap_axes(&s_handle, MPU6050_AXIS_X, MPU6050_BOOL_TRUE);
    mpu6050_dmp_set_tap_axes(&s_handle, MPU6050_AXIS_Y, MPU6050_BOOL_TRUE);
    mpu6050_dmp_set_tap_axes(&s_handle, MPU6050_AXIS_Z, MPU6050_BOOL_TRUE);

    mpu6050_dmp_set_fifo_rate(&s_handle, 50);
    mpu6050_dmp_set_interrupt_mode(&s_handle, MPU6050_DMP_INTERRUPT_MODE_CONTINUOUS);
    mpu6050_dmp_set_orientation(&s_handle, orient);
    if (mpu6050_dmp_set_feature(&s_handle,
            MPU6050_DMP_FEATURE_6X_QUAT | MPU6050_DMP_FEATURE_TAP |
            MPU6050_DMP_FEATURE_PEDOMETER | MPU6050_DMP_FEATURE_ORIENT |
            MPU6050_DMP_FEATURE_SEND_RAW_ACCEL | MPU6050_DMP_FEATURE_SEND_CAL_GYRO |
            MPU6050_DMP_FEATURE_GYRO_CAL) != 0) goto fail;
    mpu6050_dmp_set_tap_callback(&s_handle, tap_cb);
    mpu6050_dmp_set_orient_callback(&s_handle, orient_cb);

    /* report every tap (engine gates the contact-ping on count >= 2) */
    mpu6050_dmp_set_min_tap_count(&s_handle, 1);

    /* apply self-test offsets as DMP bias */
    if (mpu6050_dmp_gyro_accel_raw_offset_convert(&s_handle, gyro_off_raw, accel_off_raw,
                                                  gyro_off, accel_off) == 0) {
        mpu6050_dmp_set_accel_bias(&s_handle, accel_off);
        mpu6050_dmp_set_gyro_bias(&s_handle, gyro_off);
    }

    if (mpu6050_dmp_set_enable(&s_handle, MPU6050_BOOL_TRUE) != 0) goto fail;
    mpu6050_force_fifo_reset(&s_handle);
    return 0;

fail:
    (void)mpu6050_deinit(&s_handle);
    return -1;
}

/* ── Poll task (20 ms, no INT pin) ───────────────────────────────────────── */
static void imu_task(void *arg)
{
    (void)arg;
    int16_t  accel_raw[1][3], gyro_raw[1][3];
    float    accel_g[1][3], gyro_dps[1][3];
    int32_t  quat[1][4];
    float    pitch, roll, yaw;
    uint16_t len;
    int      ff_run_ms = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));

        len = 1;
        if (mpu6050_dmp_read(&s_handle, accel_raw, accel_g, gyro_raw, gyro_dps,
                             quat, &pitch, &roll, &yaw, &len) != 0) {
            continue;
        }
        if (len == 0) continue;

        /* die temp + pedometer */
        float   temp_c = 0.0f;
        int16_t traw = 0;
        mpu6050_read_temperature(&s_handle, &traw, &temp_c);
        uint32_t steps = 0;
        mpu6050_dmp_get_pedometer_step_count(&s_handle, &steps);

        /* motion interrupt status (bit MPU6050_INTERRUPT_MOTION = 6) */
        uint8_t istat = 0;
        mpu6050_get_interrupt_status(&s_handle, &istat);
        bool motion = (istat & (1u << MPU6050_INTERRUPT_MOTION)) != 0;

        /* free-fall: accel magnitude near 0 g sustained ~80 ms */
        float amag = sqrtf(accel_g[0][0] * accel_g[0][0] +
                           accel_g[0][1] * accel_g[0][1] +
                           accel_g[0][2] * accel_g[0][2]);
        if (amag < 0.4f) ff_run_ms += 20; else ff_run_ms = 0;
        bool freefall = ff_run_ms >= 80;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        bool was_online = s_data.online;
        s_data.qw = (float)quat[0][0] / 1073741824.0f;   /* Q30 → float */
        s_data.qx = (float)quat[0][1] / 1073741824.0f;
        s_data.qy = (float)quat[0][2] / 1073741824.0f;
        s_data.qz = (float)quat[0][3] / 1073741824.0f;
        s_data.pitch = pitch * (float)M_PI / 180.0f;      /* store radians (API contract) */
        s_data.roll  = roll  * (float)M_PI / 180.0f;
        s_data.yaw   = yaw   * (float)M_PI / 180.0f;
        s_data.ax = accel_raw[0][0]; s_data.ay = accel_raw[0][1]; s_data.az = accel_raw[0][2];
        s_data.gx = gyro_raw[0][0];  s_data.gy = gyro_raw[0][1];  s_data.gz = gyro_raw[0][2];
        s_data.temp_c = temp_c;
        s_data.step_count = steps;
        s_data.valid = true;
        s_data.online = was_online;
        if (freefall)    s_event = PROP_IMU_EVT_FREEFALL;
        else if (motion) s_event = PROP_IMU_EVT_MOTION;
        xSemaphoreGive(s_mutex);
    }
}

/* ── Public API ───────────────────────────────────────────────────────────── */
esp_err_t prop_imu_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    s_event = PROP_IMU_EVT_NONE;
    s_tap.present = false;

    if (dmp_bringup() != 0) {
        ESP_LOGW(TAG, "MPU-6050 eMD bring-up failed (not wired / power off)");
        s_data.online = false;
        return ESP_ERR_NOT_FOUND;
    }
    s_data.online = true;
    ESP_LOGI(TAG, "MPU-6050 eMD DMP ready");
    xTaskCreate(imu_task, "imu_dmp", 4096, NULL, 5, NULL);
    return ESP_OK;
}

void prop_imu_get_data(prop_imu_data_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_data;
    xSemaphoreGive(s_mutex);
}

bool prop_imu_available(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool online = s_data.online;
    xSemaphoreGive(s_mutex);
    return online;
}

/* Pitch/roll/yaw in DEGREES (matches the original API contract).
 * Returns false until the DMP produces a valid packet. */
bool prop_imu_get_orientation(float *pitch, float *roll, float *yaw)
{
    prop_imu_data_t d;
    prop_imu_get_data(&d);
    if (!d.online || !d.valid) return false;
    const float r2d = 180.0f / (float)M_PI;
    if (pitch) *pitch = d.pitch * r2d;
    if (roll)  *roll  = d.roll  * r2d;
    if (yaw)   *yaw   = d.yaw   * r2d;
    return true;
}

/* Raw accel in milli-g [X,Y,Z]; calibrated gyro in milli-°/s [X,Y,Z].
 * accel ±2g (16384 LSB/g), gyro ±2000 dps (16.4 LSB/dps). */
void prop_imu_get_raw(int32_t accel_milli_g[3], int32_t gyro_milli_dps[3])
{
    prop_imu_data_t d;
    prop_imu_get_data(&d);
    if (accel_milli_g) {
        accel_milli_g[0] = (int32_t)d.ax * 1000 / 16384;
        accel_milli_g[1] = (int32_t)d.ay * 1000 / 16384;
        accel_milli_g[2] = (int32_t)d.az * 1000 / 16384;
    }
    if (gyro_milli_dps) {
        gyro_milli_dps[0] = (int32_t)d.gx * 1000000 / 16384;
        gyro_milli_dps[1] = (int32_t)d.gy * 1000000 / 16384;
        gyro_milli_dps[2] = (int32_t)d.gz * 1000000 / 16384;
    }
}

prop_imu_tap_t prop_imu_get_tap(void)
{
    prop_imu_tap_t t;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    t = s_tap;
    s_tap.present = false;
    xSemaphoreGive(s_mutex);
    return t;
}

prop_imu_event_t prop_imu_get_motion_event(void)
{
    prop_imu_event_t e;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    e = s_event;
    s_event = PROP_IMU_EVT_NONE;
    xSemaphoreGive(s_mutex);
    return e;
}
