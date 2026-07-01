/* prop_imu — MPU-6500 eMD DMP driver (LibDriver core + bsp_i2c link).
 *
 * The installed IMU reports WHO_AM_I=0x70 (MPU-6500), not the MPU-6050 (0x68).
 * The MPU-6050 DMP firmware is incompatible with it, so this wraps LibDriver's
 * mpu6500 eMD core (vendored unmodified in components/mpu6500). The adapter owns
 * the singleton lifecycle, a 40 ms FIFO poll task, a mutex-protected cache, and
 * the public API. Tap/orient arrive via callbacks invoked inside mpu6500_dmp_read.
 * Motion and free-fall are derived in software from accel magnitude (avoids the
 * wake-on-motion low-power mode, which conflicts with DMP gyro operation).
 *
 * Shared bsp_i2c bus (I2C_NUM_0, GPIO45/46). MPU-6500 @ 0x68 does not conflict
 * with the GT911 touch controller (0x14/0x5D). Config: accel ±2g (16384 LSB/g),
 * gyro ±2000 dps. The driver requires SPI link functions to be non-NULL even in
 * IIC mode, so harmless SPI stubs are provided. Non-fatal: if the sensor doesn't
 * ACK, online stays false and the prop runs without it.
 */
#include "prop_imu.h"
#include "prop_imu_iic.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver_mpu6500.h"

#define TAG "IMU"

static mpu6500_handle_t  s_handle;
static prop_imu_data_t   s_data;
static SemaphoreHandle_t s_mutex;

/* Latest gesture/event, latched until read. */
static prop_imu_tap_t    s_tap;
static prop_imu_event_t  s_event;

/* Gyro bias currently applied to the DMP (converted "offset" units, see
 * mpu6500_dmp_gyro_accel_raw_offset_convert). Tracked here because the DMP
 * has no getter — imu_task's continuous rest re-zero adds to this and
 * re-pushes the running total via mpu6500_dmp_set_gyro_bias. */
static int32_t s_gyro_bias_off[3];

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

/* SPI link stubs — unused (IIC interface) but must be non-NULL for mpu6500_init. */
static uint8_t spi_stub_init(void)   { return 1; }
static uint8_t spi_stub_deinit(void) { return 1; }
static uint8_t spi_stub_read(uint8_t reg, uint8_t *buf, uint16_t len)  { (void)reg; (void)buf; (void)len; return 1; }
static uint8_t spi_stub_write(uint8_t reg, uint8_t *buf, uint16_t len) { (void)reg; (void)buf; (void)len; return 1; }

/* ── DMP callbacks (called from mpu6500_dmp_read, i.e. the poll task) ─────── */
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

    DRIVER_MPU6500_LINK_INIT(&s_handle, mpu6500_handle_t);
    DRIVER_MPU6500_LINK_IIC_INIT(&s_handle, prop_imu_iic_init);
    DRIVER_MPU6500_LINK_IIC_DEINIT(&s_handle, prop_imu_iic_deinit);
    DRIVER_MPU6500_LINK_IIC_READ(&s_handle, prop_imu_iic_read);
    DRIVER_MPU6500_LINK_IIC_WRITE(&s_handle, prop_imu_iic_write);
    DRIVER_MPU6500_LINK_SPI_INIT(&s_handle, spi_stub_init);
    DRIVER_MPU6500_LINK_SPI_DEINIT(&s_handle, spi_stub_deinit);
    DRIVER_MPU6500_LINK_SPI_READ(&s_handle, spi_stub_read);
    DRIVER_MPU6500_LINK_SPI_WRITE(&s_handle, spi_stub_write);
    DRIVER_MPU6500_LINK_DELAY_MS(&s_handle, imu_delay_ms);
    DRIVER_MPU6500_LINK_DEBUG_PRINT(&s_handle, imu_debug_print);
    DRIVER_MPU6500_LINK_RECEIVE_CALLBACK(&s_handle, recv_cb);

    if (mpu6500_set_interface(&s_handle, MPU6500_INTERFACE_IIC) != 0) return -1;
    if (mpu6500_set_addr_pin(&s_handle, MPU6500_ADDRESS_AD0_LOW) != 0) return -1;
    if (mpu6500_init(&s_handle) != 0) return -1;
    imu_delay_ms(100);
    if (mpu6500_set_sleep(&s_handle, MPU6500_BOOL_FALSE) != 0) goto fail;
    if (mpu6500_set_fifo_1024kb(&s_handle) != 0) goto fail;

    /* self-test → bias (non-fatal: continue without bias if it fails) */
    bool have_bias = (mpu6500_self_test(&s_handle, gyro_off_raw, accel_off_raw) == 0);
    if (!have_bias) ESP_LOGW(TAG, "self-test failed - skipping bias");

    if (mpu6500_set_fifo_1024kb(&s_handle) != 0) goto fail;
    if (mpu6500_set_clock_source(&s_handle, MPU6500_CLOCK_SOURCE_PLL) != 0) goto fail;
    if (mpu6500_set_sample_rate_divider(&s_handle, (1000 / 50) - 1) != 0) goto fail;   /* 50 Hz */
    if (mpu6500_set_accelerometer_range(&s_handle, MPU6500_ACCELEROMETER_RANGE_2G) != 0) goto fail;
    if (mpu6500_set_gyroscope_range(&s_handle, MPU6500_GYROSCOPE_RANGE_2000DPS) != 0) goto fail;
    if (mpu6500_set_low_pass_filter(&s_handle, MPU6500_LOW_PASS_FILTER_3) != 0) goto fail;

    /* wake all six axes */
    const mpu6500_source_t axes[6] = {
        MPU6500_SOURCE_ACC_X, MPU6500_SOURCE_ACC_Y, MPU6500_SOURCE_ACC_Z,
        MPU6500_SOURCE_GYRO_X, MPU6500_SOURCE_GYRO_Y, MPU6500_SOURCE_GYRO_Z
    };
    for (int i = 0; i < 6; i++) {
        if (mpu6500_set_standby_mode(&s_handle, axes[i], MPU6500_BOOL_FALSE) != 0) goto fail;
    }

    /* FIFO + interrupts (motion/free-fall are derived in software, see poll task) */
    if (mpu6500_set_fifo(&s_handle, MPU6500_BOOL_TRUE) != 0) goto fail;
    if (mpu6500_set_interrupt_level(&s_handle, MPU6500_PIN_LEVEL_LOW) != 0) goto fail;
    if (mpu6500_set_interrupt_pin_type(&s_handle, MPU6500_PIN_TYPE_PUSH_PULL) != 0) goto fail;
    mpu6500_set_interrupt(&s_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, MPU6500_BOOL_TRUE);
    mpu6500_set_interrupt_latch(&s_handle, MPU6500_BOOL_TRUE);
    mpu6500_set_interrupt_read_clear(&s_handle, MPU6500_BOOL_TRUE);

    /* DMP firmware + features */
    if (mpu6500_dmp_load_firmware(&s_handle) != 0) goto fail;

    /* enable tap on all three axes (AXIS_X=7, AXIS_Y=6, AXIS_Z=5) */
    mpu6500_dmp_set_tap_axes(&s_handle, MPU6500_AXIS_X, MPU6500_BOOL_TRUE);
    mpu6500_dmp_set_tap_axes(&s_handle, MPU6500_AXIS_Y, MPU6500_BOOL_TRUE);
    mpu6500_dmp_set_tap_axes(&s_handle, MPU6500_AXIS_Z, MPU6500_BOOL_TRUE);

    mpu6500_dmp_set_fifo_rate(&s_handle, 50);
    mpu6500_dmp_set_interrupt_mode(&s_handle, MPU6500_DMP_INTERRUPT_MODE_CONTINUOUS);
    mpu6500_dmp_set_orientation(&s_handle, orient);
    if (mpu6500_dmp_set_feature(&s_handle,
            MPU6500_DMP_FEATURE_6X_QUAT | MPU6500_DMP_FEATURE_TAP |
            MPU6500_DMP_FEATURE_PEDOMETER | MPU6500_DMP_FEATURE_ORIENT |
            MPU6500_DMP_FEATURE_SEND_RAW_ACCEL | MPU6500_DMP_FEATURE_SEND_CAL_GYRO |
            MPU6500_DMP_FEATURE_GYRO_CAL) != 0) goto fail;
    mpu6500_dmp_set_tap_callback(&s_handle, tap_cb);
    mpu6500_dmp_set_orient_callback(&s_handle, orient_cb);
    mpu6500_dmp_set_min_tap_count(&s_handle, 1);   /* report all taps; engine gates on >=2 */

    /* apply self-test offsets as DMP bias (only if self-test produced them) */
    if (have_bias &&
        mpu6500_dmp_gyro_accel_raw_offset_convert(&s_handle, gyro_off_raw, accel_off_raw,
                                                  gyro_off, accel_off) == 0) {
        mpu6500_dmp_set_accel_bias(&s_handle, accel_off);
        mpu6500_dmp_set_gyro_bias(&s_handle, gyro_off);
        s_gyro_bias_off[0] = gyro_off[0]; s_gyro_bias_off[1] = gyro_off[1]; s_gyro_bias_off[2] = gyro_off[2];
    }

    if (mpu6500_dmp_set_enable(&s_handle, MPU6500_BOOL_TRUE) != 0) goto fail;
    mpu6500_force_fifo_reset(&s_handle);

    /* Fallback when the vendor self-test fails (observed: accel self-test
     * consistently fails on this unit, so no gyro bias ever gets applied).
     * Without a bias, the DMP integrates yaw from the raw factory offset and
     * it free-spins continuously even sitting still. Measure our own
     * zero-rate gyro bias here (board assumed stationary at boot — same
     * assumption the self-test makes) and feed it through the same DMP bias
     * path used for the self-test-derived offset. This is only a seed — a
     * ~250ms average is too short/noisy to fully null the bias, so imu_task
     * keeps refining it via continuous rest-triggered re-zero below. */
    if (!have_bias) {
        int16_t  cal_accel_raw[2][3], cal_gyro_raw[2][3];
        float    cal_accel_g[2][3], cal_gyro_dps[2][3];
        int32_t  cal_quat[2][4];
        float    cal_pitch, cal_roll, cal_yaw;
        uint16_t cal_len;
        int64_t  gsum[3] = { 0, 0, 0 };
        int      gcount  = 0;

        for (int i = 0; i < 40 && gcount < 20; i++) {
            imu_delay_ms(20);
            cal_len = 2;
            if (mpu6500_dmp_read(&s_handle, cal_accel_raw, cal_accel_g, cal_gyro_raw, cal_gyro_dps,
                                 cal_quat, &cal_pitch, &cal_roll, &cal_yaw, &cal_len) != 0 || cal_len == 0) {
                continue;
            }
            gsum[0] += cal_gyro_raw[0][0];
            gsum[1] += cal_gyro_raw[0][1];
            gsum[2] += cal_gyro_raw[0][2];
            gcount++;
        }
        if (gcount >= 10) {
            int32_t gyro_off_raw2[3]  = { (int32_t)(gsum[0] / gcount), (int32_t)(gsum[1] / gcount),
                                          (int32_t)(gsum[2] / gcount) };
            int32_t accel_off_raw2[3] = { 0, 0, 0 };   /* leave accel bias untouched */
            int32_t gyro_off2[3], accel_off2[3];
            if (mpu6500_dmp_gyro_accel_raw_offset_convert(&s_handle, gyro_off_raw2, accel_off_raw2,
                                                          gyro_off2, accel_off2) == 0) {
                mpu6500_dmp_set_gyro_bias(&s_handle, gyro_off2);
                s_gyro_bias_off[0] = gyro_off2[0]; s_gyro_bias_off[1] = gyro_off2[1]; s_gyro_bias_off[2] = gyro_off2[2];
                ESP_LOGI(TAG, "applied measured gyro bias (raw %ld,%ld,%ld, n=%d)",
                         (long)gyro_off_raw2[0], (long)gyro_off_raw2[1], (long)gyro_off_raw2[2], gcount);
            } else {
                ESP_LOGW(TAG, "gyro bias conversion failed - yaw may drift");
            }
        } else {
            ESP_LOGW(TAG, "gyro bias measurement got too few samples (%d) - yaw may drift", gcount);
        }
        mpu6500_force_fifo_reset(&s_handle);
    }
    return 0;

fail:
    (void)mpu6500_deinit(&s_handle);
    return -1;
}

/* ── Poll task (40 ms, no INT pin) ───────────────────────────────────────── */
static void imu_task(void *arg)
{
    (void)arg;
    int16_t  accel_raw[2][3], gyro_raw[2][3];
    float    accel_g[2][3], gyro_dps[2][3];
    int32_t  quat[2][4];
    float    pitch, roll, yaw;
    uint16_t len;
    int      ff_run_ms = 0;
    int      mv_run_ms = 0;

    /* Continuous rest-triggered gyro re-zero: the DMP's own GYRO_CAL feature
     * and the one-shot boot bias (see dmp_bringup) aren't enough to fully
     * null the yaw drift on this unit, and bias shifts as the chip warms up.
     * Whenever the board is genuinely still (low accel deviation from 1g AND
     * low reported angular rate) for a sustained window, average the
     * "calibrated" gyro raw counts — any nonzero average at true zero rate
     * is residual bias — and fold it into the DMP bias as an increment. */
    int64_t  rest_gsum[3] = { 0, 0, 0 };
    int      rest_count = 0;
    int      rezero_cooldown_ms = 0;
#define IMU_REZERO_WINDOW_SAMPLES 50    /* ~2s of stillness at 40ms/sample */
#define IMU_REZERO_MIN_RAW        2     /* skip sub-noise-floor residuals (~0.12 dps) */
#define IMU_REZERO_COOLDOWN_MS    4000

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(40));

        len = 2;
        if (mpu6500_dmp_read(&s_handle, accel_raw, accel_g, gyro_raw, gyro_dps,
                             quat, &pitch, &roll, &yaw, &len) != 0) {
            continue;
        }
        if (len == 0) continue;

        /* die temp + pedometer */
        float   temp_c = 0.0f;
        int16_t traw = 0;
        mpu6500_read_temperature(&s_handle, &traw, &temp_c);
        uint32_t steps = 0;
        mpu6500_dmp_get_pedometer_step_count(&s_handle, &steps);

        /* motion + free-fall derived from accel magnitude (g). */
        float amag = sqrtf(accel_g[0][0] * accel_g[0][0] +
                           accel_g[0][1] * accel_g[0][1] +
                           accel_g[0][2] * accel_g[0][2]);
        if (amag < 0.4f) ff_run_ms += 40; else ff_run_ms = 0;
        bool freefall = ff_run_ms >= 80;
        if (fabsf(amag - 1.0f) > 0.25f) mv_run_ms += 40; else mv_run_ms = 0;
        bool motion = mv_run_ms >= 60 && !freefall;

        if (rezero_cooldown_ms > 0) rezero_cooldown_ms -= 40;

        float gmag = sqrtf(gyro_dps[0][0] * gyro_dps[0][0] +
                           gyro_dps[0][1] * gyro_dps[0][1] +
                           gyro_dps[0][2] * gyro_dps[0][2]);
        bool likely_still = !motion && !freefall && gmag < 2.0f;
        if (likely_still) {
            rest_gsum[0] += gyro_raw[0][0];
            rest_gsum[1] += gyro_raw[0][1];
            rest_gsum[2] += gyro_raw[0][2];
            rest_count++;
        } else {
            rest_gsum[0] = rest_gsum[1] = rest_gsum[2] = 0;
            rest_count = 0;
        }
        if (rest_count >= IMU_REZERO_WINDOW_SAMPLES && rezero_cooldown_ms <= 0) {
            int32_t resid_raw[3] = { (int32_t)(rest_gsum[0] / rest_count),
                                     (int32_t)(rest_gsum[1] / rest_count),
                                     (int32_t)(rest_gsum[2] / rest_count) };
            if (abs(resid_raw[0]) >= IMU_REZERO_MIN_RAW || abs(resid_raw[1]) >= IMU_REZERO_MIN_RAW ||
                abs(resid_raw[2]) >= IMU_REZERO_MIN_RAW) {
                int32_t accel_dummy[3] = { 0, 0, 0 }, resid_off[3], accel_off_dummy[3];
                if (mpu6500_dmp_gyro_accel_raw_offset_convert(&s_handle, resid_raw, accel_dummy,
                                                              resid_off, accel_off_dummy) == 0) {
                    s_gyro_bias_off[0] += resid_off[0];
                    s_gyro_bias_off[1] += resid_off[1];
                    s_gyro_bias_off[2] += resid_off[2];
                    mpu6500_dmp_set_gyro_bias(&s_handle, s_gyro_bias_off);
                    ESP_LOGI(TAG, "gyro rezero (resid raw %ld,%ld,%ld)",
                             (long)resid_raw[0], (long)resid_raw[1], (long)resid_raw[2]);
                }
            }
            rest_gsum[0] = rest_gsum[1] = rest_gsum[2] = 0;
            rest_count = 0;
            rezero_cooldown_ms = IMU_REZERO_COOLDOWN_MS;
        }

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
    if (s_mutex) {                       /* already initialized — ignore repeat calls */
        ESP_LOGW(TAG, "prop_imu_init called more than once — ignoring");
        return s_data.online ? ESP_OK : ESP_ERR_NOT_FOUND;
    }
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    s_event = PROP_IMU_EVT_NONE;
    s_tap.present = false;

    if (dmp_bringup() != 0) {
        ESP_LOGW(TAG, "MPU-6500 eMD bring-up failed (not wired / power off)");
        s_data.online = false;
        return ESP_ERR_NOT_FOUND;
    }
    s_data.online = true;
    ESP_LOGI(TAG, "MPU-6500 eMD DMP ready");
    xTaskCreatePinnedToCore(imu_task, "imu_dmp", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}

void prop_imu_get_data(prop_imu_data_t *out)
{
    if (!out) return;
    if (!s_mutex) { memset(out, 0, sizeof(*out)); return; }   /* before init / never inited */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_data;
    xSemaphoreGive(s_mutex);
}

bool prop_imu_available(void)
{
    if (!s_mutex) return false;
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

prop_imu_tap_t prop_imu_get_tap(void)
{
    prop_imu_tap_t t = { 0 };
    if (!s_mutex) return t;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    t = s_tap;
    s_tap.present = false;
    xSemaphoreGive(s_mutex);
    return t;
}

prop_imu_event_t prop_imu_get_motion_event(void)
{
    if (!s_mutex) return PROP_IMU_EVT_NONE;
    prop_imu_event_t e;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    e = s_event;
    s_event = PROP_IMU_EVT_NONE;
    xSemaphoreGive(s_mutex);
    return e;
}
