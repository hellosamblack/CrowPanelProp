/* Communicator/scanner prop — init orchestration.
 *
 * Hardware bring-up order mirrors Lesson09 (LDO -> I2C -> touch -> display ->
 * backlight), then layers the prop on top: physical I/O, the engine (brain),
 * the screen UI, WiFi, and the live control API.
 */
#include "main.h"
#include "prop_motion.h"
#include "prop_imu.h"
#include "prop_track.h"
#include "prop_aux_radar.h"
#include "prop_lidar.h"
#include "esp_ota_ops.h"
static esp_ldo_channel_handle_t ldo3;
static esp_ldo_channel_handle_t ldo4;

/* Physical buttons drive the console navigation (the author's dial/button model).
 * Only two GPIO buttons are wired today, so they stand in for the SELECTOR dial:
 * MODE rotates the function rail, ACTION presses (opens / steps in). The full
 * knob+switch set routes through prop_ui_input() the same way once wired. */
static void on_button(prop_button_t button, prop_button_event_t event, void *ctx)
{
    (void)ctx;
    if (event != BTN_EVENT_PRESS) {
        return;  /* act on press; long-press/release reserved for future use */
    }
    switch (button) {
        case BTN_MODE:
            prop_ui_input("selector", 1);    /* rotate the rail */
            break;
        case BTN_ACTION:
            prop_ui_input("selector", 0);    /* press: open the highlighted function */
            break;
        default:
            break;
    }
}

static void fail_loop(const char *what, esp_err_t err)
{
    while (1) {
        MAIN_ERROR("[%s] init failed: %s", what, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void hardware_init(void)
{
    esp_err_t err;

    /* 1. LDOs powering the MIPI-DSI panel (required before display). */
    prop_bootlog_mark(BOOT_STAGE_HW_LDO);
    esp_ldo_channel_config_t ldo3_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    err = esp_ldo_acquire_channel(&ldo3_cfg, &ldo3);
    if (err != ESP_OK) fail_loop("ldo3", err);
    esp_ldo_channel_config_t ldo4_cfg = { .chan_id = 4, .voltage_mv = 3300 };
    err = esp_ldo_acquire_channel(&ldo4_cfg, &ldo4);
    if (err != ESP_OK) fail_loop("ldo4", err);

    /* 2. I2C (touch bus). */
    prop_bootlog_mark(BOOT_STAGE_HW_I2C);
    err = i2c_init();
    if (err != ESP_OK) fail_loop("i2c", err);

    /* 3. Touch panel. */
    prop_bootlog_mark(BOOT_STAGE_HW_TOUCH);
    err = touch_init();
    if (err != ESP_OK) fail_loop("touch", err);

    /* 4. LCD + LVGL. */
    prop_bootlog_mark(BOOT_STAGE_HW_DISPLAY);
    err = display_init();
    if (err != ESP_OK) fail_loop("display", err);

    /* 5. Backlight on. */
    prop_bootlog_mark(BOOT_STAGE_HW_BACKLIGHT);
    err = set_lcd_blight(100);
    if (err != ESP_OK) fail_loop("backlight", err);

    MAIN_INFO("display stack up");
}

void app_main(void)
{
    /* First thing, before anything else can fault — reads back the previous
     * session's breadcrumb (if any) and logs it. See prop_bootlog.h. */
    prop_bootlog_init();

    MAIN_INFO("communicator prop starting");

    hardware_init();

    /* Physical LEDs + buttons; button events go to the engine. */
    prop_bootlog_mark(BOOT_STAGE_IO);
    ESP_ERROR_CHECK(bsp_io_init(on_button, NULL));

    /* Persistent settings (NVS) — init before anything reads config. Survives
     * reboots and reflashes. */
    prop_bootlog_mark(BOOT_STAGE_SETTINGS);
    ESP_ERROR_CHECK(prop_settings_init());

    /* Configurable I/O bench (digital/analog in+out on the header pins). Restores
     * saved pin modes from NVS; needs nvs up (prop_settings_init ran nvs_flash_init). */
    prop_bootlog_mark(BOOT_STAGE_AIO);
    ESP_ERROR_CHECK(bsp_aio_init());

    /* Re-apply the saved backlight brightness (hardware_init lit it at 100%). */
    uint32_t brightness = 80;
    prop_settings_get_u32("brightness", &brightness, 80);
    set_lcd_blight(brightness);

    /* Brain: must exist before observers (UI/API) attach. */
    prop_bootlog_mark(BOOT_STAGE_ENGINE);
    ESP_ERROR_CHECK(prop_engine_init());

    /* Screen UI (observer) — after display + engine. */
    prop_bootlog_mark(BOOT_STAGE_UI);
    ESP_ERROR_CHECK(prop_ui_init());

    /* CRT effects overlay on the top layer (hidden unless enabled in settings). */
    prop_bootlog_mark(BOOT_STAGE_FX);
    esp_err_t fx_err = prop_fx_init();
    if (fx_err != ESP_OK) {
        MAIN_ERROR("fx overlay unavailable (%s) — running without CRT effects",
                   esp_err_to_name(fx_err));
    }

    /* PDM microphone capture for the SPECTRUM instrument. NON-fatal: the prop
     * runs fine without audio input (the spectrum screen shows offline). */
    prop_bootlog_mark(BOOT_STAGE_MIC);
    esp_err_t mic_err = prop_mic_init();
    if (mic_err != ESP_OK) {
        MAIN_ERROR("mic unavailable (%s) — SPECTRUM will show offline",
                   esp_err_to_name(mic_err));
    }

    /* Synthesized feedback audio over the speaker amp. NON-fatal: if the amp/I2S
     * won't come up the prop runs silent (prop_audio_play becomes a no-op). */
    prop_bootlog_mark(BOOT_STAGE_AUDIO);
    esp_err_t audio_err = prop_audio_init();
    if (audio_err != ESP_OK) {
        MAIN_ERROR("audio unavailable (%s) — running without feedback tones",
                   esp_err_to_name(audio_err));
    }

    /* LD2450 24 GHz multi-target radar on UART2 (GPIO53/54). NON-fatal. */
    prop_bootlog_mark(BOOT_STAGE_MOTION);
    esp_err_t motion_err = prop_motion_init();
    if (motion_err != ESP_OK) {
        MAIN_ERROR("motion radar unavailable (%s) — SCANNER will show offline",
                   esp_err_to_name(motion_err));
    }

    /* MPU-6500 IMU with DMP on the shared I2C_NUM_0 bus (GPIO45/46, addr 0x68).
     * NON-fatal: absent if the module is not wired; VITALS/SCANNER show "-- °". */
    prop_bootlog_mark(BOOT_STAGE_IMU);
    esp_err_t imu_err = prop_imu_init();
    if (imu_err != ESP_OK) {
        MAIN_ERROR("IMU unavailable (%s) — gimbal/VITALS motion data offline",
                   esp_err_to_name(imu_err));
    }

    /* Dead-reckoning / spatial memory (MINIMAP): fuses the DMP pedometer + radar
     * into a world-frame pose, path, and target marks. NON-fatal: idles with an
     * invalid pose if the IMU is absent. After prop_imu/prop_motion are up. */
    prop_bootlog_mark(BOOT_STAGE_TRACK);
    esp_err_t track_err = prop_track_init();
    if (track_err != ESP_OK) {
        MAIN_ERROR("dead-reckoning unavailable (%s) — MINIMAP will show no path",
                   esp_err_to_name(track_err));
    }

    /* Seeed 24 GHz (UART3, GPIO47/48, J2) + SEN0395 (UART1, GPIO34/33, J10). NON-fatal. */
    prop_bootlog_mark(BOOT_STAGE_AUX_RADAR);
    esp_err_t aux_err = prop_aux_radar_init();
    if (aux_err != ESP_OK) {
        MAIN_ERROR("aux radar init partial (%s) — offline sensors will show AUX_OFFLINE",
                   esp_err_to_name(aux_err));
    }

    /* WiFi (AP+STA via the C6) then the live control API. Both are NON-fatal:
     * the C6 radio is optional to the prop's core function, so a co-processor
     * problem must not take down the display/LEDs/buttons. */
    prop_bootlog_mark(BOOT_STAGE_NET);
    esp_err_t net_err = prop_net_init();
    if (net_err != ESP_OK) {
        MAIN_ERROR("WiFi unavailable (%s) — prop runs locally; no remote cues/OTA",
                   esp_err_to_name(net_err));
    } else {
        prop_bootlog_mark(BOOT_STAGE_API);
        esp_err_t api_err = prop_api_init();
        if (api_err != ESP_OK) {
            MAIN_ERROR("control API failed to start: %s", esp_err_to_name(api_err));
        }

        /* Custom-RPC link to the C6 (on-C6 CSI capture). NON-fatal: the SDIO
         * transport is up from prop_net_init, so just register the receiver. */
        prop_bootlog_mark(BOOT_STAGE_COPROC);
        esp_err_t coproc_err = prop_coproc_init();
        if (coproc_err != ESP_OK) {
            MAIN_ERROR("co-processor RPC unavailable (%s) — no on-C6 CSI feed",
                       esp_err_to_name(coproc_err));
        }

        /* CSI traffic generator: pings the gateway so the C6 has frames to
         * measure. NON-fatal; idle until a rate is configured + STA is up. */
        prop_bootlog_mark(BOOT_STAGE_TRAFFIC);
        prop_traffic_init();

        /* Guided two-phase auto-calibration (CSI CONFIG panel). */
        prop_bootlog_mark(BOOT_STAGE_CALIB);
        prop_calib_init();

        /* BLE scan (CONTACT SIGNATURES) — the C6 hosts the controller, sharing the
         * SDIO link WiFi just brought up. NON-fatal: if the controller/host won't
         * come up (or RAM is tight) the panel shows "BLE OFFLINE" and the rest runs. */
        prop_bootlog_mark(BOOT_STAGE_BLE);
        esp_err_t ble_err = prop_ble_init();
        if (ble_err != ESP_OK) {
            MAIN_ERROR("BLE unavailable (%s) — CONTACTS will show offline",
                       esp_err_to_name(ble_err));
        }

        /* WiFi CSI (SIGNAL ENVIRONMENT) — best-effort real CSI from the C6, with a
         * synthetic RSSI-driven fallback baked in, so it never fails the prop. */
        prop_bootlog_mark(BOOT_STAGE_CSI);
        prop_csi_init();

        /* WiFi FTM ranging (RANGE) — real 802.11mc ranging run on the C6 (see
         * prop_ftm.c). NON-fatal: if the task can't start, the RANGE panel just
         * shows an empty table. */
        prop_bootlog_mark(BOOT_STAGE_FTM);
        esp_err_t ftm_err = prop_ftm_init();
        if (ftm_err != ESP_OK) {
            MAIN_ERROR("FTM ranging unavailable (%s) — RANGE will show empty",
                       esp_err_to_name(ftm_err));
        }

        /* LiDAR thin-client render link (LIDAR panel) — connects out to the
         * lidar-roomscanner rig's /ws-thin endpoint via mDNS. NON-fatal: if the task
         * can't start, the panel just shows LINK: SEARCHING forever. */
        prop_bootlog_mark(BOOT_STAGE_LIDAR);
        esp_err_t lidar_err = prop_lidar_init();
        if (lidar_err != ESP_OK) {
            MAIN_ERROR("LiDAR link unavailable (%s) — LIDAR panel will show no data",
                       esp_err_to_name(lidar_err));
        }
    }

    /* Full bring-up complete: mark this OTA image valid so the bootloader won't
     * roll back to the previous partition on the next reset. Harmless on factory
     * boots; essential after an OTA update (image starts as PENDING_VERIFY). */
    esp_ota_mark_app_valid_cancel_rollback();
    prop_bootlog_mark(BOOT_STAGE_READY);

    MAIN_INFO("ready — AP '%s', console at http://<ip>/", PROP_AP_SSID);

    /* Boot chime — also the on-device validation that the amp output path works. */
    prop_audio_play(PA_BOOT);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
