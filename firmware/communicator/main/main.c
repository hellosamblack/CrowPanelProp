/* Communicator/scanner prop — init orchestration.
 *
 * Hardware bring-up order mirrors Lesson09 (LDO -> I2C -> touch -> display ->
 * backlight), then layers the prop on top: physical I/O, the engine (brain),
 * the screen UI, WiFi, and the live control API.
 */
#include "main.h"

static esp_ldo_channel_handle_t ldo3;
static esp_ldo_channel_handle_t ldo4;

/* Physical buttons funnel straight into the engine (the single input hub). */
static void on_button(prop_button_t button, prop_button_event_t event, void *ctx)
{
    (void)ctx;
    if (event != BTN_EVENT_PRESS) {
        return;  /* act on press; long-press/release reserved for future use */
    }
    switch (button) {
        case BTN_MODE:
            prop_engine_next_scene();
            break;
        case BTN_ACTION:
            prop_engine_set_scene(SCENE_SCANNING);
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
    esp_ldo_channel_config_t ldo3_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    err = esp_ldo_acquire_channel(&ldo3_cfg, &ldo3);
    if (err != ESP_OK) fail_loop("ldo3", err);
    esp_ldo_channel_config_t ldo4_cfg = { .chan_id = 4, .voltage_mv = 3300 };
    err = esp_ldo_acquire_channel(&ldo4_cfg, &ldo4);
    if (err != ESP_OK) fail_loop("ldo4", err);

    /* 2. I2C (touch bus). */
    err = i2c_init();
    if (err != ESP_OK) fail_loop("i2c", err);

    /* 3. Touch panel. */
    err = touch_init();
    if (err != ESP_OK) fail_loop("touch", err);

    /* 4. LCD + LVGL. */
    err = display_init();
    if (err != ESP_OK) fail_loop("display", err);

    /* 5. Backlight on. */
    err = set_lcd_blight(100);
    if (err != ESP_OK) fail_loop("backlight", err);

    MAIN_INFO("display stack up");
}

void app_main(void)
{
    MAIN_INFO("communicator prop starting");

    hardware_init();

    /* Physical LEDs + buttons; button events go to the engine. */
    ESP_ERROR_CHECK(bsp_io_init(on_button, NULL));

    /* Persistent settings (NVS) — init before anything reads config. Survives
     * reboots and reflashes. */
    ESP_ERROR_CHECK(prop_settings_init());

    /* Re-apply the saved backlight brightness (hardware_init lit it at 100%). */
    uint32_t brightness = 80;
    prop_settings_get_u32("brightness", &brightness, 80);
    set_lcd_blight(brightness);

    /* Brain: must exist before observers (UI/API) attach. */
    ESP_ERROR_CHECK(prop_engine_init());

    /* Screen UI (observer) — after display + engine. */
    ESP_ERROR_CHECK(prop_ui_init());

    /* CRT effects overlay on the top layer (hidden unless enabled in settings). */
    esp_err_t fx_err = prop_fx_init();
    if (fx_err != ESP_OK) {
        MAIN_ERROR("fx overlay unavailable (%s) — running without CRT effects",
                   esp_err_to_name(fx_err));
    }

    /* PDM microphone capture for the SPECTRUM instrument. NON-fatal: the prop
     * runs fine without audio input (the spectrum screen shows offline). */
    esp_err_t mic_err = prop_mic_init();
    if (mic_err != ESP_OK) {
        MAIN_ERROR("mic unavailable (%s) — SPECTRUM will show offline",
                   esp_err_to_name(mic_err));
    }

    /* WiFi (AP+STA via the C6) then the live control API. Both are NON-fatal:
     * the C6 radio is optional to the prop's core function, so a co-processor
     * problem must not take down the display/LEDs/buttons. */
    esp_err_t net_err = prop_net_init();
    if (net_err != ESP_OK) {
        MAIN_ERROR("WiFi unavailable (%s) — prop runs locally; no remote cues/OTA",
                   esp_err_to_name(net_err));
    } else {
        esp_err_t api_err = prop_api_init();
        if (api_err != ESP_OK) {
            MAIN_ERROR("control API failed to start: %s", esp_err_to_name(api_err));
        }
    }

    MAIN_INFO("ready — AP '%s', console at http://<ip>/", PROP_AP_SSID);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
