/* bsp_io — discrete LED outputs + debounced buttons. See bsp_io.h. */
#include "bsp_io.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "button_gpio.h"

/* ---- Hardware tables -----------------------------------------------------
 * Edit these to match your wiring. Indices MUST match the enums in bsp_io.h.
 * Only GPIO48 is confirmed broken-out + free (UART1-RX header). The others are
 * placeholders on candidate-free pins — verify against the V1.0 schematic.
 */
static const struct {
    int gpio;
    const char *name;
} led_table[LED_COUNT] = {
    [LED_POWER]  = { 48, "power"  },
    [LED_SIGNAL] = { 47, "signal" },  /* UART1-TX header pin; verify before wiring */
    [LED_ALERT]  = { 20, "alert"  },  /* candidate free pin; verify before wiring  */
};

static const struct {
    int gpio;
    const char *name;
} button_table[BTN_COUNT] = {
    [BTN_MODE]   = { 28, "mode"   },  /* reassigned to J7/B20; GPIO33 freed for Seeed UART3 */
    [BTN_ACTION] = { 11, "action" },  /* candidate free pin; verify before wiring  */
};

/* ---- State -------------------------------------------------------------- */
static bool s_led_state[LED_COUNT];
static bsp_io_button_cb_t s_btn_cb;
static void *s_btn_ctx;
static button_handle_t s_btn_handles[BTN_COUNT];

/* Maps an iot_button event to our event + button index passed via usr_data. */
static void button_event_handler(void *arg, void *usr_data)
{
    intptr_t packed = (intptr_t)usr_data;
    prop_button_t button = (prop_button_t)(packed & 0xFF);
    prop_button_event_t event = (prop_button_event_t)((packed >> 8) & 0xFF);
    if (s_btn_cb) {
        s_btn_cb(button, event, s_btn_ctx);
    }
}

const char *bsp_io_led_name(prop_led_t led)
{
    return (led < LED_COUNT) ? led_table[led].name : "?";
}

esp_err_t bsp_io_led_set(prop_led_t led, bool on)
{
    if (led >= LED_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    s_led_state[led] = on;
    return gpio_set_level(led_table[led].gpio, on ? 1 : 0);
}

esp_err_t bsp_io_led_set_mask(uint32_t mask)
{
    esp_err_t err = ESP_OK;
    for (int i = 0; i < LED_COUNT; i++) {
        esp_err_t e = bsp_io_led_set((prop_led_t)i, (mask >> i) & 0x1);
        if (e != ESP_OK) {
            err = e;
        }
    }
    return err;
}

bool bsp_io_led_get(prop_led_t led)
{
    return (led < LED_COUNT) ? s_led_state[led] : false;
}

static esp_err_t led_init(void)
{
    uint64_t pin_mask = 0;
    for (int i = 0; i < LED_COUNT; i++) {
        pin_mask |= (1ULL << led_table[i].gpio);
    }
    const gpio_config_t cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    return bsp_io_led_set_mask(0);  /* all off */
}

/* Registers one iot_button with PRESS_DOWN, LONG_PRESS_START, PRESS_UP callbacks. */
static esp_err_t register_button(prop_button_t idx)
{
    const button_config_t btn_cfg = { 0 };
    const button_gpio_config_t gpio_cfg = {
        .gpio_num = button_table[idx].gpio,
        .active_level = 0,          /* active-low with pull-up */
        .enable_power_save = false,
        .disable_pull = false,
    };
    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &s_btn_handles[idx]);
    if (err != ESP_OK) {
        ESP_LOGE(IO_TAG, "button '%s' (gpio %d) create failed: %s",
                 button_table[idx].name, button_table[idx].gpio, esp_err_to_name(err));
        return err;
    }

    /* Pack (event<<8 | button index) into the user_data pointer. */
    intptr_t press   = (intptr_t)((BTN_EVENT_PRESS      << 8) | idx);
    intptr_t lpress  = (intptr_t)((BTN_EVENT_LONG_PRESS << 8) | idx);
    intptr_t release = (intptr_t)((BTN_EVENT_RELEASE    << 8) | idx);
    iot_button_register_cb(s_btn_handles[idx], BUTTON_PRESS_DOWN,      NULL, button_event_handler, (void *)press);
    iot_button_register_cb(s_btn_handles[idx], BUTTON_LONG_PRESS_START, NULL, button_event_handler, (void *)lpress);
    iot_button_register_cb(s_btn_handles[idx], BUTTON_PRESS_UP,         NULL, button_event_handler, (void *)release);
    return ESP_OK;
}

esp_err_t bsp_io_init(bsp_io_button_cb_t cb, void *user_ctx)
{
    s_btn_cb = cb;
    s_btn_ctx = user_ctx;

    esp_err_t err = led_init();
    if (err != ESP_OK) {
        ESP_LOGE(IO_TAG, "LED init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(IO_TAG, "%d LEDs initialized", LED_COUNT);

    if (cb) {
        for (int i = 0; i < BTN_COUNT; i++) {
            esp_err_t e = register_button((prop_button_t)i);
            if (e != ESP_OK) {
                return e;  /* surface wiring/pin errors early */
            }
        }
        ESP_LOGI(IO_TAG, "%d buttons registered", BTN_COUNT);
    }
    return ESP_OK;
}
