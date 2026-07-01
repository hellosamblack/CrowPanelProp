#ifndef _BSP_IO_H_
#define _BSP_IO_H_

/* bsp_io — physical prop I/O: discrete on/off LEDs + debounced push buttons.
 *
 * LEDs are driven directly on GPIO (active-high). Buttons use the espressif/button
 * component for debounce + press/long-press detection. Both are described by small
 * tables so adding hardware is a one-line edit.
 *
 * Pin choices: GPIO48 (UART1-RX header pin) is the known-good, broken-out default LED.
 * Verify every other pin against Eagle_SCH&PCB/1.0/ before wiring. See the project plan.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define IO_TAG "BSP_IO"

/* Logical LED identifiers. Keep in sync with led_table[] in bsp_io.c. */
typedef enum {
    LED_POWER = 0,   /* GPIO48 — known-good default (UART1-RX header pin) */
    LED_SIGNAL,
    LED_ALERT,
    LED_COUNT,
} prop_led_t;

/* Logical button identifiers. Keep in sync with button_table[] in bsp_io.c. */
typedef enum {
    BTN_MODE = 0,
    BTN_ACTION,
    BTN_COUNT,
} prop_button_t;

typedef enum {
    BTN_EVENT_PRESS,
    BTN_EVENT_LONG_PRESS,
    BTN_EVENT_RELEASE,
} prop_button_event_t;

/* Callback invoked (from the button task context) when a button event fires. */
typedef void (*bsp_io_button_cb_t)(prop_button_t button, prop_button_event_t event, void *user_ctx);

/* Initialize all LED GPIOs (outputs, off) and register all buttons.
 * Pass NULL for cb to skip button handling. */
esp_err_t bsp_io_init(bsp_io_button_cb_t cb, void *user_ctx);

/* Set a single LED on/off. Safe to call from any task. */
esp_err_t bsp_io_led_set(prop_led_t led, bool on);

/* Convenience: set every LED at once from a bitmask (bit i => LED i on). */
esp_err_t bsp_io_led_set_mask(uint32_t mask);

/* Current cached state of an LED. */
bool bsp_io_led_get(prop_led_t led);

/* Human-readable name for a LED (matches the JSON "name" used by the API). */
const char *bsp_io_led_name(prop_led_t led);

#endif /* _BSP_IO_H_ */
