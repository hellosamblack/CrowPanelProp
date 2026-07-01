/*————————————————————————————————————————Header file declaration————————————————————————————————————————*/
#include "bsp_display.h"   // Include the display BSP header
/*——————————————————————————————————————Header file declaration end——————————————————————————————————————*/

/*——————————————————————————————————————————Variable declaration—————————————————————————————————————————*/

// Handle for the GT911 touch panel
esp_lcd_touch_handle_t tp = NULL;  
// Handle for I2C panel I/O
static esp_lcd_panel_io_handle_t tp_io_handle = NULL;
/*————————————————————————————————————————Variable declaration end———————————————————————————————————————*/

/*—————————————————————————————————————————Functional function———————————————————————————————————————————*/

// Initialize the GT911 touch panel
esp_err_t touch_init(void)
{
    esp_err_t err = ESP_OK;  // Error status

    // I2C panel I/O configuration
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,  // Primary GT911 I2C address
        .control_phase_bytes = 1,                        // Control phase bytes
        .dc_bit_offset = 0,                              // Not used
        .lcd_cmd_bits = 16,                              // Command bit width
        .flags =
            {
                .disable_control_phase = 1,             // Disable control phase
            },
        .scl_speed_hz = 400000,                          // I2C clock speed
    };

    // GT911 touch configuration
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = H_size,          // Max X coordinate
        .y_max = V_size,          // Max Y coordinate
        .rst_gpio_num = Touch_GPIO_RST, // Reset GPIO
        .int_gpio_num = Touch_GPIO_INT, // Interrupt GPIO
        .levels = {
            .reset = 0,           // Reset level
            .interrupt = 0,       // Interrupt level
        },
        .flags = {
            .swap_xy = false,     // Do not swap X/Y
            .mirror_x = false,    // Do not mirror X
            .mirror_y = false,    // Do not mirror Y
        },
    };

    // Create I2C panel I/O
    err = esp_lcd_new_panel_io_i2c((i2c_master_bus_handle_t)i2c_bus_handle, &io_config, &tp_io_handle);
    if (err != ESP_OK)
        return err;

    // Initialize GT911 touch driver
    err = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp);
    if (err != ESP_OK)
    {
        // Try backup I2C address if primary fails (free the first panel IO first)
        esp_lcd_panel_io_del(tp_io_handle);
        tp_io_handle = NULL;
        io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
        err = esp_lcd_new_panel_io_i2c((i2c_master_bus_handle_t)i2c_bus_handle, &io_config, &tp_io_handle);
        if (err != ESP_OK)
            return err;
        err = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp);
        if (err != ESP_OK)
            return err;
    }

    return err;  // Return final status
}

/*———————————————————————————————————————Functional function end—————————————————————————————————————————*/
