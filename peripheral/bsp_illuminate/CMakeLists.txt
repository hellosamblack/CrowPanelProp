FILE(GLOB_RECURSE component_sources "*.c")

idf_component_register(SRCS ${component_sources}
                        INCLUDE_DIRS "include"
                        REQUIRES esp_driver_gpio esp_driver_ledc esp_lcd esp_lcd_ek79007 lvgl esp_lvgl_port bsp_display)
                        