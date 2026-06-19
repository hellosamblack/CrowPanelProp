FILE(GLOB_RECURSE main ${CMAKE_SOURCE_DIR}/main/*.c)

idf_component_register(SRCS ${main}
                       INCLUDE_DIRS "include"
                       REQUIRES bsp_display bsp_illuminate bsp_i2c bsp_io
                                esp_timer esp_lvgl_port lvgl
                                esp_wifi esp_netif esp_event nvs_flash
                                esp_http_server app_update cjson mdns
                                esp_driver_tsens esp_driver_i2s)
