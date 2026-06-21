FILE(GLOB_RECURSE main ${CMAKE_SOURCE_DIR}/main/*.c)

idf_component_register(SRCS ${main}
                       INCLUDE_DIRS "include"
                       REQUIRES bsp_display bsp_illuminate bsp_i2c bsp_io
                                esp_timer esp_lvgl_port lvgl
                                esp_wifi esp_netif esp_event nvs_flash
                                esp_http_server app_update cjson mdns
                                esp_driver_tsens esp_driver_i2s
                                esp_mm)   # esp_cache.h for the /screenshot framebuffer read

# Force the linker to keep lv_port_mem.o (our custom LVGL allocator). LVGL's archive
# references lv_malloc_core et al., but libmain.a is scanned before liblvgl.a, so the
# member would be dropped and the symbols left undefined. -u pulls the whole TU in.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u lv_malloc_core")
