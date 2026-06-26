FILE(GLOB_RECURSE main ${CMAKE_SOURCE_DIR}/main/*.c)

# ---- Build-time WiFi secret -------------------------------------------------
# Read the gitignored wifi_secret.env (KEY=VALUE or KEY: VALUE — plain .env or simple
# YAML) and generate wifi_secret.h with PROP_WIFI_SSID / PROP_WIFI_PASS. prop_net.c
# uses these as the *default* STA creds, applied only when NVS has none (fresh flash /
# erase). Missing file or blank values -> empty defaults (unit comes up AP-only).
set(WIFI_SECRET_FILE "${CMAKE_SOURCE_DIR}/wifi_secret.env")
set(WIFI_SECRET_GEN_DIR "${CMAKE_BINARY_DIR}/wifi_secret")
set(PROP_WIFI_SSID "")
set(PROP_WIFI_PASS "")
if(EXISTS "${WIFI_SECRET_FILE}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${WIFI_SECRET_FILE}")
    file(STRINGS "${WIFI_SECRET_FILE}" _wifi_lines)
    foreach(_line IN LISTS _wifi_lines)
        string(REGEX REPLACE "\r$" "" _line "${_line}")
        if(_line MATCHES "^[ \t]*#")
            # comment line — skip
        elseif(_line MATCHES "^[ \t]*SSID[ \t]*[:=][ \t]*(.*)$")
            set(PROP_WIFI_SSID "${CMAKE_MATCH_1}")
        elseif(_line MATCHES "^[ \t]*PASS(WORD)?[ \t]*[:=][ \t]*(.*)$")
            set(PROP_WIFI_PASS "${CMAKE_MATCH_2}")
        endif()
    endforeach()
endif()
# Strip one layer of surrounding quotes, then escape for a C string literal.
foreach(_v PROP_WIFI_SSID PROP_WIFI_PASS)
    string(REGEX REPLACE "^\"(.*)\"$" "\\1" ${_v} "${${_v}}")
    string(REGEX REPLACE "^'(.*)'$" "\\1" ${_v} "${${_v}}")
    string(REPLACE "\\" "\\\\" ${_v} "${${_v}}")
    string(REPLACE "\"" "\\\"" ${_v} "${${_v}}")
endforeach()
file(MAKE_DIRECTORY "${WIFI_SECRET_GEN_DIR}")
file(WRITE "${WIFI_SECRET_GEN_DIR}/wifi_secret.h"
     "#pragma once\n"
     "/* GENERATED from wifi_secret.env by main/CMakeLists.txt — do not edit / commit. */\n"
     "#define PROP_WIFI_SSID \"${PROP_WIFI_SSID}\"\n"
     "#define PROP_WIFI_PASS \"${PROP_WIFI_PASS}\"\n")
if(PROP_WIFI_SSID STREQUAL "")
    message(STATUS "wifi_secret: no default SSID (set wifi_secret.env)")
else()
    message(STATUS "wifi_secret: baking default STA SSID '${PROP_WIFI_SSID}'")
endif()

idf_component_register(SRCS ${main}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${WIFI_SECRET_GEN_DIR}"
                       REQUIRES bsp_display bsp_illuminate bsp_i2c bsp_io bsp_aio bsp_audio
                                esp_timer esp_lvgl_port lvgl
                                esp_wifi esp_netif esp_event nvs_flash
                                esp_http_server app_update cjson mdns
                                esp_driver_tsens esp_driver_i2s esp_driver_i2c
                                bt esp_hosted   # NimBLE host + C6 BT controller (prop_ble)
                                esp_mm)   # esp_cache.h for the /screenshot framebuffer read

# Force the linker to keep lv_port_mem.o (our custom LVGL allocator). LVGL's archive
# references lv_malloc_core et al., but libmain.a is scanned before liblvgl.a, so the
# member would be dropped and the symbols left undefined. -u pulls the whole TU in.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u lv_malloc_core")
