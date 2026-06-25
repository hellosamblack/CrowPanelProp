/*
 * Shim so the vendored (GPL-3.0) ESPectre sources compile outside ESPHome.
 * ESPectre's only ESPHome dependency in the ported files is this logging header;
 * its ESP_LOGx macros are just IDF's, so we forward to esp_log.h. ESP_LOGCONFIG
 * is an ESPHome-ism (config-dump logging) → map to ESP_LOGI.
 */
#pragma once
#include "esp_log.h"

#ifndef ESP_LOGCONFIG
#define ESP_LOGCONFIG(tag, ...) ESP_LOGI(tag, __VA_ARGS__)
#endif
