#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"

/* BSP (reused from Lesson09) */
#include "bsp_display.h"
#include "bsp_illuminate.h"
#include "bsp_i2c.h"
#include "bsp_io.h"

/* Prop modules */
#include "prop_settings.h"
#include "prop_engine.h"
#include "prop_net.h"
#include "prop_api.h"
#include "prop_ui.h"

#define MAIN_TAG "MAIN"
#define MAIN_INFO(fmt, ...)  ESP_LOGI(MAIN_TAG, fmt, ##__VA_ARGS__)
#define MAIN_ERROR(fmt, ...) ESP_LOGE(MAIN_TAG, fmt, ##__VA_ARGS__)

#endif /* _MAIN_H_ */
