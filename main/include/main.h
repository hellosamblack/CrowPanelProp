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
#include "bsp_aio.h"
#include "bsp_audio.h"

/* Prop modules */
#include "prop_settings.h"
#include "prop_engine.h"
#include "prop_net.h"
#include "prop_api.h"
#include "prop_ui.h"
#include "prop_fx.h"
#include "prop_mic.h"
#include "prop_audio.h"
#include "prop_ble.h"
#include "prop_csi.h"
#include "prop_coproc.h"
#include "prop_ftm.h"
#include "prop_traffic.h"
#include "prop_calib.h"
#include "prop_radar.h"
#include "prop_imu.h"
#include "prop_bootlog.h"

#define MAIN_TAG "MAIN"
#define MAIN_INFO(fmt, ...)  ESP_LOGI(MAIN_TAG, fmt, ##__VA_ARGS__)
#define MAIN_ERROR(fmt, ...) ESP_LOGE(MAIN_TAG, fmt, ##__VA_ARGS__)

#endif /* _MAIN_H_ */
