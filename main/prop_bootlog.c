/* prop_bootlog — RTC-memory boot-stage breadcrumb. See prop_bootlog.h. */
#include "prop_bootlog.h"

#include "esp_attr.h"
#include "esp_log.h"

#define TAG "PROP_BOOTLOG"
#define BOOTLOG_MAGIC 0xB007CAFEu

/* .rtc_noinit lives in LP/RTC memory (a separate physical bank from the HP
 * core's internal SRAM — see the linker's lp_ram_seg), so it doesn't compete
 * with the same tight internal-heap budget everything else draws from. It's
 * preserved across panic/WDT/brownout/esp_restart resets, but not a full
 * power cycle (the RTC domain loses power) — the magic word tells the two
 * cases apart, since a power-cycled chip's RTC memory just has undefined
 * contents rather than being reliably zeroed.
 *
 * Deliberately keep ALL breadcrumb state here rather than caching a copy in
 * ordinary internal-SRAM statics, and deliberately do NOT call
 * esp_reset_reason() (see prop_bootlog.h) — this board's internal-heap margin
 * at early boot is measured in the low hundreds of bytes (see idf6-migration /
 * ftm-ranging-and-ram-margin memories), and both of those were confirmed live
 * on hardware to be enough on their own to flip a working boot into a hard
 * boot loop (`vApplicationGetTimerTaskMemory` assert): ~9 bytes of ordinary
 * .bss from the naive first version, and ~100+ bytes pulled in by linking
 * esp_system's reset_reason.c for the first time in this app. Every byte this
 * module needs comes out of LP memory instead, which isn't part of that
 * budget at all. */
static RTC_NOINIT_ATTR uint32_t s_rtc_magic;
static RTC_NOINIT_ATTR uint32_t s_rtc_stage;        /* live, updated by mark() */
static RTC_NOINIT_ATTR uint32_t s_rtc_prev_stage;   /* snapshot taken at init() */
static RTC_NOINIT_ATTR uint32_t s_rtc_prev_valid;   /* 1 if the above is meaningful */

static const char *s_stage_names[BOOT_STAGE_COUNT] = {
    [BOOT_STAGE_START]       = "start",
    [BOOT_STAGE_HW_LDO]      = "hw_ldo",
    [BOOT_STAGE_HW_I2C]      = "hw_i2c",
    [BOOT_STAGE_HW_TOUCH]    = "hw_touch",
    [BOOT_STAGE_HW_DISPLAY]  = "hw_display",
    [BOOT_STAGE_HW_BACKLIGHT] = "hw_backlight",
    [BOOT_STAGE_IO]          = "io",
    [BOOT_STAGE_SETTINGS]    = "settings",
    [BOOT_STAGE_AIO]         = "aio",
    [BOOT_STAGE_ENGINE]      = "engine",
    [BOOT_STAGE_UI]          = "ui",
    [BOOT_STAGE_FX]          = "fx",
    [BOOT_STAGE_MIC]         = "mic",
    [BOOT_STAGE_AUDIO]       = "audio",
    [BOOT_STAGE_MOTION]      = "motion",
    [BOOT_STAGE_IMU]         = "imu",
    [BOOT_STAGE_BATTERY]     = "battery",
    [BOOT_STAGE_TRACK]       = "track",
    [BOOT_STAGE_AUX_RADAR]   = "aux_radar",
    [BOOT_STAGE_NET]         = "net",
    [BOOT_STAGE_API]         = "api",
    [BOOT_STAGE_COPROC]      = "coproc",
    [BOOT_STAGE_TRAFFIC]     = "traffic",
    [BOOT_STAGE_CALIB]       = "calib",
    [BOOT_STAGE_BLE]         = "ble",
    [BOOT_STAGE_CSI]         = "csi",
    [BOOT_STAGE_FTM]         = "ftm",
    [BOOT_STAGE_LIDAR]       = "lidar",
    [BOOT_STAGE_READY]       = "ready",
};

const char *prop_bootlog_stage_name(prop_boot_stage_t stage)
{
    if (stage < 0 || stage >= BOOT_STAGE_COUNT || !s_stage_names[stage]) {
        return "?";
    }
    return s_stage_names[stage];
}

void prop_bootlog_init(void)
{
    if (s_rtc_magic == BOOTLOG_MAGIC && s_rtc_stage < BOOT_STAGE_COUNT) {
        s_rtc_prev_stage = s_rtc_stage;
        s_rtc_prev_valid = 1;
        ESP_LOGW(TAG, "previous session: last stage='%s'",
                 prop_bootlog_stage_name((prop_boot_stage_t)s_rtc_prev_stage));
    } else {
        s_rtc_prev_valid = 0;
        ESP_LOGI(TAG, "no breadcrumb from a previous session (first boot since power-on, or "
                       "fresh flash)");
    }

    /* Reset the live breadcrumb for this session. */
    s_rtc_magic = BOOTLOG_MAGIC;
    s_rtc_stage = BOOT_STAGE_START;
}

void prop_bootlog_mark(prop_boot_stage_t stage)
{
    s_rtc_stage = (uint32_t)stage;
}

const char *prop_bootlog_prev_stage_name(void)
{
    return s_rtc_prev_valid ? prop_bootlog_stage_name((prop_boot_stage_t)s_rtc_prev_stage) : "none";
}
