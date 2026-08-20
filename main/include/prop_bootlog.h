#ifndef _PROP_BOOTLOG_H_
#define _PROP_BOOTLOG_H_

/* prop_bootlog — RTC-memory boot-stage breadcrumb.
 *
 * Diagnoses which app_main() init stage was running when the board last reset,
 * without needing a live serial capture at the exact moment of the crash. A
 * couple of words in the RTC .noinit section survive any reset that isn't a
 * full power cycle (panic, WDT, brownout, esp_restart), so the value written
 * just before this boot started is still readable now, before we overwrite it.
 *
 * Usage: prop_bootlog_init() first thing in app_main() (before hardware_init,
 * so even an early panic is attributable), then prop_bootlog_mark(stage) right
 * before each risky init call. If the board dies mid-stage, the *next* boot's
 * prop_bootlog_prev_stage_name() reports "what was running" — exposed over
 * /state for tools/prop.py.
 *
 * NOTE: deliberately does NOT also report esp_reset_reason() ("why" it reset).
 * Tried that first and it broke boot on real hardware: this board's internal
 * RAM margin at early boot is only a couple hundred bytes (see idf6-migration
 * / ftm-ranging-and-ram-margin memories), and calling esp_reset_reason() for
 * the first time anywhere in this app pulls in esp_system's reset_reason.c —
 * enough extra static footprint on its own to flip a working boot into a hard
 * `vApplicationGetTimerTaskMemory` boot loop, confirmed via git-stash
 * bisection on hardware. If you want reset-reason back, free real budget
 * first (e.g. the esp_hosted DFLT_TASK_STACK_SIZE fix noted in
 * ftm-ranging-and-ram-margin) and re-measure before re-adding the call. */

typedef enum {
    BOOT_STAGE_START = 0,
    BOOT_STAGE_HW_LDO,
    BOOT_STAGE_HW_I2C,
    BOOT_STAGE_HW_TOUCH,
    BOOT_STAGE_HW_DISPLAY,
    BOOT_STAGE_HW_BACKLIGHT,
    BOOT_STAGE_IO,
    BOOT_STAGE_SETTINGS,
    BOOT_STAGE_AIO,
    BOOT_STAGE_ENGINE,
    BOOT_STAGE_UI,
    BOOT_STAGE_FX,
    BOOT_STAGE_MIC,
    BOOT_STAGE_AUDIO,
    BOOT_STAGE_MOTION,
    BOOT_STAGE_IMU,
    BOOT_STAGE_BATTERY,
    BOOT_STAGE_TRACK,
    BOOT_STAGE_AUX_RADAR,
    BOOT_STAGE_NET,
    BOOT_STAGE_API,
    BOOT_STAGE_COPROC,
    BOOT_STAGE_TRAFFIC,
    BOOT_STAGE_CALIB,
    BOOT_STAGE_BLE,
    BOOT_STAGE_CSI,
    BOOT_STAGE_FTM,
    BOOT_STAGE_LIDAR,
    BOOT_STAGE_READY,
    BOOT_STAGE_COUNT,
} prop_boot_stage_t;

/* Reads back the previous session's breadcrumb (if any) and logs it, then
 * resets the live breadcrumb for this session. Call first in app_main(). */
void prop_bootlog_init(void);

/* Records "about to attempt this stage" — call right before each init step. */
void prop_bootlog_mark(prop_boot_stage_t stage);

/* Name of a stage constant (for logging/JSON). */
const char *prop_bootlog_stage_name(prop_boot_stage_t stage);

/* Last stage the *previous* session reached before this reset — "none" if
 * there's no valid breadcrumb (first boot ever, or a full power-cycle since
 * RTC memory doesn't survive that). */
const char *prop_bootlog_prev_stage_name(void);

#endif /* _PROP_BOOTLOG_H_ */
