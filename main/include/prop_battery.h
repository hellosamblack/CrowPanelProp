#ifndef _PROP_BATTERY_H_
#define _PROP_BATTERY_H_

/* prop_battery — battery telemetry via the STC8H1K08 co-processor (U14), on
 * the shared I2C1 bus (bsp_i2c, addr 0x2F).
 *
 * The board's TP4059 charger (J3 battery plug) has NO wire to the ESP32-P4:
 * per the KiCad netlist, /ADC_VBAT, /POWER_CHRG and /POWER_DONE all land
 * exclusively on the STC8H1K08. The P4's only path to battery data is
 * querying that co-processor over I2C. Protocol recovered from Elecrow's
 * factory source (a *different* eval board using the same STC8H1K08 chip —
 * see the stc8h1k08-battery-i2c-protocol memory) and confirmed empirically
 * on this board:
 *   - addr 0x2F, reg 0x00, sequential read of 11 raw bytes:
 *     adc_mv (u32 LE), bat_mv (u32 LE), level_pct (u8), state (u8), led (u8)
 *   - the chip computes state-of-charge itself; there is no coulomb counter,
 *     so time-to-empty/time-to-full are derived here from the %/min rate of
 *     change of level_pct over a rolling window (see prop_battery.c).
 *
 * Non-fatal: if the STC8 doesn't ACK, prop_battery_available() stays false
 * and callers get zeroed/invalid data — the prop never hangs on this.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    PROP_BATT_IDLE = 0,
    PROP_BATT_CHARGING,
    PROP_BATT_FULLY_CHARGED,
    PROP_BATT_NO_CHARGE,      /* running on battery (no USB / not charging) */
    PROP_BATT_ERROR,
    /* Not part of the STC8's own enum (0-4 above) — the chip reports the whole
     * struct as 0xFF bytes when no battery is plugged into J3 at all. Recovered
     * empirically (see stc8h1k08-battery-i2c-protocol memory); surfaced as its
     * own state rather than silently misread as "100%, ERROR". */
    PROP_BATT_NOT_PRESENT,
} prop_battery_state_t;

typedef struct {
    bool     online;              /* STC8 ACKed at init */
    bool     valid;                /* at least one good read since boot */
    uint32_t adc_mv;               /* raw ADC pin voltage, mV */
    uint32_t voltage_mv;           /* battery voltage after divider back-conversion, mV */
    uint8_t  level_pct;            /* state of charge, 0-100 (chip-computed) */
    prop_battery_state_t state;
    uint8_t  led_state;            /* raw EM_LED_STATE from the chip, for diagnostics */

    bool     time_to_empty_valid;
    uint32_t time_to_empty_min;
    bool     time_to_full_valid;
    uint32_t time_to_full_min;
} prop_battery_data_t;

/* Bring up the STC8 link and start the background poll task.
 * Returns ESP_ERR_NOT_FOUND if the co-processor doesn't ACK at 0x2F.
 * Non-fatal: prop_battery_available() stays false. */
esp_err_t prop_battery_init(void);

bool prop_battery_available(void);

/* Copy latest cached reading into *out. Safe from any task. */
void prop_battery_get_data(prop_battery_data_t *out);

#endif /* _PROP_BATTERY_H_ */
