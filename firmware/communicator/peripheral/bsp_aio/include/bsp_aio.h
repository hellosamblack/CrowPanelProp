#ifndef _BSP_AIO_H_
#define _BSP_AIO_H_

/* bsp_aio — the configurable I/O bench: per-pin config + analog in/out on the board's
 * broken-out header pins.
 *
 * Four plain-language modes, each persisted to NVS so the bench survives reboots and
 * reflashes:
 *   AIO_DIGITAL_IN   read the pin (optionally with pull-up/down and an edge interrupt)
 *   AIO_DIGITAL_OUT  drive the pin. Push-pull by default; an "open-drain" option only
 *                    pulls LOW and releases HIGH (for shared/bus lines — I2C, wired-AND,
 *                    1-Wire). The input buffer is left on so the real line is read back.
 *   AIO_ANALOG_IN    ADC2 — only on adc_ok pins (IO49..54), with selectable attenuation
 *   AIO_ANALOG_OUT   LEDC PWM. The ESP32-P4 has NO DAC, so volts are NOMINAL
 *                    (duty% x Vfull); add an external RC low-pass for a true level.
 *
 * Pin set = "safe free" headers only. NOT exposed: IO45/46 (touch I2C bus — driving
 * them kills the touchscreen) and IO47/48/33 (bsp_io LEDs/buttons). The comm-board
 * socket pins (IO6/7/8/10/53/54) and IO27/28 are free only while no LoRa/nRF module
 * is installed and the radio stays disabled in sdkconfig.
 *
 * Hardware refs: ADC channel map components/soc/esp32p4/include/soc/adc_channel.h
 * (ADC2 ch0..5 = GPIO49..54); LEDC has 8 channels / 4 timers (ch0+timer0 are the
 * backlight, see bsp_illuminate.c) — this driver shares timer 1 + channels 1..7, so up
 * to 7 analog-out pins can be live at once (all sharing one PWM frequency).
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define AIO_TAG  "BSP_AIO"
#define AIO_VREF 3.3f          /* nominal full-scale for the PWM volts readout */

/* Pin operating modes. Keep in sync with the dropdown options in prop_ui.c. */
typedef enum {
    AIO_DIGITAL_IN = 0,
    AIO_DIGITAL_OUT,
    AIO_ANALOG_IN,
    AIO_ANALOG_OUT,
    AIO_MODE_COUNT,
} aio_mode_t;

/* Internal pull resistor. Mutually exclusive — a pin can't be pulled both ways. */
typedef enum { AIO_PULL_NONE = 0, AIO_PULL_UP, AIO_PULL_DOWN, AIO_PULL_COUNT } aio_pull_t;

/* Edge interrupt selection. Edge-only on purpose — level interrupts re-fire without an
 * ack and would lock the CPU. */
typedef enum { AIO_IRQ_OFF = 0, AIO_IRQ_RISING, AIO_IRQ_FALLING, AIO_IRQ_ANY, AIO_IRQ_COUNT } aio_irq_t;

#define AIO_ATTEN_COUNT 4      /* ADC attenuation steps: 0 / 2.5 / 6 / 12 dB */
#define AIO_FREQ_COUNT  6      /* analog-out PWM frequency presets */
#define AIO_FILT_COUNT  4      /* analog-in IIR smoothing: off / light / medium / heavy */

static inline bool aio_is_input(aio_mode_t m)  { return m == AIO_DIGITAL_IN; }
static inline bool aio_is_output(aio_mode_t m) { return m == AIO_DIGITAL_OUT; }
static inline bool aio_is_digital(aio_mode_t m){ return m == AIO_DIGITAL_IN || m == AIO_DIGITAL_OUT; }

/* Static description of one exposed pin. */
typedef struct {
    int         gpio;    /* GPIO number */
    const char *label;   /* short name, e.g. "IO2" */
    const char *origin;  /* "HDR" (board header) or "COMM" (comm-board socket) */
    bool        adc_ok;  /* true if analog-in capable (an ADC2 channel) */
} aio_pin_t;

/* Bring up the bench: restore each pin's saved mode/options from NVS and apply them.
 * On first boot (a pin with nothing saved) it seeds a sensible default and persists it
 * — ADC pins as AIO_ANALOG_IN, the rest as AIO_DIGITAL_IN with a pull-down (all inputs,
 * nothing driven). Call after nvs is initialised (prop_settings_init runs nvs_flash_init). */
esp_err_t bsp_aio_init(void);

int              bsp_aio_count(void);
const aio_pin_t *bsp_aio_info(int idx);

aio_mode_t bsp_aio_get_mode(int idx);
/* Reconfigure a pin: release prior resources (free its LEDC channel / interrupt, reset
 * the GPIO), apply the new mode with its saved options, and persist. ESP_ERR_NOT_SUPPORTED
 * for AIO_ANALOG_IN on a non-ADC pin; ESP_ERR_NO_MEM if AIO_ANALOG_OUT has no free PWM channel. */
esp_err_t  bsp_aio_set_mode(int idx, aio_mode_t mode);

/* Digital options. */
aio_pull_t bsp_aio_get_pull(int idx);      esp_err_t bsp_aio_set_pull(int idx, aio_pull_t pull);
bool       bsp_aio_get_od(int idx);        esp_err_t bsp_aio_set_od(int idx, bool on);   /* output open-drain */
int        bsp_aio_get_drive(int idx);     esp_err_t bsp_aio_set_drive(int idx, int cap /*0..3*/);
aio_irq_t  bsp_aio_get_irq(int idx);       esp_err_t bsp_aio_set_irq(int idx, aio_irq_t irq);
uint32_t   bsp_aio_get_edges(int idx);     void      bsp_aio_reset_edges(int idx);

/* Pin level (0/1): read line for digital modes (the output read-back for AIO_DIGITAL_OUT),
 * or -1 for analog modes. */
int bsp_aio_read_level(int idx);

bool      bsp_aio_get_dout(int idx);
esp_err_t bsp_aio_set_dout(int idx, bool on);

/* Analog input. atten index 0..3 -> ADC_ATTEN_DB_0/2_5/6/12 (selects the input range).
 * bsp_aio_ain_vmax() is the approximate full-scale voltage for the current attenuation.
 * filter index 0..3 picks an IIR (exponential-moving-average) smoothing strength. */
int        bsp_aio_get_atten(int idx);     esp_err_t bsp_aio_set_atten(int idx, int atten);
int        bsp_aio_get_filter(int idx);    esp_err_t bsp_aio_set_filter(int idx, int filter);
float      bsp_aio_ain_vmax(int idx);
esp_err_t  bsp_aio_read_ain(int idx, int *raw, int *pct);   /* filtered raw 0..4095, pct of full scale */

/* Analog output (LEDC PWM). Frequency is one shared preset across all analog-out pins. */
int       bsp_aio_get_aout(int idx);       esp_err_t bsp_aio_set_aout(int idx, int pct);
int       bsp_aio_get_freq(void);          esp_err_t bsp_aio_set_freq(int idx);  /* 0..AIO_FREQ_COUNT-1 */
uint32_t  bsp_aio_freq_hz(int idx);        /* hz for a frequency preset index */
bool      bsp_aio_get_volts_pref(int idx); void      bsp_aio_set_volts_pref(int idx, bool volts);

/* Nominal volts for an analog-out duty percent (duty% x 3.3 V). */
static inline float bsp_aio_volts(int pct) { return (float)pct / 100.0f * AIO_VREF; }

#endif /* _BSP_AIO_H_ */
