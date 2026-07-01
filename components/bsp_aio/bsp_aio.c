/* bsp_aio — configurable digital/analog I/O bench. See bsp_aio.h. */
#include "bsp_aio.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs.h"

#define NVS_NS "aiocfg"

/* ---- Pin table ----------------------------------------------------------
 * Safe, broken-out header pins only. adc_ok marks ADC2-capable pins (IO49..54);
 * for those the ADC2 channel is simply gpio-49 (ch0=49 .. ch5=54), per
 * soc/esp32p4/adc_channel.h — derived in adc_channel_of() rather than stored. */
static const aio_pin_t s_pins[] = {
    {  2, "IO2",  "HDR",  false },
    {  3, "IO3",  "HDR",  false },
    {  4, "IO4",  "HDR",  false },
    {  5, "IO5",  "HDR",  false },
    { 49, "IO49", "HDR",  true  },
    { 50, "IO50", "HDR",  true  },
    { 51, "IO51", "HDR",  true  },
    { 52, "IO52", "HDR",  true  },
    {  6, "IO6",  "COMM", false },
    {  7, "IO7",  "COMM", false },
    {  8, "IO8",  "COMM", false },
    { 10, "IO10", "COMM", false },
};
#define AIO_N ((int)(sizeof(s_pins) / sizeof(s_pins[0])))

/* ---- LEDC (analog out) ---------------------------------------------------
 * Backlight owns LEDC_TIMER_0 / LEDC_CHANNEL_0 (bsp_illuminate.c). We share one
 * spare timer and hand out channels 1..7 to AOUT pins; all share one frequency. */
#define AIO_LEDC_MODE   LEDC_LOW_SPEED_MODE
#define AIO_LEDC_TIMER  LEDC_TIMER_1
#define AIO_LEDC_RES    LEDC_TIMER_10_BIT
#define AIO_DUTY_MAX    ((1 << 10) - 1)         /* 1023 for 10-bit */
#define AIO_CH_FIRST    LEDC_CHANNEL_1
#define AIO_CH_LAST     LEDC_CHANNEL_7

static const uint32_t s_freq_hz[AIO_FREQ_COUNT]   = { 50, 200, 1000, 5000, 10000, 25000 };
static const adc_atten_t s_atten_db[AIO_ATTEN_COUNT] = {
    ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6, ADC_ATTEN_DB_12,
};
static const float s_atten_vmax[AIO_ATTEN_COUNT] = { 1.10f, 1.50f, 2.20f, 3.30f };  /* approx */
/* IIR smoothing weight per filter step (off uses the raw sample; lower = smoother). */
static const float s_filt_alpha[AIO_FILT_COUNT] = { 1.0f, 0.5f, 0.2f, 0.06f };

/* ---- Runtime state ------------------------------------------------------- */
static aio_mode_t s_mode[AIO_N];
static bool       s_dout[AIO_N];      /* output drive level */
static uint8_t    s_duty[AIO_N];      /* analog-out percent 0..100 */
static bool       s_volts[AIO_N];     /* display preference */
static uint8_t    s_pull[AIO_N];      /* aio_pull_t */
static bool       s_od[AIO_N];        /* output open-drain */
static uint8_t    s_drive[AIO_N];     /* gpio_drive_cap_t 0..3 */
static uint8_t    s_irq[AIO_N];       /* aio_irq_t */
static uint8_t    s_atten[AIO_N];     /* ADC attenuation index 0..3 */
static uint8_t    s_filt[AIO_N];      /* ADC IIR filter index 0..3 */
static float      s_ain_acc[AIO_N];   /* IIR accumulator (-1 = uninitialised) */
static volatile uint32_t s_edges[AIO_N];
static int8_t     s_chan[AIO_N];      /* assigned LEDC channel, or -1 */
static uint8_t    s_freq = 3;         /* shared analog-out frequency preset (5 kHz) */

static adc_oneshot_unit_handle_t s_adc;   /* ADC2, lazily created */
static bool s_ledc_timer_ready;
static bool s_isr_ready;

/* ---- Helpers ------------------------------------------------------------- */
static inline bool valid(int i) { return i >= 0 && i < AIO_N; }
static inline adc_channel_t adc_channel_of(int i) { return (adc_channel_t)(s_pins[i].gpio - 49); }
static inline int duty_raw(int pct) { return pct * AIO_DUTY_MAX / 100; }

static gpio_int_type_t irq_to_gpio(aio_irq_t it)
{
    switch (it) {
        case AIO_IRQ_RISING:  return GPIO_INTR_POSEDGE;
        case AIO_IRQ_FALLING: return GPIO_INTR_NEGEDGE;
        case AIO_IRQ_ANY:     return GPIO_INTR_ANYEDGE;
        default:              return GPIO_INTR_DISABLE;
    }
}
static gpio_pull_mode_t pull_mode_of(int i)
{
    if (s_pull[i] == AIO_PULL_UP)   return GPIO_PULLUP_ONLY;
    if (s_pull[i] == AIO_PULL_DOWN) return GPIO_PULLDOWN_ONLY;
    return GPIO_FLOATING;
}

static void IRAM_ATTR aio_isr(void *arg) { s_edges[(int)(intptr_t)arg]++; }

static void nvs_save_u8(const char *fmt, int gpio, uint8_t v)
{
    char key[8];
    snprintf(key, sizeof(key), fmt, gpio);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_u8(h, key, v) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}

static uint8_t nvs_load_u8(const char *fmt, int gpio, uint8_t def)
{
    char key[8];
    snprintf(key, sizeof(key), fmt, gpio);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return def;
    uint8_t v = def;
    if (nvs_get_u8(h, key, &v) != ESP_OK) v = def;
    nvs_close(h);
    return v;
}

/* True if the key has ever been written (distinguishes "unconfigured" from a saved 0). */
static bool nvs_has(const char *fmt, int gpio)
{
    char key[8];
    snprintf(key, sizeof(key), fmt, gpio);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t v;
    esp_err_t err = nvs_get_u8(h, key, &v);
    nvs_close(h);
    return err == ESP_OK;
}

static esp_err_t ensure_adc(void)
{
    if (s_adc) return ESP_OK;
    adc_oneshot_unit_init_cfg_t cfg = { .unit_id = ADC_UNIT_2, .ulp_mode = ADC_ULP_MODE_DISABLE };
    return adc_oneshot_new_unit(&cfg, &s_adc);
}

static esp_err_t ensure_ledc_timer(void)
{
    if (s_ledc_timer_ready) return ESP_OK;
    ledc_timer_config_t t = {
        .speed_mode      = AIO_LEDC_MODE,
        .duty_resolution = AIO_LEDC_RES,
        .timer_num       = AIO_LEDC_TIMER,
        .freq_hz         = s_freq_hz[s_freq],
        /* All low-speed LEDC timers share one clock-source mux, so we MUST request
         * the same source the backlight already pinned (bsp_illuminate.c uses
         * LEDC_USE_PLL_DIV_CLK) or ledc_timer_config fails with a clock conflict. */
        .clk_cfg         = LEDC_USE_PLL_DIV_CLK,
    };
    esp_err_t err = ledc_timer_config(&t);
    if (err == ESP_OK) s_ledc_timer_ready = true;
    return err;
}

/* Find a LEDC channel not currently held by any AOUT pin, or -1. */
static int alloc_ledc_channel(void)
{
    for (int ch = AIO_CH_FIRST; ch <= AIO_CH_LAST; ch++) {
        bool used = false;
        for (int i = 0; i < AIO_N; i++) {
            if (s_chan[i] == ch) { used = true; break; }
        }
        if (!used) return ch;
    }
    return -1;
}

/* Release any AOUT/interrupt resources a pin holds and park the GPIO. */
static void release_pin(int i)
{
    if (s_chan[i] >= 0) {
        ledc_stop(AIO_LEDC_MODE, (ledc_channel_t)s_chan[i], 0);
        s_chan[i] = -1;
    }
    if (s_isr_ready) gpio_isr_handler_remove(s_pins[i].gpio);   /* harmless if none */
    gpio_reset_pin(s_pins[i].gpio);   /* detach LEDC/ADC + clear intr, back to plain GPIO */
}

/* Apply the pin's current mode + options to hardware. Assumes resources were released
 * first. Returns an error for impossible requests. */
static esp_err_t configure_pin(int i)
{
    int gpio = s_pins[i].gpio;
    switch (s_mode[i]) {
        case AIO_ANALOG_IN: {
            if (!s_pins[i].adc_ok) return ESP_ERR_NOT_SUPPORTED;
            esp_err_t err = ensure_adc();
            if (err != ESP_OK) return err;
            adc_oneshot_chan_cfg_t cc = { .atten = s_atten_db[s_atten[i]], .bitwidth = ADC_BITWIDTH_DEFAULT };
            return adc_oneshot_config_channel(s_adc, adc_channel_of(i), &cc);
        }
        case AIO_ANALOG_OUT: {
            esp_err_t err = ensure_ledc_timer();
            if (err != ESP_OK) return err;
            int ch = alloc_ledc_channel();
            if (ch < 0) return ESP_ERR_NO_MEM;   /* all PWM channels in use */
            ledc_channel_config_t cc = {
                .gpio_num   = gpio,
                .speed_mode = AIO_LEDC_MODE,
                .channel    = (ledc_channel_t)ch,
                .timer_sel  = AIO_LEDC_TIMER,
                .intr_type  = LEDC_INTR_DISABLE,
                .duty       = duty_raw(s_duty[i]),
                .hpoint     = 0,
            };
            err = ledc_channel_config(&cc);
            if (err == ESP_OK) s_chan[i] = ch;
            return err;
        }
        case AIO_DIGITAL_IN: {
            gpio_config_t c = {
                .pin_bit_mask = 1ULL << gpio, .mode = GPIO_MODE_INPUT,
                .pull_up_en = (s_pull[i] == AIO_PULL_UP), .pull_down_en = (s_pull[i] == AIO_PULL_DOWN),
                .intr_type = irq_to_gpio(s_irq[i]),
            };
            esp_err_t err = gpio_config(&c);
            if (err == ESP_OK && s_irq[i] != AIO_IRQ_OFF && s_isr_ready) {
                gpio_isr_handler_add(gpio, aio_isr, (void *)(intptr_t)i);
            }
            return err;
        }
        case AIO_DIGITAL_OUT: {
            /* Input buffer left on (INPUT_OUTPUT) so the real line is read back; the
             * open-drain variant only pulls LOW and releases HIGH (shared/bus lines). */
            gpio_config_t c = {
                .pin_bit_mask = 1ULL << gpio,
                .mode = s_od[i] ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_INPUT_OUTPUT,
                .pull_up_en = (s_pull[i] == AIO_PULL_UP), .pull_down_en = (s_pull[i] == AIO_PULL_DOWN),
                .intr_type = GPIO_INTR_DISABLE,
            };
            esp_err_t err = gpio_config(&c);
            if (err != ESP_OK) return err;
            gpio_set_level(gpio, s_dout[i] ? 1 : 0);
            gpio_set_drive_capability(gpio, (gpio_drive_cap_t)s_drive[i]);
            return ESP_OK;
        }
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

/* ---- Public API ---------------------------------------------------------- */
esp_err_t bsp_aio_init(void)
{
    esp_err_t isr = gpio_install_isr_service(0);
    s_isr_ready = (isr == ESP_OK || isr == ESP_ERR_INVALID_STATE);
    s_freq = nvs_load_u8("afq", 0, 3);
    if (s_freq >= AIO_FREQ_COUNT) s_freq = 3;

    for (int i = 0; i < AIO_N; i++) {
        s_chan[i]      = -1;
        s_edges[i]     = 0;
        s_ain_acc[i]   = -1.0f;
        s_dout[i]      = nvs_load_u8("o%d", s_pins[i].gpio, 0) ? true : false;
        s_duty[i]      = nvs_load_u8("d%d", s_pins[i].gpio, 0);
        s_volts[i]     = nvs_load_u8("v%d", s_pins[i].gpio, 0) ? true : false;
        s_pull[i]      = nvs_load_u8("pl%d", s_pins[i].gpio, AIO_PULL_NONE);
        s_od[i]        = nvs_load_u8("od%d", s_pins[i].gpio, 0) ? true : false;
        s_drive[i]     = nvs_load_u8("dr%d", s_pins[i].gpio, 2);   /* GPIO_DRIVE_CAP_2 (default) */
        s_irq[i]       = nvs_load_u8("it%d", s_pins[i].gpio, AIO_IRQ_OFF);
        s_atten[i]     = nvs_load_u8("at%d", s_pins[i].gpio, AIO_ATTEN_COUNT - 1);  /* widest range */
        s_filt[i]      = nvs_load_u8("fl%d", s_pins[i].gpio, 0);
        if (s_duty[i] > 100) s_duty[i] = 100;
        if (s_pull[i] >= AIO_PULL_COUNT)   s_pull[i] = AIO_PULL_NONE;
        if (s_drive[i] > 3)  s_drive[i] = 2;
        if (s_irq[i] >= AIO_IRQ_COUNT) s_irq[i] = AIO_IRQ_OFF;
        if (s_atten[i] >= AIO_ATTEN_COUNT) s_atten[i] = AIO_ATTEN_COUNT - 1;
        if (s_filt[i] >= AIO_FILT_COUNT) s_filt[i] = 0;

        if (nvs_has("md%d", s_pins[i].gpio)) {
            s_mode[i] = (aio_mode_t)nvs_load_u8("md%d", s_pins[i].gpio, AIO_DIGITAL_IN);
            if (s_mode[i] >= AIO_MODE_COUNT ||
                (s_mode[i] == AIO_ANALOG_IN && !s_pins[i].adc_ok)) {
                s_mode[i] = AIO_DIGITAL_IN;
            }
        } else {
            /* First boot / wiped NVS: seed a sensible bench — ADC pins as analog inputs,
             * the rest as digital inputs with a pull-down. Every pin is an input, so the
             * bench reads immediately and nothing is driven. Persist it. */
            if (s_pins[i].adc_ok) {
                s_mode[i] = AIO_ANALOG_IN;
            } else {
                s_mode[i] = AIO_DIGITAL_IN;
                s_pull[i] = AIO_PULL_DOWN;
                nvs_save_u8("pl%d", s_pins[i].gpio, AIO_PULL_DOWN);
            }
            nvs_save_u8("md%d", s_pins[i].gpio, (uint8_t)s_mode[i]);
        }

        esp_err_t err = configure_pin(i);
        if (err != ESP_OK) {
            ESP_LOGW(AIO_TAG, "%s restore mode %d failed (%s) -> DIGITAL_IN",
                     s_pins[i].label, s_mode[i], esp_err_to_name(err));
            s_mode[i] = AIO_DIGITAL_IN;
            configure_pin(i);
        }
    }
    ESP_LOGI(AIO_TAG, "I/O bench up: %d pins", AIO_N);
    return ESP_OK;
}

int bsp_aio_count(void) { return AIO_N; }

const aio_pin_t *bsp_aio_info(int idx) { return valid(idx) ? &s_pins[idx] : NULL; }

aio_mode_t bsp_aio_get_mode(int idx) { return valid(idx) ? s_mode[idx] : AIO_DIGITAL_IN; }

esp_err_t bsp_aio_set_mode(int idx, aio_mode_t mode)
{
    if (!valid(idx) || mode >= AIO_MODE_COUNT) return ESP_ERR_INVALID_ARG;
    if (mode == AIO_ANALOG_IN && !s_pins[idx].adc_ok) return ESP_ERR_NOT_SUPPORTED;

    aio_mode_t prev = s_mode[idx];
    release_pin(idx);
    s_mode[idx] = mode;
    s_ain_acc[idx] = -1.0f;            /* restart the analog filter on a mode change */
    esp_err_t err = configure_pin(idx);
    if (err != ESP_OK) {
        s_mode[idx] = prev;            /* roll back; never leave a half-configured pin */
        release_pin(idx);
        configure_pin(idx);
        ESP_LOGW(AIO_TAG, "%s set mode %d failed: %s",
                 s_pins[idx].label, mode, esp_err_to_name(err));
        return err;
    }
    nvs_save_u8("md%d", s_pins[idx].gpio, (uint8_t)mode);
    return ESP_OK;
}

aio_pull_t bsp_aio_get_pull(int idx) { return valid(idx) ? (aio_pull_t)s_pull[idx] : AIO_PULL_NONE; }
bool       bsp_aio_get_od(int idx)   { return valid(idx) ? s_od[idx] : false; }

esp_err_t bsp_aio_set_pull(int idx, aio_pull_t pull)
{
    if (!valid(idx) || pull >= AIO_PULL_COUNT) return ESP_ERR_INVALID_ARG;
    s_pull[idx] = (uint8_t)pull;
    nvs_save_u8("pl%d", s_pins[idx].gpio, (uint8_t)pull);
    if (aio_is_digital(s_mode[idx])) return gpio_set_pull_mode(s_pins[idx].gpio, pull_mode_of(idx));
    return ESP_OK;
}
esp_err_t bsp_aio_set_od(int idx, bool on)
{
    if (!valid(idx)) return ESP_ERR_INVALID_ARG;
    s_od[idx] = on;
    nvs_save_u8("od%d", s_pins[idx].gpio, on ? 1 : 0);
    if (s_mode[idx] == AIO_DIGITAL_OUT)   /* re-apply the GPIO mode (push-pull <-> OD) */
        return gpio_set_direction(s_pins[idx].gpio,
                                  on ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_INPUT_OUTPUT);
    return ESP_OK;
}

int bsp_aio_get_drive(int idx) { return valid(idx) ? s_drive[idx] : 2; }

esp_err_t bsp_aio_set_drive(int idx, int cap)
{
    if (!valid(idx)) return ESP_ERR_INVALID_ARG;
    if (cap < 0) cap = 0;
    if (cap > 3) cap = 3;
    s_drive[idx] = (uint8_t)cap;
    nvs_save_u8("dr%d", s_pins[idx].gpio, (uint8_t)cap);
    if (aio_is_output(s_mode[idx])) return gpio_set_drive_capability(s_pins[idx].gpio, (gpio_drive_cap_t)cap);
    return ESP_OK;
}

aio_irq_t bsp_aio_get_irq(int idx) { return valid(idx) ? (aio_irq_t)s_irq[idx] : AIO_IRQ_OFF; }

esp_err_t bsp_aio_set_irq(int idx, aio_irq_t irq)
{
    if (!valid(idx) || irq >= AIO_IRQ_COUNT) return ESP_ERR_INVALID_ARG;
    s_irq[idx] = (uint8_t)irq;
    nvs_save_u8("it%d", s_pins[idx].gpio, (uint8_t)irq);
    if (s_mode[idx] != AIO_DIGITAL_IN) return ESP_OK;   /* interrupts only on inputs */
    int gpio = s_pins[idx].gpio;
    gpio_set_intr_type(gpio, irq_to_gpio(irq));
    if (irq != AIO_IRQ_OFF) {
        if (s_isr_ready) gpio_isr_handler_add(gpio, aio_isr, (void *)(intptr_t)idx);
        gpio_intr_enable(gpio);
    } else {
        if (s_isr_ready) gpio_isr_handler_remove(gpio);
        gpio_intr_disable(gpio);
    }
    return ESP_OK;
}

uint32_t bsp_aio_get_edges(int idx) { return valid(idx) ? s_edges[idx] : 0; }
void     bsp_aio_reset_edges(int idx) { if (valid(idx)) s_edges[idx] = 0; }

int bsp_aio_read_level(int idx)
{
    if (!valid(idx) || !aio_is_digital(s_mode[idx])) return -1;
    return gpio_get_level(s_pins[idx].gpio);   /* read-back is on for outputs too */
}

bool bsp_aio_get_dout(int idx) { return valid(idx) ? s_dout[idx] : false; }

esp_err_t bsp_aio_set_dout(int idx, bool on)
{
    if (!valid(idx) || s_mode[idx] != AIO_DIGITAL_OUT) return ESP_ERR_INVALID_STATE;
    s_dout[idx] = on;
    esp_err_t err = gpio_set_level(s_pins[idx].gpio, on ? 1 : 0);
    nvs_save_u8("o%d", s_pins[idx].gpio, on ? 1 : 0);
    return err;
}

int   bsp_aio_get_atten(int idx)  { return valid(idx) ? s_atten[idx] : (AIO_ATTEN_COUNT - 1); }
int   bsp_aio_get_filter(int idx) { return valid(idx) ? s_filt[idx] : 0; }
float bsp_aio_ain_vmax(int idx)   { return valid(idx) ? s_atten_vmax[s_atten[idx]] : AIO_VREF; }

esp_err_t bsp_aio_set_atten(int idx, int atten)
{
    if (!valid(idx) || atten < 0 || atten >= AIO_ATTEN_COUNT) return ESP_ERR_INVALID_ARG;
    s_atten[idx] = (uint8_t)atten;
    s_ain_acc[idx] = -1.0f;            /* range change -> restart the filter */
    nvs_save_u8("at%d", s_pins[idx].gpio, (uint8_t)atten);
    if (s_mode[idx] == AIO_ANALOG_IN && s_adc) {
        adc_oneshot_chan_cfg_t cc = { .atten = s_atten_db[atten], .bitwidth = ADC_BITWIDTH_DEFAULT };
        return adc_oneshot_config_channel(s_adc, adc_channel_of(idx), &cc);
    }
    return ESP_OK;
}

esp_err_t bsp_aio_set_filter(int idx, int filter)
{
    if (!valid(idx) || filter < 0 || filter >= AIO_FILT_COUNT) return ESP_ERR_INVALID_ARG;
    s_filt[idx] = (uint8_t)filter;
    s_ain_acc[idx] = -1.0f;
    nvs_save_u8("fl%d", s_pins[idx].gpio, (uint8_t)filter);
    return ESP_OK;
}

esp_err_t bsp_aio_read_ain(int idx, int *raw, int *pct)
{
    if (!valid(idx) || s_mode[idx] != AIO_ANALOG_IN || !s_adc) return ESP_ERR_INVALID_STATE;
    int v = 0;
    esp_err_t err = adc_oneshot_read(s_adc, adc_channel_of(idx), &v);
    if (err != ESP_OK) return err;
    /* Single-pole IIR (exponential moving average). filter 0 -> alpha 1.0 = passthrough. */
    float a = s_filt_alpha[s_filt[idx]];
    if (s_ain_acc[idx] < 0.0f || a >= 1.0f) s_ain_acc[idx] = (float)v;
    else s_ain_acc[idx] += a * ((float)v - s_ain_acc[idx]);
    int out = (int)(s_ain_acc[idx] + 0.5f);
    if (raw) *raw = out;
    if (pct) *pct = out * 100 / 4095;
    return ESP_OK;
}

int      bsp_aio_get_aout(int idx) { return valid(idx) ? s_duty[idx] : 0; }
int      bsp_aio_get_freq(void)    { return s_freq; }
uint32_t bsp_aio_freq_hz(int idx)  { return (idx >= 0 && idx < AIO_FREQ_COUNT) ? s_freq_hz[idx] : 0; }

esp_err_t bsp_aio_set_aout(int idx, int pct)
{
    if (!valid(idx) || s_mode[idx] != AIO_ANALOG_OUT || s_chan[idx] < 0) return ESP_ERR_INVALID_STATE;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    s_duty[idx] = (uint8_t)pct;
    esp_err_t err = ledc_set_duty(AIO_LEDC_MODE, (ledc_channel_t)s_chan[idx], duty_raw(pct));
    if (err == ESP_OK) err = ledc_update_duty(AIO_LEDC_MODE, (ledc_channel_t)s_chan[idx]);
    nvs_save_u8("d%d", s_pins[idx].gpio, (uint8_t)pct);
    return err;
}

esp_err_t bsp_aio_set_freq(int idx)
{
    if (idx < 0 || idx >= AIO_FREQ_COUNT) return ESP_ERR_INVALID_ARG;
    s_freq = (uint8_t)idx;
    nvs_save_u8("afq", 0, (uint8_t)idx);
    if (s_ledc_timer_ready) return ledc_set_freq(AIO_LEDC_MODE, AIO_LEDC_TIMER, s_freq_hz[idx]);
    return ESP_OK;
}

bool bsp_aio_get_volts_pref(int idx) { return valid(idx) ? s_volts[idx] : false; }

void bsp_aio_set_volts_pref(int idx, bool volts)
{
    if (!valid(idx)) return;
    s_volts[idx] = volts;
    nvs_save_u8("v%d", s_pins[idx].gpio, volts ? 1 : 0);
}
