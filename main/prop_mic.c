/* prop_mic — PDM mic capture + ADC sampling + radix-2 FFT → cached spectrum bands.
 * See prop_mic.h for the public API.
 *
 * The mic task always runs.  On each iteration it reads FFT_N samples from whichever
 * source is selected (PDM I2S or a bsp_aio ADC pin), windows + FFTs them, folds into
 * 24 log-spaced bands, and caches the result for the UI to poll cheaply.
 *
 * ADC path: samples IO49-54 at ~1 kHz using vTaskDelayUntil (FreeRTOS tick = 1 ms),
 * giving a 0-500 Hz spectrum.  Samples are centred and scaled to ±int16 range so the
 * same normalisation constant (32768) works for both sources.
 */
#include "prop_mic.h"
#include "bsp_aio.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/i2s_pdm.h"

#define MIC_TAG "PROP_MIC"

#define MIC_CLK_GPIO   24
#define MIC_DIN_GPIO   26
#define MIC_SAMPLE_HZ  16000

#define FFT_N    256                 /* 16 ms/block @ 16 kHz → ~62 FFT/s for MIC */
#define N_BINS   (FFT_N / 2)         /* usable magnitude bins */

static i2s_chan_handle_t       s_rx;
static bool                    s_pdm_up;
static bool                    s_task_up;
static volatile spec_src_t     s_source = SPEC_SRC_MIC;

/* FFT working set + cached output (PSRAM — see Memory reality in CLAUDE.md). */
static float   *s_re, *s_im, *s_win;
static int16_t *s_raw;
static uint8_t  s_bands[PROP_MIC_BANDS];
static int      s_db = -60;

/* In-place iterative radix-2 DIT FFT. */
static void fft(float *re, float *im, int n)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / len;
        float wr = cosf(ang), wi = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                float a = s_re[i + k + len / 2], b = s_im[i + k + len / 2];
                float vr = a * cr - b * ci, vi = a * ci + b * cr;
                float ur = s_re[i + k], ui = s_im[i + k];
                s_re[i + k] = ur + vr; s_im[i + k] = ui + vi;
                s_re[i + k + len / 2] = ur - vr; s_im[i + k + len / 2] = ui - vi;
                float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr; cr = ncr;
            }
        }
    }
}

/* Fold the magnitude spectrum into PROP_MIC_BANDS log-spaced bands (0..100). */
static void compute_bands(void)
{
    for (int b = 0; b < PROP_MIC_BANDS; b++) {
        float f0 = powf((float)(N_BINS - 1), (float)b / PROP_MIC_BANDS);
        float f1 = powf((float)(N_BINS - 1), (float)(b + 1) / PROP_MIC_BANDS);
        int lo = 1 + (int)f0, hi = 1 + (int)f1;
        if (hi <= lo) hi = lo + 1;
        if (hi > N_BINS) hi = N_BINS;

        float peak = 0.0f;
        for (int k = lo; k < hi; k++) {
            float m = s_re[k] * s_re[k] + s_im[k] * s_im[k];
            if (m > peak) peak = m;
        }
        /* Normalise to full scale, then dB, then an aesthetic 0..100 window that
         * puts a quiet room's noise floor mid-screen and speech/claps at the top. */
        float mag = sqrtf(peak) / (FFT_N * 0.5f) / 32768.0f;
        float db  = 20.0f * log10f(mag + 1e-6f);
        int v = (int)((db + 90.0f) * 1.67f);   /* -90 dB → 0, -30 dB → 100 */
        if (v < 4)   v = 4;
        if (v > 100) v = 100;
        s_bands[b] = (uint8_t)v;
    }
}

/* Return the bsp_aio pin index whose GPIO matches target, or -1. */
static int find_aio_idx(int gpio)
{
    int n = bsp_aio_count();
    for (int i = 0; i < n; i++) {
        const aio_pin_t *p = bsp_aio_info(i);
        if (p && p->gpio == gpio) return i;
    }
    return -1;
}

static void mic_task(void *arg)
{
    (void)arg;
    while (1) {
        spec_src_t src = s_source;

        if (src == SPEC_SRC_MIC) {
            if (!s_pdm_up) {
                /* PDM unavailable — produce silence so ADC switch still works. */
                memset(s_raw, 0, FFT_N * sizeof(int16_t));
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                size_t got = 0;
                if (i2s_channel_read(s_rx, s_raw, FFT_N * sizeof(int16_t), &got,
                                     pdMS_TO_TICKS(200)) != ESP_OK ||
                    got < FFT_N * sizeof(int16_t)) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    continue;
                }
            }
        } else {
            /* ADC source: IO49..54 → ADC2 ch0..5. */
            int gpio = 49 + (int)(src - SPEC_SRC_ADC0);
            int aio  = find_aio_idx(gpio);
            if (aio < 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (bsp_aio_get_mode(aio) != AIO_ANALOG_IN) {
                bsp_aio_set_mode(aio, AIO_ANALOG_IN);
            }
            /* Sample at 1 kHz (1 tick = 1 ms at CONFIG_FREERTOS_HZ=1000).
             * 256 samples → 256 ms/frame (~4 FFT/s), 0-500 Hz Nyquist. */
            TickType_t wake = xTaskGetTickCount();
            for (int i = 0; i < FFT_N; i++) {
                int raw = 2048;
                bsp_aio_read_ain(aio, &raw, NULL);
                /* Centre 0..4095 around 0, scale to ±int16 range. */
                s_raw[i] = (int16_t)((raw - 2048) * 16);
                vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
            }
        }

        /* Remove DC offset before FFT.  Critical for ADC sources (which sit at a
         * static bias far from zero); also removes any PDM mic DC residual. */
        int32_t dc_sum = 0;
        for (int i = 0; i < FFT_N; i++) dc_sum += s_raw[i];
        int16_t dc = (int16_t)(dc_sum / FFT_N);

        /* Common path: Hann window → FFT → band-fold → level meter. */
        double sumsq = 0.0;
        for (int i = 0; i < FFT_N; i++) {
            float s = (float)(s_raw[i] - dc);
            sumsq  += (double)s * s;
            s_re[i] = s * s_win[i];
            s_im[i] = 0.0f;
        }
        fft(s_re, s_im, FFT_N);
        compute_bands();

        float rms = sqrtf((float)(sumsq / FFT_N));
        int db = (int)(20.0f * log10f(rms / 32768.0f + 1e-6f));
        if (db < -60) db = -60;
        if (db >   0) db =   0;
        s_db = db;
    }
}

esp_err_t prop_mic_init(void)
{
    s_re  = heap_caps_malloc(sizeof(float)   * FFT_N, MALLOC_CAP_SPIRAM);
    s_im  = heap_caps_malloc(sizeof(float)   * FFT_N, MALLOC_CAP_SPIRAM);
    s_win = heap_caps_malloc(sizeof(float)   * FFT_N, MALLOC_CAP_SPIRAM);
    s_raw = heap_caps_malloc(sizeof(int16_t) * FFT_N, MALLOC_CAP_SPIRAM);
    if (!s_re || !s_im || !s_win || !s_raw) {
        ESP_LOGE(MIC_TAG, "no PSRAM for FFT buffers");
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < FFT_N; i++) {   /* Hann window */
        s_win[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (FFT_N - 1)));
    }

    /* Try PDM mic — non-fatal if it fails (ADC sources remain available). */
    i2s_chan_config_t rx_cfg = {
        .id                  = I2S_NUM_0,
        .role                = I2S_ROLE_MASTER,
        .dma_desc_num        = 4,
        .dma_frame_num       = 256,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = true,
        .intr_priority       = 0,
    };
    if (i2s_new_channel(&rx_cfg, NULL, &s_rx) == ESP_OK) {
        i2s_pdm_rx_config_t pdm_cfg = {
            .clk_cfg = {
                .sample_rate_hz = MIC_SAMPLE_HZ,
                .clk_src        = I2S_CLK_SRC_DEFAULT,
                .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
                .dn_sample_mode = I2S_PDM_DSR_8S,
                .bclk_div       = 8,
            },
            .slot_cfg = {
                .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode      = I2S_SLOT_MODE_MONO,
                .slot_mask      = I2S_PDM_SLOT_LEFT,
                .hp_en          = true,
                .hp_cut_off_freq_hz = 35.5,
                .amplify_num    = 1,
            },
            .gpio_cfg = {
                .clk          = MIC_CLK_GPIO,
                .din          = MIC_DIN_GPIO,
                .invert_flags = { .clk_inv = false },
            },
        };
        esp_err_t err = i2s_channel_init_pdm_rx_mode(s_rx, &pdm_cfg);
        if (err == ESP_OK) err = i2s_channel_enable(s_rx);
        if (err != ESP_OK) {
            ESP_LOGW(MIC_TAG, "PDM RX init: %s (mic offline)", esp_err_to_name(err));
            i2s_del_channel(s_rx);
            s_rx = NULL;
        } else {
            s_pdm_up = true;
        }
    } else {
        s_rx = NULL;
    }

    if (xTaskCreatePinnedToCore(mic_task, "prop_mic", 4096, NULL, 5, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_task_up = true;
    ESP_LOGI(MIC_TAG, "spectrum task up — PDM %s, ADC sources ready",
             s_pdm_up ? "ok" : "offline");
    return ESP_OK;
}

bool prop_mic_available(void) { return s_task_up; }
bool prop_mic_pdm_up(void)    { return s_pdm_up;  }

int prop_mic_sample_rate(void)
{
    return (s_source == SPEC_SRC_MIC) ? MIC_SAMPLE_HZ : 1000;
}

int prop_mic_band_hz(int b)
{
    int sr = prop_mic_sample_rate();
    float cb = powf((float)(N_BINS - 1), (b + 0.5f) / PROP_MIC_BANDS);
    return (int)(cb * sr / FFT_N + 0.5f);
}

void prop_mic_set_source(spec_src_t src)
{
    if (src >= SPEC_SRC_COUNT) src = SPEC_SRC_MIC;
    s_source = src;
}

spec_src_t prop_mic_get_source(void) { return s_source; }

void prop_mic_get_bands(uint8_t *out)
{
    if (out) memcpy(out, s_bands, PROP_MIC_BANDS);   /* benign race: it's a meter */
}

int prop_mic_get_db(void) { return s_db; }
