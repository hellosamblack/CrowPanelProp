/* prop_mic — PDM mic capture + radix-2 FFT → cached spectrum bands. See prop_mic.h. */
#include "prop_mic.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/i2s_pdm.h"

#define MIC_TAG "PROP_MIC"

/* Board mic (confirmed via Lesson11 bsp_mic / board-mic-path memory). */
#define MIC_CLK_GPIO   24
#define MIC_DIN_GPIO   26
#define MIC_SAMPLE_HZ  16000

#define FFT_N    256                 /* 16 ms/block @ 16 kHz → ~62 FFT/s */
#define N_BINS   (FFT_N / 2)         /* usable magnitude bins (0..8 kHz) */

static i2s_chan_handle_t s_rx;
static bool s_available;

/* FFT working set + cached output (working set in PSRAM per the memory budget). */
static float *s_re, *s_im, *s_win;
static int16_t *s_raw;
static uint8_t s_bands[PROP_MIC_BANDS];
static int s_db = -60;

/* In-place iterative radix-2 DIT FFT. */
static void fft(float *re, float *im, int n)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
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
                float a = re[i + k + len / 2], b = im[i + k + len / 2];
                float vr = a * cr - b * ci;
                float vi = a * ci + b * cr;
                float ur = re[i + k], ui = im[i + k];
                re[i + k] = ur + vr; im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
                float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

/* Fold the magnitude spectrum into PROP_MIC_BANDS log-spaced bands (0..100). */
static void compute_bands(void)
{
    for (int b = 0; b < PROP_MIC_BANDS; b++) {
        /* Log-spaced bin ranges across bins 1..N_BINS-1. */
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
        /* Normalise to full scale (0..1), then dB, then a 0..100 window tuned so
         * the room noise floor sits low and speech/claps drive the bars up. */
        float mag = sqrtf(peak) / (FFT_N * 0.5f) / 32768.0f;
        float db = 20.0f * log10f(mag + 1e-6f);
        /* Aesthetic-first window: a quiet room's noise floor already dances around
         * mid-screen (per-band variation reads as "alive" on camera) and speech /
         * claps push bars to full. Real data, dramatic mapping. */
        int v = (int)((db + 90.0f) * 1.67f);   /* -90 dB → 0, -30 dB → 100 */
        if (v < 4) v = 4;
        if (v > 100) v = 100;
        s_bands[b] = (uint8_t)v;
    }
}

static void mic_task(void *arg)
{
    (void)arg;
    while (1) {
        size_t got = 0;
        if (i2s_channel_read(s_rx, s_raw, FFT_N * sizeof(int16_t), &got,
                             pdMS_TO_TICKS(200)) != ESP_OK || got < FFT_N * sizeof(int16_t)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        /* Window + load into the FFT buffers; accumulate RMS for the dB meter. */
        double sumsq = 0.0;
        for (int i = 0; i < FFT_N; i++) {
            float s = (float)s_raw[i];
            sumsq += (double)s * s;
            s_re[i] = s * s_win[i];
            s_im[i] = 0.0f;
        }
        fft(s_re, s_im, FFT_N);
        compute_bands();

        float rms = sqrtf((float)(sumsq / FFT_N));
        int db = (int)(20.0f * log10f(rms / 32768.0f + 1e-6f));
        if (db < -60) db = -60;
        if (db > 0) db = 0;
        s_db = db;
    }
}

esp_err_t prop_mic_init(void)
{
    s_re = heap_caps_malloc(sizeof(float) * FFT_N, MALLOC_CAP_SPIRAM);
    s_im = heap_caps_malloc(sizeof(float) * FFT_N, MALLOC_CAP_SPIRAM);
    s_win = heap_caps_malloc(sizeof(float) * FFT_N, MALLOC_CAP_SPIRAM);
    s_raw = heap_caps_malloc(sizeof(int16_t) * FFT_N, MALLOC_CAP_SPIRAM);
    if (!s_re || !s_im || !s_win || !s_raw) {
        ESP_LOGE(MIC_TAG, "no PSRAM for FFT buffers");
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < FFT_N; i++) {   /* Hann window */
        s_win[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (FFT_N - 1)));
    }

    i2s_chan_config_t rx_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 4,
        .dma_frame_num = 256,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = true,
        .intr_priority = 0,
    };
    esp_err_t err = i2s_new_channel(&rx_cfg, NULL, &s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(MIC_TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = MIC_SAMPLE_HZ,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .dn_sample_mode = I2S_PDM_DSR_8S,
            .bclk_div = 8,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_PDM_SLOT_LEFT,
            .hp_en = true,
            .hp_cut_off_freq_hz = 35.5,
            .amplify_num = 1,
        },
        .gpio_cfg = {
            .clk = MIC_CLK_GPIO,
            .din = MIC_DIN_GPIO,
            .invert_flags = { .clk_inv = false },
        },
    };
    err = i2s_channel_init_pdm_rx_mode(s_rx, &pdm_cfg);
    if (err == ESP_OK) {
        err = i2s_channel_enable(s_rx);
    }
    if (err != ESP_OK) {
        ESP_LOGE(MIC_TAG, "PDM RX init: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx);
        s_rx = NULL;
        return err;
    }

    if (xTaskCreate(mic_task, "prop_mic", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_available = true;
    ESP_LOGI(MIC_TAG, "PDM mic up (CLK%d DIN%d, %d Hz)", MIC_CLK_GPIO, MIC_DIN_GPIO, MIC_SAMPLE_HZ);
    return ESP_OK;
}

bool prop_mic_available(void) { return s_available; }

void prop_mic_get_bands(uint8_t *out)
{
    if (out) {
        memcpy(out, s_bands, PROP_MIC_BANDS);   /* benign race: it's a meter */
    }
}

int prop_mic_get_db(void) { return s_db; }
