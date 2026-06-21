/* bsp_audio — I2S TX to the on-board amp. See bsp_audio.h. */
#include "bsp_audio.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

#define AUDIO_TAG "BSP_AUDIO"

static i2s_chan_handle_t s_tx;

static esp_err_t amp_gpio_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << AUDIO_GPIO_CTRL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

void bsp_audio_amp(bool on)
{
    /* Active-low: drive the line low to turn the amp on, high to mute. */
    gpio_set_level(AUDIO_GPIO_CTRL, on ? 0 : 1);
}

esp_err_t bsp_audio_init(void)
{
    esp_err_t err = amp_gpio_init();
    if (err != ESP_OK) {
        ESP_LOGE(AUDIO_TAG, "amp gpio: %s", esp_err_to_name(err));
        return err;
    }
    bsp_audio_amp(false);   /* start muted */

    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_1,                /* I2S0 is the PDM mic; speaker uses I2S1 */
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 256,
        .auto_clear = true,             /* zero-fill on underrun (no buzzing tail) */
        .intr_priority = 0,
    };
    err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(AUDIO_TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = BSP_AUDIO_SAMPLE_HZ,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AUDIO_GPIO_BCLK,
            .ws = AUDIO_GPIO_LRCLK,
            .dout = AUDIO_GPIO_SDATA,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    err = i2s_channel_init_std_mode(s_tx, &std_cfg);
    if (err == ESP_OK) {
        err = i2s_channel_enable(s_tx);
    }
    if (err != ESP_OK) {
        ESP_LOGE(AUDIO_TAG, "i2s std init/enable: %s", esp_err_to_name(err));
        i2s_del_channel(s_tx);
        s_tx = NULL;
        return err;
    }

    ESP_LOGI(AUDIO_TAG, "I2S1 TX up (BCLK%d WS%d DOUT%d, amp IO%d, %d Hz mono)",
             AUDIO_GPIO_BCLK, AUDIO_GPIO_LRCLK, AUDIO_GPIO_SDATA, AUDIO_GPIO_CTRL,
             BSP_AUDIO_SAMPLE_HZ);
    return ESP_OK;
}

esp_err_t bsp_audio_write(const int16_t *pcm, size_t samples)
{
    if (!s_tx || !pcm || samples == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t written = 0;
    return i2s_channel_write(s_tx, pcm, samples * sizeof(int16_t), &written, portMAX_DELAY);
}
