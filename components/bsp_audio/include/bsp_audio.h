#ifndef _BSP_AUDIO_H_
#define _BSP_AUDIO_H_

/* bsp_audio — I2S speaker output for the on-board amplifier.
 *
 * The P4 drives an external power amp (no codec) over I2S: BCLK IO22, LRCLK/WS IO21,
 * SDATA/DOUT IO23, with the amp enable on IO30 (ACTIVE-LOW: low = sound on).  This BSP
 * uses I2S controller 1 for TX; the PDM microphone (prop_mic) owns controller 0, so the
 * two coexist.  16 kHz / 16-bit / mono — matching the synth in prop_audio and keeping the
 * DMA traffic small.  Adapted from example V1.2 Lesson12 bsp_audio (file playback trimmed). */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Audio output pins (root readme.md hardware map). */
#define AUDIO_GPIO_LRCLK    21   /* word select (left/right clock) */
#define AUDIO_GPIO_BCLK     22   /* bit clock */
#define AUDIO_GPIO_SDATA    23   /* serial data out */
#define AUDIO_GPIO_CTRL     30   /* amp enable (active-low) */

#define BSP_AUDIO_SAMPLE_HZ 16000

/* Configure the amp-enable GPIO (amp left OFF) and bring up the I2S-std TX channel.
 * Returns an error without leaving the channel half-open on failure. */
esp_err_t bsp_audio_init(void);

/* Enable/disable the speaker amplifier (handles the active-low inversion). */
void bsp_audio_amp(bool on);

/* Write `samples` mono 16-bit PCM frames to I2S, blocking until queued. */
esp_err_t bsp_audio_write(const int16_t *pcm, size_t samples);

#endif /* _BSP_AUDIO_H_ */
