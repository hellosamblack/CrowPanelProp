#ifndef _PROP_MIC_H_
#define _PROP_MIC_H_

/* prop_mic — PDM mic or ADC channel capture + FFT, feeding the SPECTRUM instrument.
 *
 * The default source is the board's PDM digital mic (I2S0, CLK GPIO24, DATA GPIO26,
 * 16 kHz/16-bit/mono).  The source can be switched at runtime to any of the six
 * ADC-capable header/comm pins (IO49..IO54, ADC2 channels 0..5) — sampled at ~1 kHz
 * via bsp_aio, giving a 0–500 Hz spectrum.
 *
 * The FFT task always starts; PDM failure leaves ADC sources still functional.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_MIC_BANDS 24

/* FFT input source.  MIC is the default; ADC0-5 target IO49..IO54. */
typedef enum {
    SPEC_SRC_MIC = 0,   /* PDM digital microphone */
    SPEC_SRC_ADC0,      /* IO49 — ADC2 ch0 */
    SPEC_SRC_ADC1,      /* IO50 — ADC2 ch1 */
    SPEC_SRC_ADC2,      /* IO51 — ADC2 ch2 */
    SPEC_SRC_ADC3,      /* IO52 — ADC2 ch3 */
    SPEC_SRC_ADC4,      /* IO53 — ADC2 ch4 */
    SPEC_SRC_ADC5,      /* IO54 — ADC2 ch5 */
    SPEC_SRC_COUNT,
} spec_src_t;

/* Bring up the spectrum subsystem.  Always starts the FFT task; PDM init is
 * non-fatal — prop_mic_pdm_up() reports whether the hardware came up. */
esp_err_t prop_mic_init(void);

/* True once the FFT task is running and bands are being produced. */
bool prop_mic_available(void);

/* True if the PDM microphone hardware was successfully brought up. */
bool prop_mic_pdm_up(void);

/* Switch the FFT input source (safe to call from any task). */
void       prop_mic_set_source(spec_src_t src);
spec_src_t prop_mic_get_source(void);

/* Sample rate (Hz) for the current source: 16000 (MIC) or 1000 (ADC). */
int prop_mic_sample_rate(void);

/* Center frequency (Hz) of spectrum band b for the current source. */
int prop_mic_band_hz(int b);

/* Copy the latest PROP_MIC_BANDS spectrum magnitudes (0..100) into out. */
void prop_mic_get_bands(uint8_t *out);

/* Latest level, roughly -60..0 dBFS (0 = loud). */
int prop_mic_get_db(void);

#endif /* _PROP_MIC_H_ */
