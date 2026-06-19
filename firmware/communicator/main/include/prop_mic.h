#ifndef _PROP_MIC_H_
#define _PROP_MIC_H_

/* prop_mic — PDM microphone capture + FFT, feeding the SPECTRUM instrument.
 *
 * The board's mic is a PDM digital mic straight into the ESP32-P4 I2S0 (CLK
 * GPIO24, DATA GPIO26, 16 kHz/16-bit/mono — see the board-mic-path memory and
 * Lesson11 bsp_mic). A background task reads blocks, runs a radix-2 FFT (working
 * set in PSRAM), and caches a small set of spectrum bands + a dB level for the UI
 * to read cheaply (mirrors the prop_net_get_rssi cached-poll pattern).
 *
 * Non-fatal: if the mic can't be brought up, prop_mic_available() stays false and
 * the SPECTRUM panel shows an offline state — the prop never hangs on audio.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PROP_MIC_BANDS 24

/* Bring up PDM RX + start the capture/FFT task. Returns an error (non-fatal) if
 * the mic is unavailable. */
esp_err_t prop_mic_init(void);

bool prop_mic_available(void);

/* Copy the latest PROP_MIC_BANDS spectrum magnitudes (0..100) into out. */
void prop_mic_get_bands(uint8_t *out);

/* Latest level, roughly -60..0 dBFS (0 = loud). */
int prop_mic_get_db(void);

#endif /* _PROP_MIC_H_ */
