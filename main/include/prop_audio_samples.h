#ifndef _PROP_AUDIO_SAMPLES_H_
#define _PROP_AUDIO_SAMPLES_H_

/* Embedded PCM assets for the SONAR/TORPEDO ping voices — real recordings
 * (Hunt for Red October's active sonar / torpedo-acquisition pings) instead
 * of synthesized approximations. Mono 16-bit, 16 kHz (matches BSP_AUDIO_
 * SAMPLE_HZ), peak-normalized to ~32000. Generated from the WAVs under
 * docs/design/audio-reference/ — see the header comment in
 * prop_audio_sonar_pcm.c / prop_audio_torpedo_pcm.c for the conversion.
 * prop_audio.c reads these directly (no copy) as one of its two voice source
 * kinds — see VSRC_SAMPLE in s_ping_voices. */

#include <stdint.h>
#include <stddef.h>

extern const int16_t prop_audio_sonar_pcm[];
extern const size_t  prop_audio_sonar_pcm_len;

extern const int16_t prop_audio_torpedo_pcm[];
extern const size_t  prop_audio_torpedo_pcm_len;

#endif /* _PROP_AUDIO_SAMPLES_H_ */
