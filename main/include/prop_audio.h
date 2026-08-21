#ifndef _PROP_AUDIO_H_
#define _PROP_AUDIO_H_

/* prop_audio — synthesized action-feedback tones over the I2S speaker amp.
 *
 * A tiny tone synth (square/sine/noise + attack-decay envelope) renders short
 * cassette-futurism bleeps per UI/scene event into a PSRAM buffer streamed to I2S on
 * a dedicated task. No asset files. Volume/mute come from prop_settings (audio_vol,
 * audio_mute) and are read per event, so changes apply immediately. The amp is enabled
 * on the first sound and powered down after a short idle grace to avoid switching pops.
 *
 * prop_audio_play() is non-blocking (it only enqueues) and safe to call from any task,
 * including under lvgl_port_lock() — though callers prefer to play outside the lock. */

#include "esp_err.h"

/* Feedback events (see the sound table in prop_audio.c). */
typedef enum {
    PA_DIAL_TICK = 0,   /* selector rotation — short noise click */
    PA_SCREEN,          /* screen/panel change — mechanical clack */
    PA_BUTTON,          /* button click — crisp high tick */
    PA_SLIDER,          /* slider drag step — subtle short tick */
    PA_OPEN,            /* open/confirm — rising 2-tone chirp */
    PA_BACK,            /* step back — single low blip */
    PA_TAB,             /* tab switch — brief click */
    PA_DENY,            /* rejected input — harsh square buzz */
    PA_SIGNAL,          /* SIGNAL DETECTED — bright 3-note sting */
    PA_ALERT,           /* ALERT — square klaxon */
    PA_BOOT,            /* startup chime */
    PA_PING,            /* radar range ping — voice selectable, see prop_audio_ping_voice_* below */
    PA_EVENT_COUNT,
} prop_audio_event_t;

/* Bring up the amp/I2S (bsp_audio) and start the synth task. NON-fatal to the caller:
 * on failure the prop runs silently and prop_audio_play() becomes a no-op. */
esp_err_t prop_audio_init(void);

/* True once the synth task is running (amp/I2S came up). */
bool prop_audio_available(void);

/* Queue an event for playback. Non-blocking; dropped if the queue is full or audio is
 * unavailable. */
void prop_audio_play(prop_audio_event_t event);

/* As prop_audio_play, but transpose the event's tones by `semitones` (noise is
 * unaffected). Used to pitch the screen-change clack up with nesting depth. */
void prop_audio_play_pitched(prop_audio_event_t event, int semitones);

/* PA_PING has several selectable timbres ("voices") instead of one fixed tone —
 * see the s_ping_voices table in prop_audio.c. The active voice is NVS
 * "ping_voice" (u32 index, default 0); SETUP -> AUDIO writes it directly
 * and prop_audio reads it per PA_PING event, so a change takes effect on the
 * very next chirp. These two just describe the table for that UI. */
const char *prop_audio_ping_voice_options(void);  /* "\n"-joined names, LVGL dropdown format */
int prop_audio_ping_voice_count(void);

#endif /* _PROP_AUDIO_H_ */
