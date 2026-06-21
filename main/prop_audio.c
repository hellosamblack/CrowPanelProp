/* prop_audio — synth + audio task. See prop_audio.h. */
#include "prop_audio.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "bsp_audio.h"
#include "prop_settings.h"

#define AUDIO_TAG "PROP_AUDIO"

#define SR          BSP_AUDIO_SAMPLE_HZ          /* 16 kHz */
#define MAX_SAMPLES (SR * 4 / 5)                 /* 0.8 s — longest event (boot chime) */
#define PEAK_AMP    9000                         /* headroom below int16 full-scale */
#define AMP_IDLE_MS 150                          /* keep amp powered this long after a sound */

/* Waveform + one note. `level` is the note's weight within its event (0..100);
 * overall loudness is then scaled by the user volume. */
typedef enum { W_SQUARE, W_SINE, W_NOISE } wave_t;
typedef struct {
    wave_t   wave;
    uint16_t freq;   /* Hz (ignored for noise) */
    uint16_t ms;     /* duration */
    uint8_t  level;  /* 0..100 */
} note_t;

/* Event → note sequence (played back-to-back). Square is harsher per-sample than sine,
 * so square notes carry a lower `level`. Tunable. */
static const note_t EV_DIAL_TICK[] = { { W_NOISE, 0, 3, 55 } };
/* Screen change: a quick noise tick into a low square "thunk" — a mechanical clack. */
static const note_t EV_SCREEN[]    = { { W_NOISE, 0, 7, 45 }, { W_SQUARE, 210, 50, 45 } };
/* Button click: a tiny noise edge + a crisp high square tick. */
static const note_t EV_BUTTON[]    = { { W_NOISE, 0, 2, 30 }, { W_SQUARE, 1700, 9, 42 } };
/* Slider step: one short, soft mid tick (kept subtle — it repeats while dragging). */
static const note_t EV_SLIDER[]    = { { W_SQUARE, 620, 7, 28 } };
static const note_t EV_OPEN[]      = { { W_SQUARE, 660, 55, 55 }, { W_SQUARE, 990, 70, 55 } };
static const note_t EV_BACK[]      = { { W_SQUARE, 300, 90, 55 } };
static const note_t EV_TAB[]       = { { W_SQUARE, 1200, 28, 50 } };
static const note_t EV_DENY[]      = { { W_SQUARE, 150, 200, 70 } };
static const note_t EV_SIGNAL[]    = { { W_SINE, 700, 70, 90 }, { W_SINE, 900, 70, 90 },
                                       { W_SINE, 1300, 130, 90 } };
static const note_t EV_ALERT[]     = { { W_SQUARE, 520, 110, 65 }, { W_SQUARE, 390, 110, 65 },
                                       { W_SQUARE, 520, 110, 65 }, { W_SQUARE, 390, 110, 65 } };
/* Boot chime: a soft ascending C-major arpeggio, pure sine, gentle level (not harsh). */
static const note_t EV_BOOT[]      = { { W_SINE, 523, 130, 65 }, { W_SINE, 659, 130, 65 },
                                       { W_SINE, 784, 150, 70 }, { W_SINE, 1047, 240, 65 } };

#define EV(arr) { (arr), sizeof(arr) / sizeof((arr)[0]) }
static const struct { const note_t *notes; int n; } s_events[PA_EVENT_COUNT] = {
    [PA_DIAL_TICK] = EV(EV_DIAL_TICK),
    [PA_SCREEN]    = EV(EV_SCREEN),
    [PA_BUTTON]    = EV(EV_BUTTON),
    [PA_SLIDER]    = EV(EV_SLIDER),
    [PA_OPEN]      = EV(EV_OPEN),
    [PA_BACK]      = EV(EV_BACK),
    [PA_TAB]       = EV(EV_TAB),
    [PA_DENY]      = EV(EV_DENY),
    [PA_SIGNAL]    = EV(EV_SIGNAL),
    [PA_ALERT]     = EV(EV_ALERT),
    [PA_BOOT]      = EV(EV_BOOT),
};

/* Queue message: which event + a semitone transpose for its tones. */
typedef struct {
    uint8_t event;
    int8_t  semitones;
} pa_msg_t;

static QueueHandle_t s_queue;
static int16_t      *s_buf;          /* render scratch (PSRAM) */
static bool          s_available;

/* Render one note into buf[0..]. Returns the sample count written (0 if it won't fit).
 * Envelope: short linear attack + linear release so tones don't click. `vol` is 0..100. */
static size_t render_note(const note_t *nt, uint32_t vol, float pitch_mul,
                          int16_t *buf, size_t cap)
{
    size_t n = (size_t)nt->ms * SR / 1000;
    if (n == 0 || n > cap) {
        n = (n > cap) ? cap : n;
    }
    if (n == 0) {
        return 0;
    }
    size_t attack = n / 8 + 1;
    if (attack > 64) attack = 64;                 /* ~4 ms */
    size_t release = n / 3;                        /* fade the tail */
    size_t rel_start = (n > release) ? n - release : 0;

    float amp = (float)PEAK_AMP * (nt->level / 100.0f) * (vol / 100.0f);
    float phase = 0.0f;
    float dphase = (float)nt->freq * pitch_mul / SR;   /* cycles per sample */
    uint32_t lcg = 0x1234567u + nt->freq;          /* deterministic noise seed */

    for (size_t i = 0; i < n; i++) {
        float s;
        switch (nt->wave) {
            case W_SINE:   s = sinf(2.0f * (float)M_PI * phase); break;
            case W_NOISE:  lcg = lcg * 1664525u + 1013904223u;
                           s = (float)((int32_t)(lcg >> 9) - (1 << 22)) / (float)(1 << 22);
                           break;
            case W_SQUARE:
            default:       s = (phase - floorf(phase)) < 0.5f ? 1.0f : -1.0f; break;
        }
        phase += dphase;

        float env = 1.0f;
        if (i < attack) {
            env = (float)i / attack;
        } else if (i >= rel_start && release) {
            env = (float)(n - i) / release;
        }
        buf[i] = (int16_t)(s * env * amp);
    }
    return n;
}

static size_t render_event(prop_audio_event_t ev, uint32_t vol, int semitones)
{
    if (ev < 0 || ev >= PA_EVENT_COUNT || !s_events[ev].notes) {
        return 0;
    }
    float pitch_mul = (semitones == 0) ? 1.0f : powf(2.0f, semitones / 12.0f);
    size_t total = 0;
    for (int k = 0; k < s_events[ev].n && total < MAX_SAMPLES; k++) {
        total += render_note(&s_events[ev].notes[k], vol, pitch_mul,
                             s_buf + total, MAX_SAMPLES - total);
    }
    return total;
}

static void audio_task(void *arg)
{
    (void)arg;
    bool amp_on = false;
    while (1) {
        pa_msg_t msg;
        TickType_t wait = amp_on ? pdMS_TO_TICKS(AMP_IDLE_MS) : portMAX_DELAY;
        if (xQueueReceive(s_queue, &msg, wait) != pdTRUE) {
            if (amp_on) {                          /* idle grace expired — power the amp down */
                bsp_audio_amp(false);
                amp_on = false;
            }
            continue;
        }

        uint32_t mute = 0, vol = 60;
        prop_settings_get_u32("audio_mute", &mute, 0);
        prop_settings_get_u32("audio_vol", &vol, 60);
        if (mute || vol == 0) {
            continue;                              /* gated: drop the event (amp times out) */
        }

        size_t n = render_event((prop_audio_event_t)msg.event, vol, msg.semitones);
        if (n == 0) {
            continue;
        }
        if (!amp_on) {
            bsp_audio_amp(true);
            amp_on = true;
        }
        bsp_audio_write(s_buf, n);
    }
}

esp_err_t prop_audio_init(void)
{
    esp_err_t err = bsp_audio_init();
    if (err != ESP_OK) {
        return err;                                /* caller logs; prop runs silent */
    }
    s_buf = heap_caps_malloc(sizeof(int16_t) * MAX_SAMPLES, MALLOC_CAP_SPIRAM);
    if (!s_buf) {
        ESP_LOGE(AUDIO_TAG, "no PSRAM for render buffer");
        return ESP_ERR_NO_MEM;
    }
    s_queue = xQueueCreate(8, sizeof(pa_msg_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(audio_task, "prop_audio", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_available = true;
    ESP_LOGI(AUDIO_TAG, "synth up (%d events)", PA_EVENT_COUNT);
    return ESP_OK;
}

bool prop_audio_available(void) { return s_available; }

void prop_audio_play_pitched(prop_audio_event_t event, int semitones)
{
    if (!s_available || event < 0 || event >= PA_EVENT_COUNT) {
        return;
    }
    if (semitones < -60) semitones = -60;
    if (semitones > 60)  semitones = 60;
    pa_msg_t msg = { .event = (uint8_t)event, .semitones = (int8_t)semitones };
    xQueueSend(s_queue, &msg, 0);                  /* non-blocking; drop if full */
}

void prop_audio_play(prop_audio_event_t event)
{
    prop_audio_play_pitched(event, 0);
}
