/* prop_audio — synth + audio task. See prop_audio.h. */
#include "prop_audio.h"
#include "prop_audio_samples.h"
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
#define PEAK_AMP    9000                         /* headroom below int16 full-scale */
/* Keep the amp powered this long after a sound before dropping it back to muted.
 * Must clear the widest gap between back-to-back events or the amp power-cycles
 * on every single one -- audibly, each cold power-up pops. The SCANNER distance-
 * ping (main/prop_ui.c scan_ping_task) is the long pole: its farthest zone
 * repeats every ~2.8s, so this needs headroom past that to keep the amp warm for
 * the whole time a scan is actively pinging. */
#define AMP_IDLE_MS 3200
/* Settle time after un-muting the amp before the first real samples reach it.
 * The I2S channel free-runs and auto-clears to silence between writes, so the
 * amp's own power-on transient (unmuting a class-D output stage) lands on that
 * silence instead of on the leading edge of the waveform -- without this, the
 * transient and the note's attack overlap and it's audible as a pop/click that
 * eats the first few ms of the tone. */
#define AMP_SETTLE_MS 8

/* ---- Polyphony ------------------------------------------------------------
 * Playback is a tiny software mixer, not a one-shot render-then-blocking-write:
 * each queued event starts its own voice, and every MIX_CHUNK samples the task
 * sums whatever voices are still playing, clips, and writes that one chunk.
 * This is what lets a long tone (the SONAR/TORPEDO pings) ring out completely
 * instead of being cut off by the next event, and lets a second ping legitimately
 * overlap the first's tail instead of queuing behind it.
 *
 * A voice is one of two source kinds:
 *  - VSRC_SYNTH:  rendered once into an owned PSRAM scratch buffer (the note_t
 *    tables below), consumed with a plain integer cursor.
 *  - VSRC_SAMPLE: read directly from an embedded const PCM array (prop_audio_
 *    samples.h) with NO copy, advancing a fractional cursor at `step` samples
 *    per output sample (linear-interpolated resample) so the same distance ->
 *    pitch transpose the synth voices get still applies to real recordings. */
#define MAX_VOICES        3
#define VOICE_MAX_SAMPLES (SR * 2)               /* 2.0 s cap per SYNTH voice's scratch buffer */
#define MIX_CHUNK          512                    /* ~32 ms per mix/write iteration */

typedef enum { VSRC_SYNTH, VSRC_SAMPLE } voice_src_t;

typedef struct {
    voice_src_t kind;
    bool        active;
    /* VSRC_SYNTH */
    int16_t *buf;      /* PSRAM, VOICE_MAX_SAMPLES, owned scratch */
    size_t   len, pos;  /* samples */
    /* VSRC_SAMPLE */
    const int16_t *pcm;
    size_t          pcm_len;
    float           fpos;   /* fractional read position into pcm */
    float           step;   /* pcm samples advanced per output sample (pitch transpose) */
    float           gain;   /* combined level/vol scale, applied on the fly */
} voice_t;

/* Waveform + one note. `level` is the note's weight within its event (0..100);
 * overall loudness is then scaled by the user volume. W_TRIANGLE and W_RINGMOD
 * exist for the PING voices (see s_ping_voices below) — triangle for a colder,
 * more clinical tone than sine without square's buzz; ring-mod (carrier *
 * modulator) for an inharmonic, metallic "computer" timbre. `freq_end` sweeps
 * freq->freq_end linearly across the note (0 = no sweep); `mod_freq` is the
 * ring-mod modulator's frequency (W_RINGMOD only). `decay_pct` overrides how
 * much of the note is the release fade (0 = default 33%; e.g. 80 = the last
 * 80% fades out). */
typedef enum { W_SQUARE, W_SINE, W_NOISE, W_TRIANGLE, W_RINGMOD } wave_t;
typedef struct {
    wave_t   wave;
    uint16_t freq;      /* Hz (ignored for noise); sweep start otherwise */
    uint16_t ms;        /* duration */
    uint8_t  level;     /* 0..100 */
    uint16_t freq_end;  /* 0 = no sweep; else linear sweep target Hz */
    uint16_t mod_freq;  /* ring-mod modulator Hz (W_RINGMOD only) */
    uint8_t  decay_pct; /* 0 = default (33%); else % of note that's the release fade */
} note_t;

/* Event → note sequence (played back-to-back). Square is harsher per-sample than sine,
 * so square notes carry a lower `level`. Tunable. */
static const note_t EV_DIAL_TICK[] = { { .wave = W_NOISE, .ms = 3, .level = 55 } };
/* Screen change: a quick noise tick into a low square "thunk" — a mechanical clack. */
static const note_t EV_SCREEN[]    = { { .wave = W_NOISE, .ms = 7, .level = 45 },
                                       { .wave = W_SQUARE, .freq = 210, .ms = 50, .level = 45 } };
/* Button click: a tiny noise edge + a crisp high square tick. */
static const note_t EV_BUTTON[]    = { { .wave = W_NOISE, .ms = 2, .level = 30 },
                                       { .wave = W_SQUARE, .freq = 1700, .ms = 9, .level = 42 } };
/* Slider step: one short, soft mid tick (kept subtle — it repeats while dragging). */
static const note_t EV_SLIDER[]    = { { .wave = W_SQUARE, .freq = 620, .ms = 7, .level = 28 } };
static const note_t EV_OPEN[]      = { { .wave = W_SQUARE, .freq = 660, .ms = 55, .level = 55 },
                                       { .wave = W_SQUARE, .freq = 990, .ms = 70, .level = 55 } };
static const note_t EV_BACK[]      = { { .wave = W_SQUARE, .freq = 300, .ms = 90, .level = 55 } };
static const note_t EV_TAB[]       = { { .wave = W_SQUARE, .freq = 1200, .ms = 28, .level = 50 } };
static const note_t EV_DENY[]      = { { .wave = W_SQUARE, .freq = 150, .ms = 200, .level = 70 } };
static const note_t EV_SIGNAL[]    = { { .wave = W_SINE, .freq = 700, .ms = 70, .level = 90 },
                                       { .wave = W_SINE, .freq = 900, .ms = 70, .level = 90 },
                                       { .wave = W_SINE, .freq = 1300, .ms = 130, .level = 90 } };
static const note_t EV_ALERT[]     = { { .wave = W_SQUARE, .freq = 520, .ms = 110, .level = 65 },
                                       { .wave = W_SQUARE, .freq = 390, .ms = 110, .level = 65 },
                                       { .wave = W_SQUARE, .freq = 520, .ms = 110, .level = 65 },
                                       { .wave = W_SQUARE, .freq = 390, .ms = 110, .level = 65 } };
/* Boot chime: a soft ascending C-major arpeggio, pure sine, gentle level (not harsh). */
static const note_t EV_BOOT[]      = { { .wave = W_SINE, .freq = 523, .ms = 130, .level = 65 },
                                       { .wave = W_SINE, .freq = 659, .ms = 130, .level = 65 },
                                       { .wave = W_SINE, .freq = 784, .ms = 150, .level = 70 },
                                       { .wave = W_SINE, .freq = 1047, .ms = 240, .level = 65 } };
/* PA_PING is handled separately from the table below — see s_ping_voices. */

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

/* SCANNER distance-ping voices — selectable in SETUP -> AUDIO -> PING TONE
 * (NVS "ping_voice", read per PA_PING event). CLASSIC/METALLIC/BELL/CRISP are
 * synthesized (cold, "cassette-futurism console" tones). SONAR and TORPEDO are
 * real recordings (Hunt for Red October's active sonar / torpedo-acquisition
 * pings, prop_audio_samples.h) played back directly rather than approximated —
 * a synthesized sweep read as a bird chirp, an FFT of the actual clips showed
 * they're each a single very steady tone with a long natural decay, and a
 * plain oscillator can't reproduce that texture convincingly. Exactly one of
 * {notes, pcm} is set per row; pcm_level is the sample's equivalent of a
 * note's `level` (0..100 trim, since the source recording's own loudness
 * doesn't match the synthesized voices' PEAK_AMP headroom). */
static const note_t EV_PING_CLASSIC[]  = { { .wave = W_SINE, .freq = 440, .ms = 28, .level = 60 },
                                           { .wave = W_SINE, .freq = 587, .ms = 42, .level = 70 } };
/* Two crisp triangle ticks, high register, almost no tail — cold and stark
 * rather than warm/round. */
static const note_t EV_PING_METALLIC[] = { { .wave = W_TRIANGLE, .freq = 1046, .ms = 14, .level = 60 },
                                           { .wave = W_TRIANGLE, .freq = 1568, .ms = 20, .level = 65 } };
/* Ring-modulated carrier/modulator pair (inharmonic ~sqrt(2) ratio) — a
 * cold, metallic "computer console" blip; the Alien/Nostromo reference. */
static const note_t EV_PING_BELL[]     = { { .wave = W_RINGMOD, .freq = 660, .ms = 65,
                                             .level = 68, .mod_freq = 934 } };
/* Ultra-short, low-level square tick — the most clinical/minimal option, a
 * bare digital blip with essentially no decay tail. */
static const note_t EV_PING_CRISP[]    = { { .wave = W_SQUARE, .freq = 1800, .ms = 16, .level = 36 } };

#define PING_VOICE_COUNT 6
/* NOT const: SONAR/TORPEDO's pcm/pcm_len are wired up at runtime in
 * prop_audio_init (extern arrays aren't constant expressions, so they can't
 * be initializers here). */
static struct {
    const note_t   *notes;      /* NULL if this voice is a PCM sample instead */
    int             n;
    const char     *name;
    const int16_t  *pcm;         /* NULL if this voice is synthesized instead */
    size_t          pcm_len;
    uint8_t         pcm_level;   /* 0..100, like a note's `level` */
} s_ping_voices[PING_VOICE_COUNT] = {
    { EV_PING_CLASSIC,  sizeof(EV_PING_CLASSIC)  / sizeof(EV_PING_CLASSIC[0]),  "CLASSIC",  NULL, 0, 0 },
    { EV_PING_METALLIC, sizeof(EV_PING_METALLIC) / sizeof(EV_PING_METALLIC[0]), "METALLIC", NULL, 0, 0 },
    { NULL, 0, "SONAR",   NULL, 0, 32 },   /* pcm/pcm_len filled in at init -- see prop_audio_init */
    { EV_PING_BELL,     sizeof(EV_PING_BELL)     / sizeof(EV_PING_BELL[0]),     "BELL",     NULL, 0, 0 },
    { EV_PING_CRISP,    sizeof(EV_PING_CRISP)    / sizeof(EV_PING_CRISP[0]),    "CRISP",    NULL, 0, 0 },
    { NULL, 0, "TORPEDO", NULL, 0, 32 },
};
#define PING_VOICE_SONAR   2
#define PING_VOICE_TORPEDO 5

/* Queue message: which event + a semitone transpose for its tones. */
typedef struct {
    uint8_t event;
    int8_t  semitones;
} pa_msg_t;

static QueueHandle_t s_queue;
static voice_t       s_voices[MAX_VOICES];
static int32_t       s_mix[MIX_CHUNK];   /* mix accumulator, wide enough to not clip mid-sum */
static int16_t       s_out[MIX_CHUNK];   /* clipped-to-int16 chunk handed to bsp_audio_write */
static bool           s_available;

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
    size_t release = nt->decay_pct ? (size_t)((uint32_t)n * nt->decay_pct / 100) : n / 3;
    if (release > n) release = n;
    size_t rel_start = (n > release) ? n - release : 0;

    float amp = (float)PEAK_AMP * (nt->level / 100.0f) * (vol / 100.0f);
    float phase = 0.0f, mphase = 0.0f;
    float dphase0 = (float)nt->freq * pitch_mul / SR;      /* cycles per sample, sweep start */
    float dphase1 = nt->freq_end ? (float)nt->freq_end * pitch_mul / SR : dphase0;
    float dmphase = (float)nt->mod_freq * pitch_mul / SR;  /* ring-mod modulator, if any */
    uint32_t lcg = 0x1234567u + nt->freq;          /* deterministic noise seed */

    for (size_t i = 0; i < n; i++) {
        /* Linear sweep dphase0->dphase1 across the note (no-op when freq_end==0). */
        float sweep_t = (n > 1) ? (float)i / (float)(n - 1) : 0.0f;
        float dphase = dphase0 + (dphase1 - dphase0) * sweep_t;

        float s;
        switch (nt->wave) {
            case W_SINE:     s = sinf(2.0f * (float)M_PI * phase); break;
            case W_TRIANGLE: {
                float x = phase - floorf(phase);
                s = 4.0f * fabsf(x - floorf(x + 0.5f)) - 1.0f;
                break;
            }
            case W_RINGMOD:  s = sinf(2.0f * (float)M_PI * phase) * sinf(2.0f * (float)M_PI * mphase);
                             break;
            case W_NOISE:    lcg = lcg * 1664525u + 1013904223u;
                             s = (float)((int32_t)(lcg >> 9) - (1 << 22)) / (float)(1 << 22);
                             break;
            case W_SQUARE:
            default:         s = (phase - floorf(phase)) < 0.5f ? 1.0f : -1.0f; break;
        }
        phase += dphase;
        mphase += dmphase;

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

static size_t render_notes(const note_t *notes, int n, uint32_t vol, int semitones,
                           int16_t *dst, size_t cap)
{
    float pitch_mul = (semitones == 0) ? 1.0f : powf(2.0f, semitones / 12.0f);
    size_t total = 0;
    for (int k = 0; k < n && total < cap; k++) {
        total += render_note(&notes[k], vol, pitch_mul, dst + total, cap - total);
    }
    return total;
}

/* Render `ev` into a free voice slot (or, if every slot is busy — rare, needs
 * MAX_VOICES+1 events truly overlapping — steal whichever voice has the least
 * left to play) so it starts mixing in on the very next chunk. PA_PING picks
 * between a synthesized note table and a PCM sample per s_ping_voices; every
 * other event is always synthesized. */
static void start_voice(prop_audio_event_t ev, uint32_t vol, int semitones)
{
    int slot = -1;
    for (int i = 0; i < MAX_VOICES; i++) {
        if (!s_voices[i].active) { slot = i; break; }
    }
    if (slot < 0) {
        size_t best_remain = 0;
        for (int i = 0; i < MAX_VOICES; i++) {
            voice_t *v = &s_voices[i];
            size_t remain = (v->kind == VSRC_SYNTH) ? (v->len - v->pos)
                                                     : (size_t)((float)v->pcm_len - v->fpos);
            if (i == 0 || remain < best_remain) { best_remain = remain; slot = i; }
        }
    }
    voice_t *v = &s_voices[slot];

    if (ev == PA_PING) {
        uint32_t idx = 0;
        prop_settings_get_u32("ping_voice", &idx, 0);
        if (idx >= (uint32_t)PING_VOICE_COUNT) idx = 0;
        if (s_ping_voices[idx].pcm && s_ping_voices[idx].pcm_len > 1) {
            v->kind = VSRC_SAMPLE;
            v->pcm = s_ping_voices[idx].pcm;
            v->pcm_len = s_ping_voices[idx].pcm_len;
            v->fpos = 0.0f;
            v->step = (semitones == 0) ? 1.0f : powf(2.0f, semitones / 12.0f);
            v->gain = (s_ping_voices[idx].pcm_level / 100.0f) * (vol / 100.0f);
            v->active = true;
            return;
        }
        v->kind = VSRC_SYNTH;
        size_t n = render_notes(s_ping_voices[idx].notes, s_ping_voices[idx].n, vol, semitones,
                                v->buf, VOICE_MAX_SAMPLES);
        if (n > 0) { v->len = n; v->pos = 0; v->active = true; }
        return;
    }

    if (ev < 0 || ev >= PA_EVENT_COUNT || !s_events[ev].notes) {
        return;
    }
    v->kind = VSRC_SYNTH;
    size_t n = render_notes(s_events[ev].notes, s_events[ev].n, vol, semitones, v->buf, VOICE_MAX_SAMPLES);
    if (n > 0) { v->len = n; v->pos = 0; v->active = true; }
}

/* Pop one queued event (if any) and start it, honoring mute/volume. Shared by
 * the idle (blocking) and active (draining) paths in audio_task below. */
static bool pump_queue(TickType_t wait)
{
    pa_msg_t msg;
    if (xQueueReceive(s_queue, &msg, wait) != pdTRUE) {
        return false;
    }
    uint32_t mute = 0, vol = 60;
    prop_settings_get_u32("audio_mute", &mute, 0);
    prop_settings_get_u32("audio_vol", &vol, 60);
    if (!mute && vol != 0) {
        start_voice((prop_audio_event_t)msg.event, vol, msg.semitones);
    }
    return true;
}

static void audio_task(void *arg)
{
    (void)arg;
    bool amp_on = false;
    while (1) {
        /* Drain everything queued without blocking -- rendering is cheap (a few
         * hundred us for even the longest note table), so doing it inline with
         * mixing keeps this all on one task/one set of buffers, no extra locks. */
        while (pump_queue(0)) { }

        bool any_active = false;
        for (int i = 0; i < MAX_VOICES; i++) {
            if (s_voices[i].active) { any_active = true; break; }
        }

        if (!any_active) {
            /* Grace period before cutting the amp, so a burst of quick, closely
             * spaced UI sounds doesn't power-cycle (and pop) it on every single
             * one -- see AMP_IDLE_MS. Block on the next event; only actually
             * power down if this wait times out with nothing arriving. */
            if (pump_queue(pdMS_TO_TICKS(AMP_IDLE_MS))) {
                continue;               /* got one -- go mix it, don't touch the amp */
            }
            if (amp_on) {
                bsp_audio_amp(false);
                amp_on = false;
            }
            continue;
        }

        if (!amp_on) {
            bsp_audio_amp(true);
            vTaskDelay(pdMS_TO_TICKS(AMP_SETTLE_MS));   /* let the power-on pop land on silence */
            amp_on = true;
        }

        /* Mix one chunk from every active voice, clip to int16, write. Voices
         * that end mid-chunk just stop contributing (their own release fade /
         * source data already tapers toward 0, so there's no click at the seam). */
        memset(s_mix, 0, sizeof(s_mix));
        for (int i = 0; i < MAX_VOICES; i++) {
            voice_t *v = &s_voices[i];
            if (!v->active) continue;

            if (v->kind == VSRC_SYNTH) {
                size_t remain = v->len - v->pos;
                size_t take = remain < MIX_CHUNK ? remain : MIX_CHUNK;
                for (size_t s = 0; s < take; s++) {
                    s_mix[s] += v->buf[v->pos + s];
                }
                v->pos += take;
                if (v->pos >= v->len) v->active = false;
            } else {
                for (size_t s = 0; s < MIX_CHUNK; s++) {
                    if (v->fpos >= (float)(v->pcm_len - 1)) { v->active = false; break; }
                    size_t i0 = (size_t)v->fpos;
                    float frac = v->fpos - (float)i0;
                    float s0 = (float)v->pcm[i0];
                    float s1 = (float)v->pcm[i0 + 1];
                    float sample = s0 + (s1 - s0) * frac;
                    s_mix[s] += (int32_t)(sample * v->gain);
                    v->fpos += v->step;
                }
            }
        }
        for (size_t s = 0; s < MIX_CHUNK; s++) {
            int32_t x = s_mix[s];
            if (x > 32767) x = 32767;
            else if (x < -32768) x = -32768;
            s_out[s] = (int16_t)x;
        }
        bsp_audio_write(s_out, MIX_CHUNK);
    }
}

esp_err_t prop_audio_init(void)
{
    esp_err_t err = bsp_audio_init();
    if (err != ESP_OK) {
        return err;                                /* caller logs; prop runs silent */
    }
    for (int i = 0; i < MAX_VOICES; i++) {
        s_voices[i].buf = heap_caps_malloc(sizeof(int16_t) * VOICE_MAX_SAMPLES, MALLOC_CAP_SPIRAM);
        if (!s_voices[i].buf) {
            ESP_LOGE(AUDIO_TAG, "no PSRAM for voice %d render buffer", i);
            return ESP_ERR_NO_MEM;
        }
    }
    /* s_ping_voices is otherwise a compile-time const table; these two rows
     * point at the embedded PCM (prop_audio_samples.h) instead of a note_t
     * table, and extern arrays aren't constant expressions, so they're wired
     * up here rather than in the initializer above. */
    s_ping_voices[PING_VOICE_SONAR].pcm       = prop_audio_sonar_pcm;
    s_ping_voices[PING_VOICE_SONAR].pcm_len   = prop_audio_sonar_pcm_len;
    s_ping_voices[PING_VOICE_TORPEDO].pcm     = prop_audio_torpedo_pcm;
    s_ping_voices[PING_VOICE_TORPEDO].pcm_len = prop_audio_torpedo_pcm_len;

    s_queue = xQueueCreate(8, sizeof(pa_msg_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(audio_task, "prop_audio", 4096, NULL, 5, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_available = true;
    ESP_LOGI(AUDIO_TAG, "synth up (%d events, %d voices)", PA_EVENT_COUNT, MAX_VOICES);
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

const char *prop_audio_ping_voice_options(void)
{
    /* "\n"-joined names in table order (LVGL dropdown format), built once into
     * static storage the first time it's asked for — the caller doesn't own it. */
    static char opts[128];
    if (opts[0] == '\0') {
        size_t off = 0;
        for (int i = 0; i < PING_VOICE_COUNT && off < sizeof(opts); i++) {
            int w = snprintf(opts + off, sizeof(opts) - off, "%s%s",
                             i ? "\n" : "", s_ping_voices[i].name);
            if (w > 0) off += (size_t)w;
        }
    }
    return opts;
}

int prop_audio_ping_voice_count(void) { return PING_VOICE_COUNT; }
