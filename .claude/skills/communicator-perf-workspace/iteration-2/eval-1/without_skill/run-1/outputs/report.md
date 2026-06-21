# Spectrum Screen Performance Analysis

## Root-cause summary

The spectrum analyzer screen runs the observer (`ui_observer`) at 20 Hz (50 ms per engine tick, `ANIM_PERIOD_MS=50`). Every observer call while `PK_SPECTRUM` is active hits a tight loop over all 24 frequency bands (PROP_MIC_BANDS = 24), and for each band calls three LVGL style-mutation APIs:

```c
// prop_ui.c ~line 2722-2728
lv_obj_set_height(s_spec_bars[i], h);
lv_obj_align(s_spec_bars[i], LV_ALIGN_BOTTOM_LEFT,
             SPEC_X0 + i * (SPEC_BW + SPEC_GAP), -SPEC_BASE);
lv_obj_set_style_bg_color(s_spec_bars[i], ..., 0);
```

That is **3 x 24 = 72 LVGL invalidations per observer call**, every single frame, whether or not any bar value actually changed. Each invalidation marks the bar's bounding rectangle as dirty; LVGL then recomposites all 24 dirty rectangles during the next render. On PSRAM (custom allocator, `CONFIG_LV_USE_CUSTOM_MALLOC=y`), every LVGL heap touch crosses the PSRAM bus roughly 3-5x slower than internal SRAM. The net effect is a render budget entirely consumed by these 72 redundant style writes, leaving little headroom for the display pipeline.

Contrast this with the home/scanner screen, which is heavily optimised: the waveform trace uses a shadow buffer (`s_wave_shadow`) to detect which of 10 wave segments actually changed, only calls `lv_line_set_points()` on those, and guards every other widget update behind a `!= last_value` check. Steady state invalidates only 1 segment (4 px wide) and the blip (4 px), so that screen renders cheaply.

---

## Findings, impact-ranked

### Finding 1 (CRITICAL) -- Unconditional 72-invalidation storm per frame

**File:** `firmware/communicator/main/prop_ui.c`, lines ~2714-2728 (inside `ui_observer`)

**What happens:** The spectrum update block runs unconditionally on every observer tick while `PK_SPECTRUM` is active. For all 24 bars it calls `lv_obj_set_height`, `lv_obj_align`, and `lv_obj_set_style_bg_color` with no change-detection. Even when audio is silent and all bar values are identical to the previous frame, all 72 mutations fire, causing LVGL to mark 24 dirty regions and repaint them all.

**Fix:** Add a shadow array (parallel to `s_spec_decay`) tracking last-rendered height and color. Only call the three LVGL APIs when the value has actually changed. In a quiet room this reduces invalidations from 72/frame to ~0/frame; during active audio, only bars that moved get re-rendered.

```c
// Add near s_spec_decay declaration:
static int    s_spec_last_h[PROP_MIC_BANDS];
static uint8_t s_spec_last_col[PROP_MIC_BANDS];  // 0=mute, 1=amber, 2=alert

// In the observer, replace the unconditional writes:
for (int i = 0; i < PROP_MIC_BANDS; i++) {
    float v = (float)bands[i];
    if (v >= s_spec_decay[i]) s_spec_decay[i] = v;
    else s_spec_decay[i] *= 0.80f;
    int pct = (int)s_spec_decay[i];
    int h = 2 + pct * SPEC_MAXH / 100;
    uint8_t col_id = pct > 85 ? 2 : (pct > 35 ? 1 : 0);

    if (h != s_spec_last_h[i] || col_id != s_spec_last_col[i]) {
        lv_obj_set_height(s_spec_bars[i], h);
        lv_obj_align(s_spec_bars[i], LV_ALIGN_BOTTOM_LEFT,
                     SPEC_X0 + i * (SPEC_BW + SPEC_GAP), -SPEC_BASE);
        lv_color_t c = col_id == 2 ? COL_ALERT : (col_id == 1 ? COL_AMBER : COL_MUTE);
        lv_obj_set_style_bg_color(s_spec_bars[i], c, 0);
        s_spec_last_h[i] = h;
        s_spec_last_col[i] = col_id;
    }
}
```

**Expected impact:** Drops per-frame LVGL work from 72 mutations to ~3-8 during active audio, ~0 in silence. Frame time should match the home screen.

---

### Finding 2 (HIGH) -- `lv_obj_align` called every frame for static X geometry

**File:** `prop_ui.c`, lines ~2724-2725 (spectrum), also ~2748 (RF band) and ~2790 (CSI)

**What happens:** `lv_obj_align` is not a no-op when the position does not change. In LVGL 9, it recomputes the object's coordinates relative to its parent and calls `lv_obj_invalidate` unconditionally. The X position of every spectrum bar is deterministic (`SPEC_X0 + i * (SPEC_BW + SPEC_GAP)`) and never changes at runtime -- only the height changes. Calling `lv_obj_align` every frame for the X position is wasteful.

**Fix:** Set the X position once at build time in `build_spectrum_panel` using `lv_obj_set_pos` or set the bar's bottom alignment there. In the observer, use `lv_obj_set_height` only (height grows upward if the bar is bottom-anchored). This removes 24 `lv_obj_align` calls per frame regardless of change-detection (those calls are still expensive even when gated, since the coord comparison has overhead). This fix is synergistic with Finding 1: apply both.

**Expected impact:** Eliminates 24 `lv_obj_align` calls per frame unconditionally.

---

### Finding 3 (MEDIUM) -- dB label updated every 4 ticks with no dedup

**File:** `prop_ui.c`, lines ~2729-2732

```c
if (st->tick % 4 == 0 && s_spec_db) {
    lv_label_set_text_fmt(s_spec_db, "%d dB", prop_mic_get_db());
    set_meter(s_spec_db_bar, (prop_mic_get_db() + 60) * 100 / 60, COL_AMBER);
}
```

**What happens:** `lv_label_set_text_fmt` in LVGL 9 always reallocates and invalidates the label, even if the text content is unchanged. The `label_set_text_cached` helper already exists in this file (used on the scanner screen for exactly this purpose) but is not used here.

**Fix:** Cache the last dB value and bar percentage; only call these when they differ. Also call `prop_mic_get_db()` once, not twice.

**Expected impact:** Minor individually, but consistent with the project's existing pattern and eliminates a forced label re-rasterization 5x/second.

---

### Finding 4 (MEDIUM) -- FFT trig recomputed per butterfly stage, not pre-tabled

**File:** `firmware/communicator/main/prop_mic.c`, lines 44-47 (inside `fft()`)

```c
float ang = -2.0f * (float)M_PI / len;
float wr = cosf(ang), wi = sinf(ang);
```

This runs 8 butterfly stages x multiple inner loops, calling `cosf`/`sinf` at every stage. The ESP32-P4 has hardware floating-point but no hardware trig; these are software implementations. A pre-computed twiddle-factor table (computed once in `prop_mic_init`) would eliminate ~60-80 trig calls per FFT block. This is the standard embedded FFT optimisation.

**Expected impact:** Reduces `mic_task` CPU time (priority 5). Since mic_task and the LVGL render task share a core, reducing mic_task's slice directly increases render headroom.

---

### Finding 5 (LOW) -- `compute_bands` calls `powf` 48x per FFT block

**File:** `prop_mic.c`, lines 69-72

```c
float f0 = powf((float)(N_BINS - 1), (float)b / PROP_MIC_BANDS);
float f1 = powf((float)(N_BINS - 1), (float)(b + 1) / PROP_MIC_BANDS);
```

These band boundary calculations are constant (they don't depend on audio data) and can be pre-computed once in `prop_mic_init` into `static int band_lo[PROP_MIC_BANDS], band_hi[PROP_MIC_BANDS]` tables, removing 48 `powf` calls per ~16 ms audio block.

**Expected impact:** Minor on the P4. The fix is trivial.

---

### Finding 6 (LOW) -- RF-band and CSI screens have the same unconditional align+color pattern

**File:** `prop_ui.c`, lines ~2738-2751 (RF band), ~2779-2793 (CSI)

These share identical geometry: static X positions re-passed to `lv_obj_align` every frame. Lower priority than spectrum (RF band only refreshes on scan; CSI data changes slowly), but the same shadow-array + static-geometry fix applies when those screens are polished.

---

## Measurement plan (for when a device is available)

1. **Enable FPS HUD** (SETUP > DISPLAY > FPS METER). Navigate to SPECTRUM and note baseline FPS. Navigate back to CONSOLE and note home-screen FPS for comparison.

2. **Instrument the observer** with `esp_timer_get_time()` wrapping only the spectrum block. Log "SPEC_OBS %lld us" at 1 Hz (not per-frame -- logging itself takes time). Compare before/after Finding 1.

3. **Count dirty invalidations**: add a temporary counter that increments per `lv_obj_invalidate` call via a thin wrapper. Count per second; expect to drop from ~1440/s (72 x 20 Hz) to ~60-160/s after the fix.

4. **Check core affinity**: confirm whether the LVGL task and the mic task share a core. If they do, the mic FFT trig optimisation (Finding 4) will have direct impact on render jitter.

5. **A/B test sequence**: (a) baseline, (b) apply Finding 1 only (shadow), (c) apply Finding 1+2 (shadow + static align), (d) apply Finding 4 (twiddle table). Step (c) is the expected turning point.

---

## Conclusion

The spectrum screen stutter is caused by a missing change-detection layer: all 24 bars fire `lv_obj_set_height + lv_obj_align + lv_obj_set_style_bg_color` unconditionally on every 20 Hz observer tick regardless of whether bar values changed. This generates 72 LVGL invalidations per frame -- the same class of bug that was previously fixed on the scanner screen (documented in the `s_wave_shadow` array and the `s_last_blip_x/col` trackers in the same file). Applying the same shadow pattern to the spectrum bars (Finding 1) and eliminating the redundant static-geometry `lv_obj_align` calls (Finding 2) are the primary fixes. All other findings are secondary.
