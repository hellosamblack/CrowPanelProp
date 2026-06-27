# MOTION SCAN Autorange Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the MOTION SCAN radar autorange in smooth discrete chunks, reposition the fan to clear the bottom status overlay, and replace the 5-arc pulse with a single Halo-style gradient pulse.

**Architecture:** All changes are in `main/prop_ui.c` — `build_motion_panel()` (widget construction), the `PK_MOTION` block of `ui_observer` (per-tick rendering), and `close_panel()` (lifecycle NULLing). A single animated `ppm` (pixels-per-metre) becomes the sole scale source: arcs, range labels, and blips all derive from it. A small autorange state machine picks a scale from a discrete ladder with hysteresis and eases `ppm` between levels.

**Tech Stack:** ESP-IDF 6.0.1, LVGL 9.4 (`lv_*` v9 API), C. Build target `esp32p4`.

## Global Constraints

- **Single file:** all edits in `main/prop_ui.c`. No new `.c` files → no `idf.py reconfigure` needed.
- **LVGL 9.4 rules** (from CLAUDE.md): no `%f` in `lv_label_set_text_fmt` — use `snprintf`+`label_set_text_cached`/`lv_label_set_text`. Use `lv_point_precise_t` for line points. Panel takes native RGB565.
- **Panel lifecycle:** every new `s_motion_*` widget pointer MUST be NULLed in `close_panel()`, and all observer use guarded by `s_cur_kind == PK_MOTION && <ptr>` (CLAUDE.md "Add a screen" rules).
- **No automated tests** in this repo. Each task is verified by build → flash → live `/screenshot`. The dial/`/cmd input` simulator drives navigation; LD2450 targets come from real hardware (wave a hand in front of the sensor) — note this where target-dependent.
- **Commits deferred.** `main/prop_ui.c` carries ~165 lines of pre-existing, uncommitted motion work (the 5-arc pulse + three-column T1/T2/T3 layout) that this plan builds on. `git add main/prop_ui.c` cannot separate our changes from that baseline, so the per-task commit steps are **held** — make edits + build-verify only, and commit at end-of-run after user review. Subagents must NOT run `git add`/`git commit`.
- **Style kit:** `COL_AMBER` 0xE0B000, `COL_MUTE` 0xB58A00, `COL_DIM` 0x6B5300, `COL_ALERT` 0xFF3030. Fonts `FONT_BODY`/`FONT_HEAD`. Camera-legible: never `COL_DIM` for must-read text.

### Build / flash / screenshot commands (used in every task)

```powershell
# Build (PowerShell):
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" build
# Confirm port first (varies COM7/COM4): [System.IO.Ports.SerialPort]::GetPortNames()
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash
```

```bash
# Screenshot of the MOTION SCAN panel (mDNS, no IP hunting):
python tools/prop.py shot motion.png --screen motion --wait
# Tight crop of the radar box (left 440x440 from rbox origin 10,64):
python tools/prop.py shot radar.png --screen motion --crop 10,64,440,440 --zoom 2 --wait
```

---

## Task 1: Reposition the fan apex

**Files:**
- Modify: `main/prop_ui.c` — the `FAN_APEX_Y` define (~line 222).

**Interfaces:**
- Consumes: nothing.
- Produces: nothing new. `FAN_APEX_Y` is already consumed by all fan geometry (arcs, edges, ticks, blips, direction ring) so they follow automatically.

**Why:** Apex sits at `y=410` of a 440 px box (~30 px below it), so the bottom-anchored sensor bars / flag words / direction ring crowd the fan's lower flanks. Raising the apex opens a clear band below it. `apex_y − FAN_R = 330 − 240 = 90 > 0`, so no clipping at the box top; `440 − 330 = 110 px` of clearance below.

- [ ] **Step 1: Raise the apex**

Current (`main/prop_ui.c` ~222):
```c
#define FAN_APEX_Y   410      /* apex near bottom; fan sweeps upward */
```
Change to:
```c
#define FAN_APEX_Y   330      /* apex raised to clear the bottom status overlay; fan sweeps upward */
```

- [ ] **Step 2: Build**

Run the build command. Expected: `Project build complete`.

- [ ] **Step 3: Flash + screenshot**

Flash, then `python tools/prop.py shot radar.png --screen motion --crop 10,64,440,440 --zoom 2 --wait`.
Expected: the fan apex sits higher; the sensor bars (bottom-left), flag words (bottom-right), and the N/E/S/W direction ring no longer overlap the fan arcs/edges. If overlap remains, decrement `FAN_APEX_Y` in ~20 px steps (down to ~300) and/or reduce `FAN_R` (240 → 220) and re-shoot until the bottom band is clear.

- [ ] **Step 4: Commit**

```bash
git add main/prop_ui.c
git commit -m "feat(motion): raise radar fan apex to clear bottom status overlay"
```

---

## Task 2: Single Halo-style gradient pulse

**Files:**
- Modify: `main/prop_ui.c` — pulse define + array (~201), build loop (~3355), observer pulse block (~4489), `close_panel` loop (~410).

**Interfaces:**
- Consumes: `FAN_APEX_X/Y`, `FAN_R`, `FAN_LEFT_DEG`, `FAN_RIGHT_DEG`, `s_pulse_phase`.
- Produces: `s_motion_pulse[PULSE_BAND]` (still the guard pointer at observer entry: `s_motion_pulse[0]`).

**Why:** Replace 5 spaced thin arcs (separate dots) with one ~36 px-thick band whose leading (outer) edge is opaque and trailing (inner) edge fades to 0 — a contiguous stack of thin sub-arcs, since LVGL arcs are single-opacity across their width.

- [ ] **Step 1: Change the pulse define + array**

Current (~201):
```c
#define PULSE_STEPS 5
static lv_obj_t *s_motion_pulse[PULSE_STEPS]; /* radial pulse arcs (comet tail) */
```
Change to:
```c
#define PULSE_BAND  9      /* contiguous sub-arcs forming one thick gradient band */
#define PULSE_WIDTH 4      /* px per sub-arc -> ~36 px band thickness */
static lv_obj_t *s_motion_pulse[PULSE_BAND]; /* one Halo-style pulse: bright leading edge -> 0 trailing */
```

- [ ] **Step 2: Update the build loop**

Current (~3357):
```c
    for (int i = 0; i < PULSE_STEPS; i++) {
        s_motion_pulse[i] = lv_arc_create(rbox);
        lv_obj_remove_style(s_motion_pulse[i], NULL, LV_PART_KNOB);
        lv_obj_clear_flag(s_motion_pulse[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(s_motion_pulse[i], 2, 2);
        lv_obj_set_pos(s_motion_pulse[i], FAN_APEX_X - 1, FAN_APEX_Y - 1);
        lv_arc_set_rotation(s_motion_pulse[i], 0);
        lv_arc_set_bg_angles(s_motion_pulse[i], FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_arc_set_angles(s_motion_pulse[i], FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_obj_set_style_arc_width(s_motion_pulse[i], 3, LV_PART_MAIN);
        lv_obj_set_style_arc_color(s_motion_pulse[i], COL_AMBER, LV_PART_MAIN);
        lv_obj_set_style_arc_width(s_motion_pulse[i], 0, LV_PART_INDICATOR);
        lv_obj_set_style_opa(s_motion_pulse[i], LV_OPA_TRANSP, 0);
    }
    s_pulse_phase = 0;
```
Change the two loop bounds and the arc width (3 → `PULSE_WIDTH`):
```c
    for (int i = 0; i < PULSE_BAND; i++) {
        s_motion_pulse[i] = lv_arc_create(rbox);
        lv_obj_remove_style(s_motion_pulse[i], NULL, LV_PART_KNOB);
        lv_obj_clear_flag(s_motion_pulse[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(s_motion_pulse[i], 2, 2);
        lv_obj_set_pos(s_motion_pulse[i], FAN_APEX_X - 1, FAN_APEX_Y - 1);
        lv_arc_set_rotation(s_motion_pulse[i], 0);
        lv_arc_set_bg_angles(s_motion_pulse[i], FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_arc_set_angles(s_motion_pulse[i], FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_obj_set_style_arc_width(s_motion_pulse[i], PULSE_WIDTH + 1, LV_PART_MAIN);
        lv_obj_set_style_arc_color(s_motion_pulse[i], COL_AMBER, LV_PART_MAIN);
        lv_obj_set_style_arc_width(s_motion_pulse[i], 0, LV_PART_INDICATOR);
        lv_obj_set_style_opa(s_motion_pulse[i], LV_OPA_TRANSP, 0);
    }
    s_pulse_phase = 0;
```
(Sub-arc draw width is `PULSE_WIDTH + 1` so neighbouring rings overlap by 1 px and read as one solid band with no seams.)

- [ ] **Step 3: Replace the observer pulse block**

Current (~4489):
```c
        /* Radial pulse: expand from apex to FAN_R then reset, comet-tail fade. */
        static const int TRAIL_STEP = 40;
        static const uint8_t TRAIL_OPA[PULSE_STEPS] = {220, 150, 90, 45, 15};
        s_pulse_phase = (s_pulse_phase + 6) % (FAN_R + PULSE_STEPS * TRAIL_STEP);
        for (int i = 0; i < PULSE_STEPS; i++) {
            if (!s_motion_pulse[i]) break;
            int ri = s_pulse_phase - i * TRAIL_STEP;
            if (ri <= 0 || ri > FAN_R) {
                lv_obj_set_style_opa(s_motion_pulse[i], LV_OPA_TRANSP, 0);
            } else {
                lv_obj_set_size(s_motion_pulse[i], ri * 2, ri * 2);
                lv_obj_set_pos(s_motion_pulse[i], FAN_APEX_X - ri, FAN_APEX_Y - ri);
                lv_obj_set_style_opa(s_motion_pulse[i], TRAIL_OPA[i], 0);
            }
        }
```
Change to a single band — `j=0` is the bright leading edge at radius `s_pulse_phase`, each deeper sub-arc sits `PULSE_WIDTH` px inward and dimmer; whole band fades out over the last 16 px of the rim:
```c
        /* Single Halo-style pulse: one thick band sweeps apex->rim and repeats.
         * j=0 = bright leading (outer) edge; opacity ramps to 0 at the trailing edge. */
        s_pulse_phase = (s_pulse_phase + 6) % (FAN_R + PULSE_BAND * PULSE_WIDTH);
        for (int j = 0; j < PULSE_BAND; j++) {
            if (!s_motion_pulse[j]) break;
            int rj = s_pulse_phase - j * PULSE_WIDTH;
            if (rj <= 0 || rj > FAN_R) {
                lv_obj_set_style_opa(s_motion_pulse[j], LV_OPA_TRANSP, 0);
                continue;
            }
            lv_obj_set_size(s_motion_pulse[j], rj * 2, rj * 2);
            lv_obj_set_pos(s_motion_pulse[j], FAN_APEX_X - rj, FAN_APEX_Y - rj);
            int opa = 235 - (235 * j) / (PULSE_BAND - 1);   /* 235 (lead) -> 0 (trail) */
            if (rj > FAN_R - 16) opa = opa * (FAN_R - rj) / 16;  /* fade into the rim */
            lv_obj_set_style_opa(s_motion_pulse[j], (lv_opa_t)opa, 0);
        }
```

- [ ] **Step 4: Update `close_panel` loop bound**

Current (~410):
```c
    for (int i = 0; i < PULSE_STEPS; i++) s_motion_pulse[i] = NULL;
```
Change to:
```c
    for (int i = 0; i < PULSE_BAND; i++) s_motion_pulse[i] = NULL;
```

- [ ] **Step 5: Build**

Run build. Expected: `Project build complete`. (If a `PULSE_STEPS` reference remains it errors `undefined` — grep `PULSE_STEPS` to confirm zero hits.)

- [ ] **Step 6: Flash + screenshot**

`python tools/prop.py shot radar.png --screen motion --crop 10,64,440,440 --zoom 2 --wait`.
Expected: one thick amber band sweeping outward from the apex, bright on its outer edge and fading to nothing on its inner edge (not 5 separate thin rings). Shoot twice ~0.3 s apart to confirm it travels and repeats.

- [ ] **Step 7: Commit**

```bash
git add main/prop_ui.c
git commit -m "feat(motion): replace 5-arc pulse with single Halo-style gradient band"
```

---

## Task 3: Unify scale onto animated `ppm` + sliding gridlines

**Files:**
- Modify: `main/prop_ui.c` — new statics (~214), build (replace static arcs/legend/ticks ~3282-3353), new grid-layout helper, observer blip math (~4516) + call helper, `close_panel` (~416).

**Interfaces:**
- Consumes: `FAN_APEX_X/Y`, `FAN_R`, `FAN_RIGHT_DEG`, `label_set_text_cached()`.
- Produces:
  - `static float s_motion_ppm;` — current pixels-per-metre (this task: fixed `FAN_R/6`).
  - `static lv_obj_t *s_motion_grid_arc[6];` / `s_motion_grid_lbl[6];` — major-ring pool (1..6 m).
  - `static void motion_layout_grid(void);` — positions the pool from `s_motion_ppm` each tick.

**Why:** Today the grid (fixed arcs labelled 1m/2m at 87/174 px) and blips (`dist·FAN_R/6000` ⇒ 40 px/m) disagree. Route both through one `ppm` so the grid is honest and ready to animate.

- [ ] **Step 1: Add statics**

After line ~214 (`static lv_obj_t *s_motion_dir_dot;`), add:
```c
/* Animated radar scale: one pixels-per-metre value drives arcs, labels, and blips. */
static float     s_motion_ppm;            /* current px per metre (animated in Task 4) */
static lv_obj_t *s_motion_grid_arc[6];    /* major range rings, 1..6 m */
static lv_obj_t *s_motion_grid_lbl[6];    /* "Nm" labels riding the right fan edge */
```

- [ ] **Step 2: Replace static arcs / legend / boresight quarter-ticks in `build_motion_panel`**

Delete three current blocks: the major-arc creation loop (`for (int r = 87; r <= FAN_R; r += 87) { ... }`, ~3283-3295), the quarter-metre boresight ticks block (`static lv_point_precise_t s_bore_ticks[8][2]; { ... }`, ~3320-3336), and the `"1m"/"2m"` legend block (~3338-3353). **Keep** the fan edge + boresight line block (`s_fan_left/right/bore`, ~3297-3318) unchanged.

In their place (after the fan edge block), create the major-ring pool, hidden:
```c
    /* Major range-ring pool (1..6 m). Positioned each tick from s_motion_ppm so the
     * rings slide as the scale animates; created hidden. */
    for (int i = 0; i < 6; i++) {
        lv_obj_t *arc = lv_arc_create(rbox);
        lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
        lv_arc_set_rotation(arc, 0);
        lv_arc_set_bg_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_arc_set_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_obj_set_style_arc_width(arc, 1, LV_PART_MAIN);
        lv_obj_set_style_arc_color(arc, COL_DIM, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
        lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
        s_motion_grid_arc[i] = arc;

        lv_obj_t *lbl = lv_label_create(rbox);
        lv_label_set_text(lbl, "");
        lv_obj_set_style_text_font(lbl, FONT_BODY, 0);
        lv_obj_set_style_text_color(lbl, COL_DIM, 0);
        lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
        s_motion_grid_lbl[i] = lbl;
    }
    s_motion_ppm = (float)FAN_R / 6.0f;   /* fixed 6 m full-scale until Task 4 */
```

- [ ] **Step 3: Add the grid-layout helper**

Add above `ui_observer` (e.g. just after `clamp_pct`, ~3264):
```c
/* Lay out the major range rings + labels from s_motion_ppm. Rings at fixed real
 * distances (1..6 m) slide as ppm changes; the outermost fades over the last 14 px
 * of the rim and parks when it crosses it. */
static void motion_layout_grid(void)
{
    const float d2r = 3.14159265f / 180.0f;
    const float re  = FAN_RIGHT_DEG * d2r;       /* right fan edge for labels */
    for (int d = 1; d <= 6; d++) {
        lv_obj_t *arc = s_motion_grid_arc[d - 1];
        lv_obj_t *lbl = s_motion_grid_lbl[d - 1];
        if (!arc) break;
        int r = (int)(d * s_motion_ppm + 0.5f);
        if (r < 8 || r > FAN_R) {
            lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        int opa = LV_OPA_COVER;
        if (r > FAN_R - 14) opa = LV_OPA_COVER * (FAN_R - r) / 14;   /* fade into rim */
        lv_obj_set_size(arc, r * 2, r * 2);
        lv_obj_set_pos(arc, FAN_APEX_X - r, FAN_APEX_Y - r);
        lv_obj_set_style_arc_opa(arc, (lv_opa_t)opa, LV_PART_MAIN);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_HIDDEN);

        char tb[8];
        snprintf(tb, sizeof(tb), "%dm", d);
        label_set_text_cached(lbl, tb);
        lv_obj_set_pos(lbl, (int)(FAN_APEX_X + cosf(re) * r) + 4,
                            (int)(FAN_APEX_Y + sinf(re) * r) - 10);
        lv_obj_set_style_text_opa(lbl, (lv_opa_t)opa, 0);
        lv_obj_clear_flag(lbl, LV_OBJ_FLAG_HIDDEN);
    }
}
```

- [ ] **Step 4: Call the helper + fix blip scale in the observer**

In the `PK_MOTION` block, right after the pulse loop, add:
```c
        motion_layout_grid();
```
Then change the blip radius (current ~4516):
```c
                float r = dist * (float)FAN_R / 6000.0f; if (r > FAN_R) r = (float)FAN_R;
```
to use `ppm` (note `dist` is in mm here):
```c
                float r = (dist / 1000.0f) * s_motion_ppm; if (r > FAN_R) r = (float)FAN_R;
```

- [ ] **Step 5: NULL the pool in `close_panel`**

After the `s_motion_dir_dot = NULL;` line (~416), add:
```c
    for (int i = 0; i < 6; i++) { s_motion_grid_arc[i] = NULL; s_motion_grid_lbl[i] = NULL; }
```

- [ ] **Step 6: Build**

Run build. Expected: `Project build complete`.

- [ ] **Step 7: Flash + screenshot**

`python tools/prop.py shot radar.png --screen motion --crop 10,64,440,440 --zoom 2 --wait`.
Expected: six faint rings labelled `1m`..`6m` at 40 px spacing; the `6m` ring sits at the rim and is faint. Wave a hand ~1 m in front of the LD2450 — the blip should land **on the `1m` ring** (grid + blip now agree). The outer rings should not abruptly clip — the 6m ring is dimmed by the fade.

- [ ] **Step 8: Commit**

```bash
git add main/prop_ui.c
git commit -m "feat(motion): route grid + blips through one px-per-metre scale"
```

---

## Task 4: Autorange state machine + smooth slide

**Files:**
- Modify: `main/prop_ui.c` — new statics + ladder (~after Task 3 statics), build init, observer (compute furthest, run state machine, advance ease) before `motion_layout_grid()`.

**Interfaces:**
- Consumes: `s_motion_ppm`, `MOTION_LADDER`, target cache (`prop_motion_get_targets`, already fetched as `tgts`/`cnt` in the observer).
- Produces:
  - `static const float MOTION_LADDER[5]` / `#define MOTION_NLEVEL 5`.
  - `s_motion_level` (committed ladder index), `s_motion_level_pend`, `s_motion_dwell`.
  - `s_motion_ppm_from`, `s_motion_ppm_to`, `s_motion_anim_ms` (-1 = idle).
  - `static int motion_desired_level(float furthest_m, int have_target);`

**Why:** Drive `s_motion_ppm` from the furthest target via a 5-level ladder, with hysteresis (dwell) so re-ranging happens in single deliberate chunks, eased over ~500 ms.

- [ ] **Step 1: Add ladder + state**

After the Task 3 statics block, add:
```c
/* Autorange ladder — full-scale distance (m) at the rim. LD2450 max ~6 m. */
static const float MOTION_LADDER[5] = { 1.0f, 2.0f, 3.0f, 4.5f, 6.0f };
#define MOTION_NLEVEL 5
static int   s_motion_level;        /* committed ladder index */
static int   s_motion_level_pend;   /* candidate index awaiting dwell */
static int   s_motion_dwell;        /* ticks the candidate has persisted */
static float s_motion_ppm_from;     /* slide start ppm */
static float s_motion_ppm_to;       /* slide target ppm */
static int   s_motion_anim_ms;      /* slide elapsed ms; -1 = idle */
```

- [ ] **Step 2: Add the level-selection helper**

Add next to `motion_layout_grid` (above `ui_observer`):
```c
/* Smallest ladder level keeping the furthest target at <=80% radius; 6 m when idle. */
static int motion_desired_level(float furthest_m, int have_target)
{
    if (!have_target) return MOTION_NLEVEL - 1;
    for (int i = 0; i < MOTION_NLEVEL; i++)
        if (furthest_m <= 0.80f * MOTION_LADDER[i]) return i;
    return MOTION_NLEVEL - 1;
}
```

- [ ] **Step 3: Initialise state in `build_motion_panel`**

Replace the Task 3 init line `s_motion_ppm = (float)FAN_R / 6.0f;` with:
```c
    s_motion_level = MOTION_NLEVEL - 1;            /* boot at full 6 m, zoom in as targets appear */
    s_motion_level_pend = s_motion_level;
    s_motion_dwell = 0;
    s_motion_ppm = (float)FAN_R / MOTION_LADDER[s_motion_level];
    s_motion_ppm_to = s_motion_ppm;
    s_motion_ppm_from = s_motion_ppm;
    s_motion_anim_ms = -1;
```

- [ ] **Step 4: Run the state machine in the observer**

In the `PK_MOTION` block, after `cnt = prop_motion_get_targets(tgts, 3);` and before the blip loop, compute the furthest range and step the autorange. Insert:
```c
        /* --- Autorange: pick a ladder level from the furthest target, debounce,
         * then ease s_motion_ppm toward it. One deliberate zoom per threshold. --- */
        float furthest_m = 0.0f;
        for (int i = 0; i < cnt; i++) {
            float fx = (float)tgts[i].x_mm, fy = (float)tgts[i].y_mm;
            float dm = sqrtf(fx * fx + fy * fy) / 1000.0f;
            if (dm > furthest_m) furthest_m = dm;
        }
        int desired = motion_desired_level(furthest_m, cnt > 0);
        if (desired == s_motion_level) {
            s_motion_dwell = 0;
        } else {
            if (desired != s_motion_level_pend) { s_motion_level_pend = desired; s_motion_dwell = 0; }
            else                                 { s_motion_dwell++; }
            /* 0.4 s dwell for target-driven changes; 3 s before the idle decay to 6 m. */
            int need = (desired == MOTION_NLEVEL - 1 && cnt == 0) ? 60 : 8;
            if (s_motion_dwell >= need) {
                s_motion_level    = desired;
                s_motion_ppm_from = s_motion_ppm;
                s_motion_ppm_to   = (float)FAN_R / MOTION_LADDER[desired];
                s_motion_anim_ms  = 0;
                s_motion_dwell    = 0;
            }
        }
        if (s_motion_anim_ms >= 0) {            /* ~50 ms per observer tick (20 Hz) */
            s_motion_anim_ms += 50;
            float t = s_motion_anim_ms / 500.0f;   /* 500 ms slide */
            if (t >= 1.0f) { t = 1.0f; s_motion_anim_ms = -1; }
            float e = t * t * (3.0f - 2.0f * t);   /* smoothstep ease-in-out */
            s_motion_ppm = s_motion_ppm_from + (s_motion_ppm_to - s_motion_ppm_from) * e;
        }
```
(`motion_layout_grid()` and the blip loop already consume the updated `s_motion_ppm`.)

- [ ] **Step 5: Add the new state to `close_panel`**

Reset is not strictly required (re-initialised in build), but for cleanliness add after the grid-pool NULL line:
```c
    s_motion_anim_ms = -1; s_motion_dwell = 0;
```

- [ ] **Step 6: Build**

Run build. Expected: `Project build complete`.

- [ ] **Step 7: Flash + screenshot the behaviour**

Flash. With no target, confirm the scope rests at 6 m (`6m` ring at rim). Then wave a hand and walk it in from ~1.5 m to ~0.5 m in front of the LD2450, taking shots:
`python tools/prop.py shot near.png --screen motion --crop 10,64,440,440 --zoom 2 --wait`.
Expected: as the contact crosses ~0.8 m the scope makes **one** smooth zoom; the outer ring slides off the rim and the labelled rings respread (e.g. 2 m scale → 1 m scale leaves `1m` as the furthest ring). Removing the target returns to 6 m after ~3 s. A target hovering near a boundary must not oscillate between levels.

- [ ] **Step 8: Commit**

```bash
git add main/prop_ui.c
git commit -m "feat(motion): autorange radar in eased chunks with hysteresis"
```

---

## Task 5: Minor 0.5 m rings at close scale

**Files:**
- Modify: `main/prop_ui.c` — minor-ring statics (~after Task 4 statics), build pool, `motion_layout_grid` (extend), `close_panel`.

**Interfaces:**
- Consumes: `s_motion_ppm`, `MOTION_LADDER[s_motion_level]`, `FAN_APEX_X/Y`, `FAN_R`.
- Produces: `static lv_obj_t *s_motion_grid_minor[2];` — half-metre rings shown only when the committed scale ≤ 2 m.

**Why:** At the 1 m and 2 m levels, half-metre rings add close-range detail without clutter at wider scales. At scale 2 m only 0.5 m and 1.5 m fit (2.5 m → 300 px > FAN_R); at scale 1 m only 0.5 m fits. Two arcs suffice.

- [ ] **Step 1: Add statics**

After the Task 3/4 statics:
```c
static lv_obj_t *s_motion_grid_minor[2];  /* 0.5 m / 1.5 m rings, shown when scale <= 2 m */
```

- [ ] **Step 2: Build the minor pool**

In `build_motion_panel`, right after the major-ring pool loop, add:
```c
    /* Minor half-metre rings (close-range detail); created hidden, even fainter. */
    for (int i = 0; i < 2; i++) {
        lv_obj_t *arc = lv_arc_create(rbox);
        lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
        lv_arc_set_rotation(arc, 0);
        lv_arc_set_bg_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_arc_set_angles(arc, FAN_LEFT_DEG, FAN_RIGHT_DEG);
        lv_obj_set_style_arc_width(arc, 1, LV_PART_MAIN);
        lv_obj_set_style_arc_color(arc, COL_DIM, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(arc, LV_OPA_40, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
        lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
        s_motion_grid_minor[i] = arc;
    }
```

- [ ] **Step 3: Extend `motion_layout_grid` with the minor rings**

At the end of `motion_layout_grid` (before its closing brace), add:
```c
    /* Minor 0.5 m / 1.5 m rings — only at the close scales (<= 2 m), else hidden. */
    static const float minor_d[2] = { 0.5f, 1.5f };
    int show_minor = (MOTION_LADDER[s_motion_level] <= 2.0f);
    for (int i = 0; i < 2; i++) {
        lv_obj_t *arc = s_motion_grid_minor[i];
        if (!arc) break;
        int r = (int)(minor_d[i] * s_motion_ppm + 0.5f);
        if (!show_minor || r < 8 || r > FAN_R) {
            lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_set_size(arc, r * 2, r * 2);
        lv_obj_set_pos(arc, FAN_APEX_X - r, FAN_APEX_Y - r);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_HIDDEN);
    }
```
(`s_motion_level` is defined in Task 4. Implement Task 5 after Task 4.)

- [ ] **Step 4: NULL the minor pool in `close_panel`**

After the major-pool NULL line:
```c
    for (int i = 0; i < 2; i++) s_motion_grid_minor[i] = NULL;
```

- [ ] **Step 5: Build**

Run build. Expected: `Project build complete`.

- [ ] **Step 6: Flash + screenshot**

Drive a close target so the scope settles at the 1 m or 2 m level, then:
`python tools/prop.py shot minor.png --screen motion --crop 10,64,440,440 --zoom 2 --wait`.
Expected: faint half-metre rings appear between the labelled 1m/2m rings at the close scales, and vanish once the scope ranges out to 3 m+.

- [ ] **Step 7: Commit**

```bash
git add main/prop_ui.c
git commit -m "feat(motion): add 0.5 m minor rings at close radar scales"
```

---

## Task 6: Comet tails on targets (direction of travel)

**Files:**
- Modify: `main/prop_ui.c` — trail statics (~after Task 5 statics), shared screen-pos helper (above `ui_observer`), trail pool in build (before the blip pool), blip+trail render in observer (replace the blip loop), `close_panel`.

**Interfaces:**
- Consumes: `s_motion_ppm`, `FAN_APEX_X/Y`, `FAN_R`, `FAN_HALF_DEG`, target cache `tgts`/`cnt`.
- Produces:
  - `static void motion_target_screen_pos(int x_mm, int y_mm, int *bx, int *by);` — maps an LD2450 sample to a fan screen point at the current scale (shared by blips + trails).
  - `s_motion_trail[3][TRAIL_LEN]` dot pool, plus `s_trail_x/y[3][TRAIL_LEN]` (world-mm history), `s_trail_n[3]`, `s_trail_head[3]`, `s_trail_tick`.

**Why:** A fading dot trail behind each blip shows where the target came from → direction of travel. History is stored in world coordinates (mm) and re-projected each frame, so trails rescale smoothly during autorange instead of smearing.

- [ ] **Step 1: Add trail statics**

After the Task 5 statics, add:
```c
/* Comet trails: per-target ring buffer of recent world samples (mm), re-projected
 * each frame so the tail rescales with autorange. */
#define TRAIL_LEN 8
static lv_obj_t *s_motion_trail[3][TRAIL_LEN];   /* fading dot pool per target */
static int s_trail_x[3][TRAIL_LEN], s_trail_y[3][TRAIL_LEN];  /* world history (mm) */
static int s_trail_n[3];      /* valid samples per slot */
static int s_trail_head[3];   /* ring-buffer head (newest) */
static int s_trail_tick;      /* sample decimation counter */
```

- [ ] **Step 2: Add the shared screen-position helper**

Add next to `motion_layout_grid` (above `ui_observer`):
```c
/* Map an LD2450 target (mm, sign-agnostic) to a fan screen point at the current
 * scale. Shared by the blips and their comet trails. */
static void motion_target_screen_pos(int x_mm, int y_mm, int *bx, int *by)
{
    const float maxoff = (float)FAN_HALF_DEG * (3.14159265f / 180.0f);
    float fx = (float)x_mm, fy = (float)y_mm;
    float dist = sqrtf(fx * fx + fy * fy);
    float r = (dist / 1000.0f) * s_motion_ppm;
    if (r > (float)FAN_R) r = (float)FAN_R;
    float off = atan2f(fx, fabsf(fy) + 1.0f);
    if (off >  maxoff) off =  maxoff;
    if (off < -maxoff) off = -maxoff;
    float scr = (270.0f * (3.14159265f / 180.0f)) + off;
    *bx = (int)(FAN_APEX_X + cosf(scr) * r);
    *by = (int)(FAN_APEX_Y + sinf(scr) * r);
}
```

- [ ] **Step 3: Build the trail pool (before the blip pool)**

In `build_motion_panel`, immediately before the `/* 3 target blips ... */` creation loop, add (so trails render *behind* the blips):
```c
    /* Comet-tail dot pool (per target), created hidden — drawn behind the blips. */
    for (int i = 0; i < 3; i++) {
        s_trail_n[i] = 0; s_trail_head[i] = 0;
        for (int k = 0; k < TRAIL_LEN; k++) {
            lv_obj_t *d = lv_obj_create(rbox);
            lv_obj_remove_style_all(d);
            lv_obj_set_size(d, 4, 4);
            lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(d, COL_AMBER, 0);
            lv_obj_set_style_bg_opa(d, LV_OPA_TRANSP, 0);
            lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
            s_motion_trail[i][k] = d;
        }
    }
    s_trail_tick = 0;
```

- [ ] **Step 4: Replace the blip loop with blip + trail render**

In the observer, delete the now-unused `const float maxoff = ...;` line (the helper owns it) that sits just above the blip loop, and replace the entire blip `for (int i = 0; i < 3; i++) { ... }` loop with:
```c
        /* Sample decimation for trails: push ~every 3rd tick => ~1.2 s of history. */
        s_trail_tick++;
        int trail_push = (s_trail_tick % 3 == 0);

        for (int i = 0; i < 3; i++) {
            if (!s_motion_blips[i]) break;
            if (i < cnt) {
                int bx, by;
                motion_target_screen_pos(tgts[i].x_mm, tgts[i].y_mm, &bx, &by);
                lv_obj_set_pos(s_motion_blips[i], bx - 6, by - 6);
                lv_obj_set_style_bg_color(s_motion_blips[i],
                    (tgts[i].speed_mm_s > 200 || tgts[i].speed_mm_s < -200)
                        ? COL_ALERT : COL_AMBER, 0);
                lv_obj_clear_flag(s_motion_blips[i], LV_OBJ_FLAG_HIDDEN);

                if (trail_push) {
                    s_trail_head[i] = (s_trail_head[i] + 1) % TRAIL_LEN;
                    s_trail_x[i][s_trail_head[i]] = tgts[i].x_mm;
                    s_trail_y[i][s_trail_head[i]] = tgts[i].y_mm;
                    if (s_trail_n[i] < TRAIL_LEN) s_trail_n[i]++;
                }
            } else {
                lv_obj_add_flag(s_motion_blips[i], LV_OBJ_FLAG_HIDDEN);
                s_trail_n[i] = 0;
            }

            /* Comet trail: k=0 newest (brightest/biggest) -> oldest faint/small.
             * Re-projected each tick so it rescales smoothly during autorange. */
            for (int k = 0; k < TRAIL_LEN; k++) {
                lv_obj_t *d = s_motion_trail[i][k];
                if (!d) break;
                if (k < s_trail_n[i]) {
                    int idx = (s_trail_head[i] - k + TRAIL_LEN) % TRAIL_LEN;
                    int tx, ty;
                    motion_target_screen_pos(s_trail_x[i][idx], s_trail_y[i][idx], &tx, &ty);
                    int sz  = 8 - (6 * k) / (TRAIL_LEN - 1);       /* 8 -> 2 px */
                    int opa = 170 - (170 * k) / (TRAIL_LEN - 1);   /* 170 -> 0 */
                    lv_obj_set_size(d, sz, sz);
                    lv_obj_set_pos(d, tx - sz / 2, ty - sz / 2);
                    lv_obj_set_style_bg_opa(d, (lv_opa_t)opa, 0);
                    lv_obj_clear_flag(d, LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
```

- [ ] **Step 5: NULL the trail pool in `close_panel`**

After the minor-pool NULL line:
```c
    for (int i = 0; i < 3; i++) {
        s_trail_n[i] = 0; s_trail_head[i] = 0;
        for (int k = 0; k < TRAIL_LEN; k++) s_motion_trail[i][k] = NULL;
    }
    s_trail_tick = 0;
```

- [ ] **Step 6: Build**

Run build. Expected: `Project build complete`. (Confirm no `unused variable 'maxoff'` warning — the line must be deleted.)

- [ ] **Step 7: Flash + screenshot**

Drive a moving target across the fan, then shoot mid-motion:
`python tools/prop.py shot trail.png --screen motion --crop 10,64,440,440 --zoom 2 --wait`.
Expected: each blip drags a fading dot tail pointing back along its path; the tail shrinks/dims toward the oldest sample, and points opposite the direction of travel. Tail should rescale (not smear) when the scope autoranges.

- [ ] **Step 8: Commit** (deferred — see Global Constraints note; held until end-of-run user review)

```bash
git add main/prop_ui.c
git commit -m "feat(motion): comet tails on targets to show direction of travel"
```

---

## Self-Review

**Spec coverage:**
- §1 Autoranging — single `ppm` (Task 3), ladder + 0.8 selection + dwell hysteresis + idle 6 m (Task 4), eased slide (Task 4), grid pool slide/fade (Task 3), minor rings (Task 5), blip on `ppm` (Task 3). ✓
- §2 Reposition fan — Task 1 (apex raised, screenshot-tuned). ✓
- §3 Halo pulse — Task 2. ✓
- Comet tails (user addition, post-spec) — Task 6; world-coord history re-projected each frame so tails rescale with autorange. ✓
- Pre-existing grid/blip mismatch — fixed in Task 3 (both on `ppm`). ✓
- Testing/verification — every task ends with build + `/screenshot`; the user's 1.5 m→0.5 m example is Task 4 Step 7. ✓

**Placeholder scan:** No TBD/TODO; every code step shows complete code. `FAN_APEX_Y`/`FAN_R` tuning in Task 1 is an explicit screenshot-driven loop with concrete starting values and step sizes, not a placeholder.

**Type consistency:** `s_motion_ppm` (float) defined Task 3, consumed Tasks 3–5. `motion_layout_grid(void)` defined Task 3, extended Task 5, called Task 3 Step 4. `motion_desired_level(float,int)` defined/called Task 4. `s_motion_level` defined Task 4, used Task 5 (ordering noted). `PULSE_BAND`/`PULSE_WIDTH` replace `PULSE_STEPS` at all four sites (decl/build/observer/close). Pool sizes consistent: major `[6]`, minor `[2]`, pulse `[PULSE_BAND]`.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-27-motion-scan-autorange.md`.
