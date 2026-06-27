# MOTION SCAN — Autoranging Radar, Fan Reposition, Halo Pulse

**Date:** 2026-06-27
**Scope:** `main/prop_ui.c` only — `build_motion_panel()` and the `PK_MOTION` block of
`ui_observer`. No engine/driver/content changes.
**Status:** Approved design, pending spec review.

## Motivation

The MOTION SCAN pie-slice radar uses a fixed scale, which wastes most of the display when
targets are close. Three changes make it more useful and better-looking:

1. **Autoranging** — the radar zooms (in discrete chunks, animated smoothly) so the furthest
   target always fills a useful fraction of the scope, while preserving a true sense of scale.
2. **Reposition the fan** higher in its box so the geometry stops colliding with the bottom
   status overlay (sensor bars + flag words + direction ring).
3. **Replace the 5-arc pulse** with a single thick gradient pulse (Halo motion-tracker style):
   opaque leading edge fading to transparent at the trailing edge.

### Pre-existing bug this fixes

Grid and blips currently disagree on scale. Range arcs are created once at fixed pixel radii
(87, 174, 240 px) and the legend labels them "1m"/"2m" ([prop_ui.c:3283](../../../main/prop_ui.c),
[:3338](../../../main/prop_ui.c)), but blips are placed with `r = dist_mm · FAN_R / 6000`
(6 m → 240 px ⇒ 40 px/m, [prop_ui.c:4516](../../../main/prop_ui.c)). A real 1 m target lands at
40 px, not on the "1m" ring at 87 px. The legend is decorative, not truthful. The autorange work
collapses everything onto one source of truth (`ppm`, pixels-per-metre), making the grid honest.

---

## 1. Autoranging

### Single source of truth

Introduce an animated scale `ppm` (pixels per metre): `ppm = FAN_R / scale_m`. Arcs, range
labels, boresight ticks, and blips all derive from `ppm` every observer tick.

### Scale ladder & selection

Discrete full-scale levels (distance at the rim `FAN_R`):

```
LADDER = { 1.0, 2.0, 3.0, 4.5, 6.0 }  metres   (LD2450 max range ≈ 6 m)
```

Each tick:
- Compute `furthest_m` = max true range over currently tracked targets (`sqrt(x²+y²)/1000`).
- Desired level = **smallest L in LADDER such that `furthest_m ≤ 0.80 · L`** (target sits at
  ≤ 80 % radius, leaving headroom). If `furthest_m` exceeds `0.80·6`, clamp to 6 m (targets
  beyond 6 m clamp to the rim anyway).
- **No targets** → desired level = 6 m (full scale).

### "One chunk" behaviour (anti-flap)

Desired level is debounced before it commits, so range changes are deliberate, not continuous:

- A `pending` level and a `dwell` counter. When `desired != committed`, count up; when
  `desired` returns to `committed` before dwell elapses, reset the counter.
- **Dwell:** ~0.4 s (8 ticks @ 20 Hz) for target-driven changes; ~3 s (60 ticks) for the
  no-target decay back to 6 m. The longer idle dwell prevents a jarring zoom-out the instant a
  contact drops or flickers at the FOV edge.
- On commit: set `committed = pending`, start the slide animation.

### Smooth slide

On commit, ease `ppm` from its current value to `FAN_R / scale_m[committed]`:

- Capture `ppm_from = ppm_current`, `ppm_to = target`, reset `anim_t = 0`.
- Each tick advance `anim_t` by the observer delta (~50 ms); over `DUR ≈ 500 ms` apply an
  ease-in-out (smoothstep). `ppm = lerp(ppm_from, ppm_to, ease(anim_t/DUR))`.
- If a new level commits mid-slide, retarget `ppm_from = ppm_current` and restart `anim_t`
  (no snap).

Because every ring radius is `dist · ppm`, rings physically slide: zooming in grows `ppm`,
the outer ring crosses `FAN_R`, fades over its last ~12 px and parks; inner rings spread apart.
Blips ride the same `ppm`, so grid and targets zoom as one field — matching the "2 m gridline
slides off, leaving 1 m" requirement.

### Grid rendering (reuse pool, no churn)

Replace the 3 static arcs + fixed legend labels + quarter-metre boresight ticks with a fixed
pool sized for the widest case (1–6 m at 1 m spacing):

- `s_motion_grid_arc[6]`, `s_motion_grid_lbl[6]` — created once in `build_motion_panel`.
- Each tick, for `d = 1..6` m:
  - `r = d · ppm`.
  - If `r ≤ FAN_R + pad`: size/position the arc to `r` across the full fan angle
    (`FAN_LEFT_DEG..FAN_RIGHT_DEG`); opacity fades in/out within ~12 px of the rim
    (and near the apex). Label `"{d}m"` rides the right fan edge at radius `r`, same fade.
  - Else: hide arc + label.
- **Minor 0.5 m rings:** when `scale_m ≤ 2` (levels 1 m and 2 m), show fainter half-metre rings
  for close-range detail. Implemented as part of the same pool pass or a small dedicated minor
  pool — decided in the plan.
- Fan edge lines + boresight line stay static (unchanged).

### Lifecycle

- New `s_motion_*` statics live near the other MOTION SCAN statics (~[prop_ui.c:194](../../../main/prop_ui.c)).
- Grid pool pointers NULLed in `close_panel`; all observer use guarded by
  `s_cur_kind == PK_MOTION && s_motion_grid_arc[0]`, per the panel-lifecycle rules in CLAUDE.md.
- On panel open: init `committed = 6 m`, `ppm = FAN_R/6`, no animation in progress; let it zoom
  in as targets appear.

### Blip placement change

In the observer, replace `r = dist · FAN_R / 6000` with `r = dist_m · ppm` (clamp to `FAN_R`).
Bearing/offset math (`atan2f`, fan clamp) is unchanged.

---

## 2. Reposition the fan

The fan apex currently sits low in the radar box (`FAN_APEX_Y = 410` of a 440 px box), leaving
only ~30 px below it; the bottom-anchored sensor bars (bottom-left), flag words (bottom-right),
and the direction ring at the apex crowd into the fan's lower flanks.

- **Raise the apex** (decrease `FAN_APEX_Y`, e.g. 410 → ~330) so a clear band opens below it for
  the bars/flags overlay. The direction ring is drawn at the apex and moves up with it (good —
  it separates from the overlay too). Fan top = `apex_y − FAN_R` stays > 0, so no clipping at the
  box top; reduce `FAN_R` slightly only if top clearance gets tight.
- Exact `FAN_APEX_Y` (and `FAN_R` if needed) are **tuned against a live `/screenshot`** of the
  panel, not guessed — the success check is "no fan geometry overlapping the bars/flags band."

---

## 3. Single Halo-style pulse

Replace the 5 spaced thin pulse arcs (`PULSE_STEPS = 5`, `TRAIL_STEP = 40`,
[prop_ui.c:4489](../../../main/prop_ui.c)) with **one thick pulse band** that expands from apex
to `FAN_R` and repeats, with a radial opacity gradient: opaque leading (outer) edge → 0 at the
trailing (inner) edge.

- LVGL arcs are single-opacity (no gradient across their width), so the band is rendered as a
  **contiguous stack of K thin sub-arcs** (e.g. K ≈ 9 at 4 px each ⇒ ~36 px band), no gaps —
  reads as one solid pulse, not 5 dots.
- Leading edge radius = `pulse_r` (the animated sweep position, full fan width). Sub-arc `j`
  (j = 0 leading) sits at radius `pulse_r − j·width`, opacity ramping from max at j = 0 to 0 at
  j = K−1 (linear or eased).
- Sweep apex → `FAN_R` then repeat. Clamp the trailing edge at the apex while the band is still
  emerging; optionally fade the whole band as it reaches the rim.
- The pulse is geometry-based (full `FAN_R` in px) and **independent of `ppm`/autorange** — it
  always sweeps the whole fan.
- Reuses an arc pool (the existing `s_motion_pulse[]`, resized to K and renamed/retuned); not new
  per-frame allocations.

---

## Testing / verification

No automated tests in this repo. Verify on the live panel via the screenshot loop
(`python tools/prop.py shot motion.png --screen scan --wait` / the MOTION SCAN deep-link),
driving synthetic targets where possible:

1. **Grid honesty:** a target at a known range sits on the matching labelled ring.
2. **Autorange chunks:** target closing from ~1.5 m to ~0.5 m triggers one smooth zoom; the 2 m
   ring slides off leaving 1 m as the furthest ring (the user's example).
3. **Anti-flap:** a target hovering near a level boundary does not oscillate.
4. **Idle decay:** removing all targets returns the scope to 6 m after ~3 s, smoothly.
5. **Fan clearance:** no fan arcs/labels/blips overlap the bottom bars/flags band.
6. **Pulse:** a single thick band sweeps apex→rim, bright leading edge fading to nothing behind.

## Out of scope

- Engine/driver changes, content changes.
- Manual zoom override / dial control of range (could be a follow-up).
- Nonlinear (log) radial compression (explicitly rejected — warps sense of scale).
