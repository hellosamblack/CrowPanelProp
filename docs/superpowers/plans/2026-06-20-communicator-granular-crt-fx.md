# Granular CRT FX Implementation Plan (Thread 1 of 4)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `prop_fx`'s single master intensity with independent per-effect intensity (scanlines, phosphor, vignette, refresh sweep), exposed as separate sliders in Setup ▸ Display and persisted in NVS.

**Architecture:** `prop_fx` keeps its lazy-allocated `lv_layer_top()` overlay (static baked canvas + scrolling refresh band). Today one global layer opacity scales a canvas baked at fixed per-effect constants. We move the three baked effects (scanlines/phosphor/vignette) to **per-effect alpha applied at bake time** and give the refresh band its **own layer opacity**, dropping the single global-opacity model. Each effect gets a 0–100 setting (0 = off), an NVS key, and a Display-panel slider. The legacy `fx_intensity` key seeds defaults on first boot after upgrade.

**Tech Stack:** ESP-IDF 6.0.1, LVGL 8.4 (`lv_canvas`/`lv_draw_rect`), NVS via `prop_settings`, esp_lvgl_port locking.

## Global Constraints

- **Target/board:** `esp32p4`, chip rev v1.3 — do NOT touch `CONFIG_ESP32P4_*REV*`.
- **LV_MEM hard-capped at 32 KB** — do not raise it. Overlay buffers stay in PSRAM (`MALLOC_CAP_SPIRAM`).
- **LVGL is 8.4** — v8 API (`LV_IMG_CF_TRUE_COLOR_ALPHA`, `lv_canvas_draw_rect`, `lv_slider_*`).
- **Any LVGL call off the LVGL task MUST hold `lvgl_port_lock()/unlock()`.** Setters here are called from the UI/LVGL event context (already locked) OR from the API task (must lock). Follow the existing `prop_fx_set_intensity` locking pattern exactly.
- **`/screenshot` captures the active screen ONLY, not the `lv_layer_top()` overlay.** The overlay look is verified on the physical panel; the Display *panel* (sliders) IS screenshot-verifiable.
- **Palette (from `prop_ui.c`):** `COL_AMBER 0xE0B000`, `COL_MUTE 0xB58A00`, `COL_DIM 0x6B5300`. Never use `COL_DIM` for text the viewer must read.
- **Build/flash (this machine):**
  ```powershell
  & "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
  idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
  idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" -p COM7 flash
  ```

---

## File Structure

- `firmware/communicator/main/include/prop_fx.h` — add per-effect setter/getter declarations.
- `firmware/communicator/main/prop_fx.c` — per-effect state, bake using per-effect alpha, NVS load/migrate, setters/getters.
- `firmware/communicator/main/prop_ui.c` — `build_display_panel` gains the four sliders (scrollable panel); replaces the single FX-intensity slider.
- `firmware/communicator/main/prop_api.c` — extend `{"cmd":"fx",...}` to accept per-effect values so the effects are scriptable (the only programmatic handle, since the overlay isn't screenshot-visible).

No new files.

---

## Task 1: Per-effect state + bake in `prop_fx.c`

**Files:**
- Modify: `firmware/communicator/main/prop_fx.c`

**Interfaces:**
- Consumes: nothing new (internal refactor).
- Produces (used by Tasks 2–4): four static `uint8_t` effect levels and a re-bake helper. Internal only this task; public setters added in Task 2.

**Background:** Today `paint_canvas()` bakes scanlines/phosphor/vignette at compile-time opacities (`FX_SCAN_OPA 95`, `FX_WASH_OPA 20`, `FX_VIGN_MAX 70`) and a single `s_intensity` scales the whole layer via `intensity_opa()`. We make each effect's *baked* alpha scale with its own 0–100 level, and give the refresh band its own level.

- [ ] **Step 1: Add per-effect state and default constants**

Replace the single intensity state near the top of `prop_fx.c`:

```c
/* Per-effect levels, 0..100 (0 = that effect contributes nothing). */
static uint8_t s_scan_pct     = 60;   /* scanline darkness        */
static uint8_t s_phosphor_pct = 30;   /* amber wash / glow        */
static uint8_t s_vignette_pct = 45;   /* edge falloff             */
static uint8_t s_refresh_pct  = 25;   /* scrolling sweep band     */
```

Remove the old `static uint8_t s_intensity = 55;` line. Keep `s_enabled`, `s_built`, and the canvas/band statics.

- [ ] **Step 2: Replace `intensity_opa()` with a per-level scaler**

```c
/* Scale a base alpha (0..255) by a 0..100 percent level. */
static lv_opa_t scale_opa(uint8_t base, uint8_t pct)
{
    if (pct > 100) pct = 100;
    return (lv_opa_t)((uint16_t)base * pct / 100);
}
```

Delete the old `intensity_opa()` function.

- [ ] **Step 3: Re-bake `paint_canvas()` using per-effect levels**

Each baked element now uses `scale_opa(BASE, level)` instead of the fixed constant, and the canvas is drawn at full layer opacity (per-effect strength now lives in the bake, not the layer opacity):

```c
static void paint_canvas(void)
{
    /* Phosphor amber wash. */
    lv_canvas_fill_bg(s_canvas, FX_AMBER, scale_opa(FX_WASH_OPA, s_phosphor_pct));

    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.radius = 0;
    d.border_width = 0;

    /* Horizontal scanlines. */
    d.bg_color = FX_BLACK;
    d.bg_opa = scale_opa(FX_SCAN_OPA, s_scan_pct);
    if (d.bg_opa) {
        for (int y = 0; y < FX_H; y += FX_SCAN_STEP) {
            lv_canvas_draw_rect(s_canvas, 0, y, FX_W, 1, &d);
        }
    }

    /* Vignette. */
    d.bg_color = FX_BLACK;
    for (int i = 0; i < FX_VIGN; i++) {
        lv_opa_t edge = (lv_opa_t)(FX_VIGN_MAX * (FX_VIGN - i) / FX_VIGN);
        d.bg_opa = scale_opa(edge, s_vignette_pct);
        if (!d.bg_opa) continue;
        lv_canvas_draw_rect(s_canvas, 0, i, FX_W, 1, &d);
        lv_canvas_draw_rect(s_canvas, 0, FX_H - 1 - i, FX_W, 1, &d);
        lv_canvas_draw_rect(s_canvas, i, 0, 1, FX_H, &d);
        lv_canvas_draw_rect(s_canvas, FX_W - 1 - i, 0, 1, FX_H, &d);
    }
}
```

- [ ] **Step 4: Bake the canvas at full opacity; band uses its own level**

In `fx_build()`, change the canvas opacity line from `intensity_opa(s_intensity)` to full cover, and the band line to use the refresh level:

```c
    paint_canvas();
    lv_obj_set_style_opa(s_canvas, LV_OPA_COVER, 0);
```

and for the band:

```c
        paint_band();
        lv_obj_set_style_opa(s_band, scale_opa(LV_OPA_COVER, s_refresh_pct), 0);
```

- [ ] **Step 5: Build**

Run:
```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
```
Expected: compiles clean (no references to the removed `intensity_opa`/`s_intensity`).

- [ ] **Step 6: Commit**

```bash
git add firmware/communicator/main/prop_fx.c
git commit -m "prop_fx: bake CRT effects at per-effect alpha (internal refactor)"
```

---

## Task 2: Per-effect setters/getters + NVS load/migrate

**Files:**
- Modify: `firmware/communicator/main/include/prop_fx.h`
- Modify: `firmware/communicator/main/prop_fx.c`

**Interfaces:**
- Produces (used by Tasks 3–4):
  - `void prop_fx_set_scanlines(uint8_t pct);` `uint8_t prop_fx_scanlines(void);`
  - `void prop_fx_set_phosphor(uint8_t pct);` `uint8_t prop_fx_phosphor(void);`
  - `void prop_fx_set_vignette(uint8_t pct);` `uint8_t prop_fx_vignette(void);`
  - `void prop_fx_set_refresh(uint8_t pct);` `uint8_t prop_fx_refresh(void);`
  - Existing `prop_fx_set_enabled`/`prop_fx_enabled` unchanged.
- Consumes: `prop_settings_get_u32`/`prop_settings_set_u32` from `prop_settings.h`.

- [ ] **Step 1: Declare the new API and retire the global-intensity API in the header**

In `prop_fx.h`, replace the two `prop_fx_set_intensity`/`prop_fx_intensity` lines with:

```c
/* Per-effect strength, 0..100 (0 = effect off). Each re-bakes / re-opacities the
 * overlay live and persists to NVS. Safe to call from any task (locks internally). */
void prop_fx_set_scanlines(uint8_t pct);
uint8_t prop_fx_scanlines(void);
void prop_fx_set_phosphor(uint8_t pct);
uint8_t prop_fx_phosphor(void);
void prop_fx_set_vignette(uint8_t pct);
uint8_t prop_fx_vignette(void);
void prop_fx_set_refresh(uint8_t pct);
uint8_t prop_fx_refresh(void);
```

- [ ] **Step 2: Load per-effect levels (with migration) in `prop_fx_init`**

Replace the `fx_intensity` load block at the top of `prop_fx_init()` with:

```c
    /* Migration: first boot after upgrade has no per-effect keys but may have the
     * legacy "fx_intensity". Seed each effect's default scaled by the old global
     * level so the look is roughly preserved; thereafter the per-effect keys win. */
    uint32_t legacy = 55;
    bool had_legacy = prop_settings_has("fx_intensity");
    prop_settings_get_u32("fx_intensity", &legacy, 55);

    uint32_t v;
    v = had_legacy ? (60u * legacy / 55u) : 60; prop_settings_get_u32("fx_scan",     &v, v); s_scan_pct     = (uint8_t)(v > 100 ? 100 : v);
    v = had_legacy ? (30u * legacy / 55u) : 30; prop_settings_get_u32("fx_phosphor", &v, v); s_phosphor_pct = (uint8_t)(v > 100 ? 100 : v);
    v = had_legacy ? (45u * legacy / 55u) : 45; prop_settings_get_u32("fx_vignette", &v, v); s_vignette_pct = (uint8_t)(v > 100 ? 100 : v);
    v = had_legacy ? (25u * legacy / 55u) : 25; prop_settings_get_u32("fx_refresh",  &v, v); s_refresh_pct  = (uint8_t)(v > 100 ? 100 : v);
```

Leave the existing `fx_on` load and the `if (s_enabled) { ... fx_build() ... }` block below it unchanged. Update the final `ESP_LOGI` to log the four levels instead of `s_intensity`.

- [ ] **Step 3: Replace `prop_fx_set_intensity`/`prop_fx_intensity` with the eight per-effect functions**

Delete the old two functions and add:

```c
/* Scanlines + phosphor + vignette are baked into the static canvas, so changing any
 * of them re-bakes that canvas (cheap; PSRAM, off the hot path). */
static void rebake_canvas_locked(void)
{
    if (s_built && s_canvas) {
        paint_canvas();
        lv_obj_invalidate(s_canvas);
    }
}

void prop_fx_set_scanlines(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_scan_pct = pct;
    if (lvgl_port_lock(200)) { rebake_canvas_locked(); lvgl_port_unlock(); }
    prop_settings_set_u32("fx_scan", pct);
}
uint8_t prop_fx_scanlines(void) { return s_scan_pct; }

void prop_fx_set_phosphor(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_phosphor_pct = pct;
    if (lvgl_port_lock(200)) { rebake_canvas_locked(); lvgl_port_unlock(); }
    prop_settings_set_u32("fx_phosphor", pct);
}
uint8_t prop_fx_phosphor(void) { return s_phosphor_pct; }

void prop_fx_set_vignette(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_vignette_pct = pct;
    if (lvgl_port_lock(200)) { rebake_canvas_locked(); lvgl_port_unlock(); }
    prop_settings_set_u32("fx_vignette", pct);
}
uint8_t prop_fx_vignette(void) { return s_vignette_pct; }

/* Refresh band has its own layer opacity (it is a separate floating canvas). */
void prop_fx_set_refresh(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_refresh_pct = pct;
    if (s_built && s_band && lvgl_port_lock(200)) {
        lv_obj_set_style_opa(s_band, scale_opa(LV_OPA_COVER, pct), 0);
        lvgl_port_unlock();
    }
    prop_settings_set_u32("fx_refresh", pct);
}
uint8_t prop_fx_refresh(void) { return s_refresh_pct; }
```

- [ ] **Step 4: Build**

Run:
```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
```
Expected: compiles clean. (Tasks 3–4 still reference the removed `prop_fx_intensity` — they are fixed in those tasks. If building before them, expect `prop_ui.c`/`prop_api.c` errors at link/compile referencing `prop_fx_intensity`; that is why Tasks 3 and 4 follow immediately.)

> NOTE: To keep each task independently buildable, do Steps in Task 3 and Task 4 before flashing. If you need a clean build at THIS task boundary, temporarily leave `prop_fx_set_intensity`/`prop_fx_intensity` as thin shims (`set` → `prop_fx_set_scanlines`; `get` → `prop_fx_scanlines`) and remove them in Task 4. Preferred path: proceed straight to Task 3.

- [ ] **Step 5: Commit**

```bash
git add firmware/communicator/main/include/prop_fx.h firmware/communicator/main/prop_fx.c
git commit -m "prop_fx: per-effect setters/getters + NVS load with legacy migration"
```

---

## Task 3: Display panel — four effect sliders (scrollable)

**Files:**
- Modify: `firmware/communicator/main/prop_ui.c` (`build_display_panel` ~line 635; its callbacks `fx_toggle_cb`/`fx_int_cb` ~line 624; the `s_fx_int_val` static).

**Interfaces:**
- Consumes: `prop_fx_set_scanlines/phosphor/vignette/refresh` + getters (Task 2); `prop_fx_enabled`/`prop_fx_set_enabled`; `make_slider`, `make_switch`, `panel_label`, `make_panel`.
- Produces: the real granular Display panel (consumed by the user/QA).

- [ ] **Step 1: Replace the FX callbacks**

Replace `fx_int_cb` (line ~628) and keep `fx_toggle_cb`. Add one value-label per effect. Replace the `s_fx_int_val` static declaration with four label statics near the other panel statics:

```c
static lv_obj_t *s_fx_scan_val, *s_fx_phos_val, *s_fx_vign_val, *s_fx_refr_val;
```

New callbacks (place where `fx_int_cb` was):

```c
static void fx_scan_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_scanlines(v);
    lv_label_set_text_fmt(s_fx_scan_val, "%d%%", v);
}
static void fx_phos_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_phosphor(v);
    lv_label_set_text_fmt(s_fx_phos_val, "%d%%", v);
}
static void fx_vign_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_vignette(v);
    lv_label_set_text_fmt(s_fx_vign_val, "%d%%", v);
}
static void fx_refr_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(lv_event_get_target(e));
    prop_fx_set_refresh(v);
    lv_label_set_text_fmt(s_fx_refr_val, "%d%%", v);
}
```

- [ ] **Step 2: Rewrite `build_display_panel` body (keep backlight + master CRT toggle, swap the single intensity slider for four)**

Replace from the `panel_label(p, "FX INTENSITY", ...)` block through the closing `return p;` with the four effect rows; make the panel scrollable so the rows always fit and Thread 3's transition controls can append later. Helper to cut repetition:

```c
/* One labelled FX slider row at vertical position y; returns the value label. */
static lv_obj_t *fx_row(lv_obj_t *p, const char *name, lv_coord_t y,
                        uint8_t val, lv_event_cb_t cb)
{
    panel_label(p, name, 40, y);
    lv_obj_t *v = lv_label_create(p);
    lv_label_set_text_fmt(v, "%u%%", val);
    lv_obj_set_style_text_color(v, COL_MUTE, 0);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, -40, y);
    make_slider(p, 0, 100, val, y + 34, cb);
    return v;
}
```

Body (the brightness + master toggle stay as-is above this):

```c
    lv_obj_set_scroll_dir(p, LV_DIR_VER);   /* rows can exceed panel height */

    panel_label(p, "CRT EFFECTS", 40, 196);
    make_switch(p, prop_fx_enabled(), SCAN_W - 140, 192, fx_toggle_cb, NULL);

    s_fx_scan_val = fx_row(p, "SCANLINES", 250, prop_fx_scanlines(), fx_scan_cb);
    s_fx_phos_val = fx_row(p, "PHOSPHOR",  330, prop_fx_phosphor(),  fx_phos_cb);
    s_fx_vign_val = fx_row(p, "VIGNETTE",  410, prop_fx_vignette(),  fx_vign_cb);
    s_fx_refr_val = fx_row(p, "REFRESH SWEEP", 490, prop_fx_refresh(), fx_refr_cb);
    return p;
```

Also NULL the four label pointers in `close_panel` alongside the other Display statics (search `close_panel` for where `s_disp_bright_val`/`s_fx_int_val` are cleared; replace `s_fx_int_val = NULL;` with the four new ones).

- [ ] **Step 3: Build**

Run:
```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
```
Expected: compiles clean.

- [ ] **Step 4: Flash and verify the panel renders (screenshot)**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" -p COM7 flash
python firmware/communicator/tools/prop.py shot display.png --screen display --wait
```
Expected: `display.png` shows BACKLIGHT, CRT EFFECTS toggle, and four labelled sliders (SCANLINES/PHOSPHOR/VIGNETTE/REFRESH SWEEP) with % values, themed amber-on-black. Open the PNG and confirm.

- [ ] **Step 5: Verify persistence + overlay on the panel (manual)**

Enable CRT, set SCANLINES high / VIGNETTE low, reboot the board, reopen Display: sliders restore their values (NVS). On the physical panel confirm scanlines strengthen and vignette weakens independently (the overlay is NOT in the screenshot — eyeball the panel).

- [ ] **Step 6: Commit**

```bash
git add firmware/communicator/main/prop_ui.c
git commit -m "ui: granular per-effect CRT sliders in Display settings"
```

---

## Task 4: Scriptable per-effect `/cmd fx` (programmatic handle)

**Files:**
- Modify: `firmware/communicator/main/prop_api.c` (the `"fx"` command handler).

**Interfaces:**
- Consumes: `prop_fx_set_scanlines/phosphor/vignette/refresh`, `prop_fx_set_enabled` (Task 2).
- Produces: extended `{"cmd":"fx",...}` accepting `scan|phosphor|vignette|refresh` (0–100) and `on` (bool). This is the only programmatic way to drive the overlay (it never appears in `/screenshot`).

- [ ] **Step 1: Locate the current fx handler**

Run: `grep -n '"fx"' firmware/communicator/main/prop_api.c`
Read the handler; today it reads `on` (bool) and `value` (0–100 → `prop_fx_set_intensity`).

- [ ] **Step 2: Replace the `value`→intensity call with per-effect keys**

Keep the `on` handling (→ `prop_fx_set_enabled`). Replace the single `value` parse with optional per-effect fields (using whatever cJSON helper the surrounding code already uses — match the existing pattern in this file, e.g. `cJSON_GetObjectItem`):

```c
    cJSON *j;
    if ((j = cJSON_GetObjectItem(root, "scan"))      && cJSON_IsNumber(j)) prop_fx_set_scanlines((uint8_t)j->valueint);
    if ((j = cJSON_GetObjectItem(root, "phosphor"))  && cJSON_IsNumber(j)) prop_fx_set_phosphor((uint8_t)j->valueint);
    if ((j = cJSON_GetObjectItem(root, "vignette"))  && cJSON_IsNumber(j)) prop_fx_set_vignette((uint8_t)j->valueint);
    if ((j = cJSON_GetObjectItem(root, "refresh"))   && cJSON_IsNumber(j)) prop_fx_set_refresh((uint8_t)j->valueint);
```

(Match the actual root-object variable name and cJSON accessors already used in the handler — read the surrounding lines first.) Remove the now-undefined `prop_fx_set_intensity` call. If Task 2's optional shims were added, delete them from `prop_fx.c`/`.h` now so no caller references the old API.

- [ ] **Step 3: Build**

Run:
```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" build
```
Expected: compiles clean; no remaining references to `prop_fx_set_intensity`/`prop_fx_intensity` anywhere (`grep -rn prop_fx_intensity firmware/communicator/main` returns nothing).

- [ ] **Step 4: Flash and verify via /cmd**

```powershell
idf.py -C "f:\git\personal\CrowPanelProp\firmware\communicator" -p COM7 flash
Invoke-RestMethod -Uri "http://comm-unit-7.local/cmd" -Method Post -Body '{"cmd":"fx","on":true,"scan":90,"phosphor":10,"vignette":80,"refresh":40}'
python firmware/communicator/tools/prop.py shot display.png --screen display --wait
```
Expected: command returns OK; opening Display shows the four sliders at 90/10/80/40; the panel shows strong scanlines, faint phosphor. Reboot → values persist.

- [ ] **Step 5: Commit**

```bash
git add firmware/communicator/main/prop_api.c firmware/communicator/main/prop_fx.c firmware/communicator/main/include/prop_fx.h
git commit -m "api: per-effect fx control via /cmd; drop legacy global intensity"
```

---

## Self-Review

**Spec coverage (Thread 3 §"Granular CRT FX"):**
- Per-effect intensity, 0=off, behind master toggle → Tasks 1–3. ✓
- Effects = scanlines, phosphor, vignette, refresh sweep → Task 1/3. ✓
- Display becomes scrolling with the four sliders + master CRT → Task 3 (`lv_obj_set_scroll_dir`). ✓
- NVS key per effect + legacy `fx_intensity` seeds defaults → Task 2. ✓
- Setters mirror existing `prop_fx_set_intensity` pattern → Task 2. ✓
- *Out of scope here:* TRANSITION dropdown + Transitions toggle live in **Thread 3's plan** (they append to this scrollable panel). Display panel left scrollable to receive them.

**Placeholder scan:** none — every code step shows full code; the one "match the existing cJSON pattern" instruction in Task 4 Step 2 is bounded by "read the surrounding lines" because the exact cJSON accessor varies and must match the file (the rest of the handler is the authority).

**Type consistency:** `prop_fx_set_scanlines/phosphor/vignette/refresh(uint8_t)` and `prop_fx_scanlines/phosphor/vignette/refresh(void)→uint8_t` used identically across Tasks 2/3/4. NVS keys `fx_scan`/`fx_phosphor`/`fx_vignette`/`fx_refresh` consistent in Tasks 2/4. `scale_opa(uint8_t base, uint8_t pct)` defined Task 1, used Tasks 1–2.
