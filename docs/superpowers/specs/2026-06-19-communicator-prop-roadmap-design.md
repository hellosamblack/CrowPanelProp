# Communicator Prop — Feature Roadmap

**Date:** 2026-06-19
**Status:** Approved design / roadmap — **scanner-instrument phases below largely built.**
**See also:** `2026-06-19-communicator-archive-addendum.md`, which reframes the *top level*
of the device around the author's brief (the data **archive** is now primary; this scanner
is one instrument on the console rail).
**Scope:** `` (ESP-IDF 6.0.1, ESP32-P4, LVGL 8.4)

## Context

The communicator firmware turns the CrowPanel Advance into a cassette-futurism
scanner/communicator prop. Today it is a working *skeleton*: a `prop_engine` scene
state machine (IDLE / SCANNING / SIGNAL_ACQUIRED / COMMS / ALERT) driving a sparse
main readout at 10 Hz, a SETUP menu where only **WI-FI** is real (DISPLAY / AUDIO /
LEDS / ABOUT are stubs), WiFi AP+STA networking, and an HTTP/WebSocket remote-control
API with `/screenshot`. A lot of onboard hardware (I2S speaker amp, the spec'd dual
microphones, SX1262 LoRa, nRF24, SD card) is present but unused by the firmware.

This roadmap sequences the work to turn that skeleton into a finished prop. The guiding
intent, decided during brainstorming:

- **Aesthetic-first** (a convincing *fiction* of a working scanner) with **real data
  sprinkled in for flavor** where it adds authenticity.
- **The main readout is the hero.** It must stay **visually striking and uncluttered**
  so it reads well **on camera** — drama comes from *motion and choreography*, not from
  packing the screen with widgets.
- The **microphone** is a wanted real input (spectrum analyzer + dB meter), but the mic
  signal path is **undocumented and unverified** — it must be de-risked before we build on it.
- **Audio output (speaker SFX) is low priority**; **LoRa is parked** until a module is present.
- Everything must hold the **cassette-futurism vibe** (amber-on-black, square corners,
  the existing style kit in `prop_ui.c`).

### Three-bucket mental model

The design keeps three concerns cleanly separated so the hero screen never gets crowded:

- **Scenes** *choreograph* — the engine state machine drives what the main readout performs.
- **Instruments** *display real data* — spectrum, vitals, signal-scan; reachable as their
  own screens, kept off the main page.
- **Settings** *configure* — the real SETUP panels (brightness, etc.).

## Architectural notes (apply across phases)

- **No new architecture for the main page.** All Phase 1 dynamics ride the existing
  `prop_engine` 10 Hz tick + `animate_scene()` (single source of truth) fanned out to the
  pure-view `prop_ui` via the observer. The UI renders state; it never owns logic.
- **Screen effects are a global top-layer overlay** (LVGL top/sys layer), independent of
  individual screens, with a runtime toggle and tunable intensity.
- **LVGL is not a shader engine.** Overlay-based effects are cheap; true per-pixel
  blur/bloom at 1024×600 is expensive on the CPU and is treated as best-effort/faked, with
  the P4 PPA / 2D-DMA accelerator noted as a research stretch — never a blocker.
- **Memory discipline holds** (see `CLAUDE.md`): do not raise
  `LV_MEM`; big buffers/assets go to PSRAM. The mic capture buffer and any spectrum/FFT
  working set must be PSRAM-allocated.
- **Verify each phase with `tools/screenshot.py`** — drive the UI via `/cmd` and capture
  the screen as PNG to review the result directly rather than asking a human to eyeball it.

## Roadmap

### Phase 1 — Main readout dynamics (the hero; do first)

Make the existing main page *perform* without adding clutter. Camera-striking, minimal.

- **Waveform recorder trace.** Replace the single sweeping blip with a sweep that *draws a
  waveform trail behind it* (oscilloscope/seismograph style) inside the existing scanner track.
- **Scene-driven trace character:**
  - IDLE — near-flat baseline with faint ripple; slow drift.
  - SCANNING — steady sweep laying down low "noise grass"; occasional small spikes.
  - SIGNAL_ACQUIRED — a big spike erupts at the blip; sweep freezes/holds; peak-hold marker.
  - COMMS — trace becomes a modulated carrier (rhythmic "transmission in progress").
  - ALERT — whole trace jitters, red, fast.
- **Dramatic status text (the money shots).** Entering SIGNAL_ACQUIRED punches in a bold
  **"SIGNAL DETECTED"** (brief scale/flash, settle); ALERT flashes **"\*\* ALERT \*\*"** in red.
- **Frequency scramble-and-lock.** While SCANNING the `CH -- / --- MHz` readout scrambles
  (digits cycling like a hunting tuner); on acquisition it snaps to a locked value.
- **Idle life.** Even in IDLE, an occasional faint spike crosses the trace so the device
  never looks dead on camera.
- **Boot sweep.** Brief power-on diagnostic choreography (self-test, settle to IDLE) the
  first time the screen builds.

*Touches:* `prop_engine.c` (per-scene animation data in `animate_scene`/`scene_led_mask`
neighborhood, plus waveform sample generation), `prop_ui.c` (`build_screen` trace widget,
observer render path), `prop_engine.h` (extend the state pushed to the UI).

### Phase 2 — Mic discovery spike (gates the spectrum analyzer)

Research/spike, not a feature. Identify the microphone signal path on this board (chip rev
v1.3): I2S-RX peripheral + input codec (likely an ES7210-class ADC configured over the
existing I2C bus), pin assignments, and a minimal capture proof. Board spec claims "dual
microphones" but neither the root `readme.md` pinout nor any `example/`/`factory_sourcecode`
documents the input path — this must be established empirically. **Deliverable:** a confirmed
capture path + short notes (and an `idf6`/board memory update), OR a documented finding that
the mic is unreachable, in which case Phases 5/6 reprioritize and the spectrum feature is cut
or deferred.

### Phase 3 — Screen effects layer

A global, tunable, runtime-toggleable post overlay on the LVGL top layer, applied above
every screen. Camera-tuned (off by default if it harms legibility on capture).

- **Cheap / overlay-based (confident):** CRT scanlines (tiled semi-transparent overlay),
  rolling refresh line (soft gradient bar animated down-screen), grid overlay, vignette,
  phosphor amber tint/glow.
- **Best-effort / faked:** text blur / bloom — via LVGL shadow styles or a dim
  duplicated-text halo behind labels. True full-screen gaussian blur is out of CPU budget;
  offloading to the P4 PPA / 2D-DMA accelerator is a research stretch, not a requirement.

*Touches:* new effects module (e.g. `prop_fx.c` + overlay objects on `lv_layer_top()`),
hook into `prop_ui` init; expose a toggle via `/cmd` and (later) the DISPLAY settings panel.

### Phase 4 — Finish real settings

Make the SETUP stubs real (replace `build_stub` calls in `build_setup_screens`):

- **DISPLAY** — backlight brightness (bsp_illuminate already drives the backlight); also the
  effects-layer intensity/toggle from Phase 3.
- **LEDS** — brightness + test/identify.
- **ABOUT** — firmware version, build, IP, uptime.
- **AUDIO** — volume/mute control stub (wired now; consumed if/when speaker SFX is promoted).

Settings persist via `prop_settings` (NVS).

### Phase 5 — VITALS instrument

A diegetic instrument screen ("reactor/system vitals"): battery voltage, temperature,
uptime, free RAM. No new hardware — proves the instrument framework end-to-end with zero
hardware risk. Reachable as its own screen, kept off the hero main page.

### Phase 6 — SPECTRUM analyzer + dB meter (gated on Phase 2)

The marquee mic feature: live FFT bars + a dB level meter, themed amber-on-black with retro
ballistics (peak-hold, slow decay). FFT working set in PSRAM. Optionally feeds real audio
into the Phase 1 main-page waveform trace, making the recorder bar the bridge between the
aesthetic main page and real-data instruments.

### Phase 7 — SIGNAL SCAN (WiFi as "contacts")

WiFi scan instrument: nearby AP count + RSSI bars rendered as detected signal sources, tied
into the SCANNING scene fiction. Reuses the C6 radio (already used by `prop_net`); the
existing `rssi_to_bars()` helper in `prop_ui.c` is a starting point for the meter mapping.

### Phase 8 — Vibe polish & optional LoRa

Ongoing: richer scene choreography, SFX hooks if audio output gets promoted, and a LoRa
(SX1262) scanner *if/when* the plug-in module is present.

## Risks & open items

- **Mic path unknown (Phase 2)** — highest-uncertainty item; gates Phase 6 and the optional
  mic-driven main-page trace. Sequenced early so the rest of the roadmap can absorb the answer.
- **Text blur/bloom feasibility (Phase 3)** — may only ever be a faked approximation; PPA
  offload unproven. Not allowed to block the cheap effects.
- **Internal RAM pressure** — C6 esp_hosted + LWIP + LVGL already make internal RAM tight;
  spectrum/FFT and any new buffers must live in PSRAM.

## Verification

Per phase, build + flash, then drive and capture the UI to review directly:

```powershell
& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"; $env:PYTHONIOENCODING="utf-8"
idf.py -C "f:\git\personal\CrowPanelProp" build
idf.py -C "f:\git\personal\CrowPanelProp" -p COM7 flash
# drive a scene / screen, then screenshot it:
Invoke-RestMethod -Uri "http://<ip>/cmd" -Method Post -Body '{"cmd":"scene","value":"SIGNAL_ACQUIRED"}'
python tools/screenshot.py <ip> phase1_signal.png
```

- **Phase 1:** cycle every scene via `/cmd` and screenshot each; confirm trace character,
  "SIGNAL DETECTED"/ALERT punch-in, scramble-and-lock, idle life, boot sweep.
- **Phase 2:** capture mic samples; confirm non-trivial signal; document the path.
- **Phase 3:** toggle the effects layer on/off via `/cmd`; screenshot both; confirm legibility
  on camera-representative capture.
- **Phases 4–7:** screenshot each new/updated screen; confirm real data renders and persists
  (settings) and that instruments stay off the hero page.
