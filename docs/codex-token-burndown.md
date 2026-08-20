# Codex token burndown — exhaustive firmware audit

Purpose: spend a large Codex token budget on something durably useful. This file holds the
prompt to paste into Codex, plus the model/effort settings to run it with.

## Model & effort

- **Model:** Sol (the top Codex model; fall back to the highest `codex-max` tier if Sol is
  unavailable in your plan).
- **Reasoning effort:** **Extra High (`xhigh`)** — or **Ultra** if the account exposes it.
  This task is explicitly a burndown: prefer the deepest setting available over speed or cost.
- Run from the repo root (`~/git/personal/CrowPanelProp`) with full read access. Write access
  only as described in the prompt (report file + optional fix branch).

## The prompt

Paste everything below the line into Codex verbatim.

---

You are auditing the firmware for a cassette-futurism communicator prop built on the CrowPanel
Advance ESP32-P4 7-inch HMI board (ESP32-P4 + ESP32-C6 radio co-processor over esp_hosted,
LVGL 9.4, ESP-IDF 6.0.1). The repo root is the ESP-IDF project; read `CLAUDE.md` first — it
documents hard-won board constraints that your findings must not contradict (LVGL lock rules,
PSRAM allocator, main-task stack size, PPA patches, RGB565 swap_bytes, sign-magnitude LD2450
decoding, STC8 battery protocol, and more).

This is a maximum-depth audit, not a quick pass. There are no automated tests in this repo, so
your review is the test suite. Budget is not a constraint — be exhaustive, then adversarially
re-verify your own findings before reporting them.

**Ground rules**

- Treat the current working tree (including uncommitted changes) as the source of truth. Do
  not revert, stash, or clean anything.
- Do not modify `sdkconfig`, `sdkconfig.defaults`, or anything matching `CONFIG_ESP32P4_*REV*`.
- Do not touch `managed_components/` (local LVGL PPA patches live there and are gitignored).
- The default deliverable is a report, not code changes (see Deliverables).

**Scope — every `.c`/`.h` under `main/` and `components/` (bsp_* and mpu6500), plus
`main/CMakeLists.txt` and `tools/prop.py`. Audit each file against all of these lenses:**

1. **Concurrency & locking.** Every LVGL call from a non-LVGL task must hold
   `lvgl_port_lock()`; nothing may run the draw pipeline (`lv_snapshot`, canvas layer-draw)
   under that lock from a non-LVGL task; no WiFi/SDIO call may happen under the LVGL lock.
   Check every mutex/queue/cache pattern (`prop_ble`, `prop_track`, `prop_lidar` triple
   buffer, `prop_api` telemetry task, `prop_audio` queue) for races, missed unlocks on error
   paths, use-after-free of torn-down panels (`close_panel` NULLing rules), and ISR-context
   violations.
2. **Memory.** PSRAM vs internal allocation correctness, cache-line/DMA alignment on the
   P4 (framebuffer reads need cache invalidation), leaks on error paths and panel teardown,
   stack sizing of every created task, unbounded ring buffers.
3. **Protocol decoders & parsers.** LD2450 frame parsing (sign-magnitude!), MR24HPC1 and
   SEN0395 UART handling, STC8 I2C reads, LIDAR `THIN_FRAME` binary + `thin_telemetry` JSON,
   FTM RPC framing, WebSocket/HTTP handlers in `prop_api.c`. Hunt for out-of-bounds reads,
   truncated-frame handling, integer overflow/sign bugs, and missing validation of
   network-supplied lengths/offsets.
4. **Error handling & recovery.** Every `esp_*` return value that is ignored; retry/backoff
   loops that can spin hot; OTA safety (nothing blocking before `esp_ota_mark_app_valid`);
   reconnect paths in `prop_net`, `prop_lidar`, `prop_ble` after link loss.
5. **Performance.** Per-frame work in `ui_observer` and canvas paths, redundant redraws,
   busy-wait loops, JSON built when no client is listening, anything that would drag LVGL
   frame rate or add jank on a 400 MHz RISC-V with a 1024×600 panel.
6. **Consistency with the documented gotchas.** Any code that contradicts a rule stated in
   `CLAUDE.md` or the module map (e.g. `%f` in `lv_label_set_text_fmt`, `swap_bytes`,
   COL_DIM used for must-read text) is a finding even if it "works".

**Method**

- Pass 1: read `CLAUDE.md`, then read every in-scope file fully and take structured notes.
- Pass 2: per-lens sweeps across all files (all six lenses, every file — no sampling).
- Pass 3: adversarial verification. For each candidate finding, actively try to refute it by
  re-reading the code and the board constraints; discard anything you cannot defend with a
  concrete failure scenario (inputs/state → wrong behavior). Plausible-but-unconfirmed items
  go in a separate "unverified" section, clearly labeled.
- Pass 4: cross-cutting review — module interactions, init ordering in `app_main`, task
  priorities, and whether the recent uncommitted LIDAR/FTM changes introduce regressions.

**Deliverables**

1. Write `docs/codex-audit-report.md`: prioritized findings (Critical / High / Medium / Low),
   each with `file:line`, a one-sentence defect statement, the concrete failure scenario, and
   a suggested fix. Include an "unverified" section and a short architecture-level summary of
   systemic patterns you noticed.
2. Optionally, on a new branch `codex/audit-fixes` (never on `master`), apply only fixes that
   are mechanical and provably safe (ignored return values, missing NULL guards, off-by-ones).
   One commit per finding, referencing the report entry. Anything requiring judgment or
   hardware testing stays report-only.
3. Do not claim the code builds or runs — you cannot flash the board. State clearly which
   findings are static-analysis-only.

Keep going until all four passes are complete for every in-scope file. If you run low on
context, checkpoint your notes into the report file and continue from there.
