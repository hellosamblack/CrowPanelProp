# CrowPanel communicator firmware audit

Audit date: 2026-08-20

Repository revision: `051282a2` plus the pre-existing dirty working tree

Scope: every `.c`/`.h` under `main/` and `components/` (73 files, 48,791 lines),
`main/CMakeLists.txt`, and `tools/prop.py`

## Executive summary

The reported battery behavior is a confirmed high-severity decode defect, not evidence of a
disconnected or over-voltage battery. The STC8 returns an 11-byte structure spread across registers
`0x00..0x0A`; Elecrow's own driver reads each register with a separate one-byte transaction. The
current firmware reads only register `0x00` and treats that byte—the low byte of the 32-bit
`adc_voltage` field—as a complete 8-bit ADC sample. That explains both observed states: the low byte
can alias below the firmware's presence threshold while USB is attached, then alias to a value that
the fabricated conversion maps to 4.32 V / 100% when USB is removed.

The audit found one critical network trust-boundary failure, four additional high-severity findings,
eleven medium findings, and six low findings. The dominant systemic themes are:

- Network diagnostics grew into an unauthenticated administrative plane, including arbitrary OTA
  firmware installation and hardware-mutating commands.
- The normal sensor architecture is sound—background task, locked cache, cheap UI read—but several
  configuration paths bypass it and perform NVS, Wi-Fi/SDIO, or full-screen work synchronously in
  LVGL callbacks.
- Concurrency discipline varies by module. Some caches use mutexes or critical sections correctly;
  others use `volatile` as if it provided ownership, atomic snapshots, or event correlation.
- Several modules are documented as non-fatal, yet either abort during early initialization or
  report success after unchecked task/handler creation.

The exact working tree built successfully with ESP-IDF 6.0.1 for `esp32p4`; `communicator.bin` is
1,960,864 bytes (`0x1DEBA0`, approximately 47% of its OTA slot). It was **not flashed** during this
audit. The connected board reported firmware `17a353e0-dirty`, so live observations corroborate
behavior but do not prove that the exact audited tree runs correctly.

## Architecture and review method

`app_main` brings up the board, engine, UI, network, sensors, API, and optional links. The engine owns
scene state and publishes snapshots to observers. Most sensors own a FreeRTOS task and publish a
small cached snapshot; the LVGL observer reads those caches and updates one lazily-created panel at a
time. The C6 supplies Wi-Fi, BLE, CSI, and FTM over `esp_hosted`; LiDAR is an outbound WebSocket
client with PSRAM reassembly and frame buffers. Large LVGL allocations and image buffers correctly
favor PSRAM.

Review was performed in four passes: complete source inventory/read, six lens-specific sweeps
(locking, memory, protocols, recovery, performance, documented board constraints), adversarial
re-reading of each candidate, then cross-module/init-order review. Generated font bitmap payloads
were checked structurally rather than interpreting every literal glyph byte; their descriptors were
compiled and linked by the build. The vendored MPU-6500 driver was reviewed mechanically in full and
manually at every call path used by `prop_imu.c`.

Additional checks:

- `python3 -m py_compile tools/prop.py` passed.
- No active use of `lv_snapshot`, deprecated canvas drawing, `lv_color_t.full`, a true RGB565
  `swap_bytes`, or `%f` in `lv_label_set_text_fmt` was found.
- The current GPIO assignments were reconciled with `docs/gpio_registry.yml`; the touchscreen,
  C6 SDIO, audio, backlight, radar UARTs, and exposed AIO set do not introduce an undocumented
  simultaneous pin collision. GPIO53/54 remain intentionally claimed by the LD2450 and therefore
  cannot also serve the optional radio modules.
- No automated test suite or host protocol fuzz tests exist in the repository.

## Live-board evidence and performance baseline

The board was reachable on its STA address (`172.17.2.67`) during the live portion of the audit.
While USB-powered, `/telemetry` returned battery `online=true`, `valid=true`, `voltage_v=0`,
`level_pct=0`, and `state="no_battery"`: the STC8 ACKs and the bad state is produced after a
successful read, so this is not an I2C discovery or physical-connectivity failure. A later sample
in the same USB-connected session reported 2.661 V and `charging`, further demonstrating that the
derived reading is unstable nonsense rather than a real cell-voltage measurement.

The FPS HUD was temporarily enabled and then disabled. Settled displayed update cadence on the
running `17a353e0-dirty` image was approximately:

| Panel | displayed min/typical/max FPS | Interpretation |
|---|---:|---|
| Scanner | 20 / 20 / 20 | Matches the engine observer cadence |
| Spectrum | 18 / 20 / 19 | Near the intended live cadence |
| Signal environment | 5 / 7 / 6 | Data/invalidation limited |
| Contacts | 3 / 4 / 3 | Data/invalidation limited |
| Vitals | 3 / 3 / 3 | Data/invalidation limited |
| Home | 3 / 3 / 3 | Mostly static |
| RF band | 1 / 1 / 1 | Static after scan, not a throughput result |

The HUD counts completed renders, so static-panel values are not a maximum rendering benchmark.
No destructive sensor commands, PPA stress command, OTA, erase, or flash operation was run.

## Critical

### C-01 — The network API permits unauthenticated hardware control and arbitrary firmware installation

**Defect:** A hard-coded OTA token is shipped in the firmware and rendered into the public web page,
while `/cmd` and `/ws` accept mutating commands without any authentication; secure boot and flash
encryption are disabled (`main/include/prop_api.h:30-31`, `main/prop_api.c:282-495`,
`main/prop_api.c:764-821`, `main/prop_api.c:841-872`, `main/include/prop_net.h:22-23`,
`sdkconfig:1150`, `sdkconfig:5541`).

**Failure scenario:** Any client on the same LAN—or anyone joining the fallback AP using the
repository-known password—can read the token from `/`, install an attacker-controlled but
syntactically valid ESP image, and reboot into it. Even without OTA, the same unauthenticated plane
can overwrite saved Wi-Fi credentials, reconfigure exposed I/O, redirect LiDAR, issue LD2450
factory-reset/baud/restart commands, or run the synchronous PPA stress path
(`main/prop_api.c:367-449`, `main/prop_api.c:322`, `main/prop_ppa_spike.c:189-422`). The WebSocket
handler does not validate `Origin`; simple cross-origin HTTP requests can also trigger side effects
because the command handler neither authenticates nor requires a preflight-only content type.

**Suggested fix:** Treat the API as one administrative trust boundary. Require a unique per-device
secret in an `Authorization` header for **all** mutating HTTP/WS operations, never serve that secret,
reject untrusted WebSocket origins, disable factory/baud/PPA diagnostics in production builds, and
rate-limit expensive operations. Use HTTPS or an explicitly isolated trusted control network. For
OTA, enable ESP-IDF signed-app verification and secure boot before treating the unit as deployable;
flash encryption is a separate decision if local secret extraction matters.

## High

### H-01 — Battery voltage and presence are decoded from one byte of an 11-register structure

**Defect:** `battery_task` reads only register `0x00` and rescales it as an 8-bit ADC count even
though it is the low byte of the STC8's 32-bit millivolt field (`main/prop_battery.c:145-175`); the
header simultaneously—but still incorrectly—describes a single sequential 11-byte read
(`main/include/prop_battery.h:10-18`).

**Failure scenario:** The low byte of a legitimate millivolt value changes non-monotonically as the
real value changes. With USB attached it can map below 2.5 V and report `NOT PRESENT`; after USB is
removed a different low byte can map through the invented divider formula to 4.32 V and clamp the
SOC curve to 100%, exactly matching the observed behavior. A failed later read also leaves the last
sample marked online and valid indefinitely (`main/prop_battery.c:146-149`,
`main/prop_battery.c:206-218`).

**Evidence:** Elecrow's archived factory implementation loops over `sizeof(Battery_info_t)` and
performs an individually addressed one-byte read of `STC8_REG_ADDR_BATTERY + i`; the packed fields
are `u32 adc_voltage`, `u32 bat_voltage`, `u8 bat_level`, `u8 bat_state`, `u8 led_state`
(`docs/referenceDesign/factory_sourcecode/V1.0/ESP32-P4-Adcance-brookesia_phone_inch7.zip`, inner
`components/espressif__esp32_p4_function_ev_board/bsp_stc8h1kxx.c:62-75` and
`bsp_stc8h1kxx.h:44-59`). The prior experiment that saw `0xFF` filler used one auto-incrementing
transaction and therefore did not reproduce the factory protocol.

**Suggested fix:** Read registers `0x00..0x0A` with eleven separately addressed one-byte
transactions, assemble both `u32` values little-endian, and range-check voltage, percentage,
state, and LED enum before publication. Mark the cache stale/offline after a bounded run of I2C
failures. Temporarily expose all eleven raw bytes in logs or telemetry, then validate with four
hardware cases: USB+battery, battery-only, USB-only, and neither. Update `prop_battery.h`,
`prop_battery.c`, `AGENTS.md`, and `CLAUDE.md` together; their current raw-ADC conclusion is refuted
by the factory access pattern and live symptom.

### H-02 — Concurrent Wi-Fi scans can clear another scan's crash-prevention guard

**Defect:** All scan callers share an unlocked boolean `s_scanning`, and each invocation sets and
clears it without owning a mutex or scan token (`main/prop_net.c:46`,
`main/prop_net.c:339-373`).

**Failure scenario:** The Wi-Fi UI, signal scan, RF-band task, or FTM task starts scan A; scan B
enters concurrently, receives `ESP_ERR_WIFI_STATE`/busy from `esp_wifi_scan_start`, and clears
`s_scanning` while A is still blocking. The RSSI task then calls `esp_wifi_sta_get_ap_info` during A;
the source comment records that this exact mid-scan RPC can make the esp-hosted port `memcpy` from a
null payload and assert (`main/prop_net.c:347-350`, `main/prop_net.c:577-578`, callers at
`main/prop_ui.c:774`, `main/prop_ui.c:1377`, and `main/prop_ftm.c:212`). Concurrent UI scan tasks can
also overwrite their shared `s_aps` array.

**Suggested fix:** Create one Wi-Fi scan service or mutex that owns the complete start/fetch/clear
lifecycle and returns caller-local results. A caller that does not acquire ownership must return
busy without changing the owner's state. Have RSSI polling consult the same synchronized state.

### H-03 — LVGL event callbacks synchronously perform NVS and C6/Wi-Fi operations

**Defect:** UI callbacks call `prop_net_forget` and `prop_net_set_sta_credentials` directly
(`main/prop_ui.c:860-867`, `main/prop_ui.c:882-912`), which commit NVS and issue C6-backed Wi-Fi
disconnect/config/connect/AP calls (`main/prop_net.c:486-523`); CSI setting callbacks similarly
commit NVS and send an esp-hosted RPC (`main/prop_ui.c:2203-2210`,
`main/prop_coproc.c:214-233`).

**Failure scenario:** These callbacks execute in the LVGL task while its port lock is held. A slow
flash commit or SDIO round trip stalls rendering and touch/dial processing; a wedged C6 can make the
screen appear frozen. This violates the repository's explicit rule that Wi-Fi/SDIO calls must not
run under the LVGL lock.

**Suggested fix:** Copy widget input into an owned request, queue it to a network/settings worker,
return immediately, and publish completion/error to a cache or LVGL async callback. Keep only cheap
widget changes in the event callback.

### H-04 — A pending OTA image can hang before rollback is armed again

**Defect:** Core hardware initialization failures enter an infinite delay loop
(`main/main.c:40-45`), but the image is not marked valid until all bring-up completes
(`main/main.c:260-264`).

**Failure scenario:** An OTA image that breaks I2C, display, touch, LDO, or another fatal precondition
boots in `PENDING_VERIFY` and then remains in `fail_loop` forever. Because it never reboots, the
bootloader gets no opportunity to roll back to the previous slot. The return from
`esp_ota_mark_app_valid_cancel_rollback` is also ignored, yet the firmware logs READY regardless.

**Suggested fix:** Detect the pending-verify state at boot and use a bounded health deadline. On a
fatal initialization failure call `esp_ota_mark_app_invalid_rollback_and_reboot`; if rollback is
unavailable, restart under a bounded retry/watchdog policy. Check and log the mark-valid return before
declaring READY.

## Medium

### M-01 — LiDAR's canvas buffer can be overwritten before LVGL renders it

**Defect:** The UI points the canvas directly at a triple-buffer slot and immediately returns a
server credit, although actual draw/flush happens later (`main/prop_ui.c:6249-6278`); the receiver
selects only `(front + 1) % 3` and can rotate back to that still-referenced slot
(`main/prop_lidar.c:516-529`, `main/prop_lidar.c:591-633`) while advertising six outstanding credits
(`main/prop_lidar.c:71-81`, `main/prop_lidar.c:953-960`).

**Failure scenario:** Three or more frames arrive between UI observation and completion of the
deferred LVGL render. The socket task overwrites the buffer that the canvas still references,
producing a torn frame or pixels combined from different LiDAR frames. Triple buffering protects
only a reader of the instantaneous `front`, not a retained zero-copy pointer.

**Suggested fix:** Pin/refcount the displayed slot until `LV_EVENT_RENDER_READY` (or the relevant
display completion callback), and return credit only when that slot is releasable. Alternatives are
an explicit free-buffer queue or one bounded copy into a UI-owned canvas buffer. Make the negotiated
credit window no larger than the number of independently writable slots unless lifetime tracking
proves it safe.

### M-02 — FTM result correlation has a lost-wakeup window and a torn shared structure

**Defect:** The FTM RX callback copies a multiword result and increments a `volatile` sequence with
no lock (`main/prop_coproc.c:111-124`), while the requester sends first and only then snapshots the
sequence counter inside `prop_coproc_ftm_wait_result` (`main/prop_ftm.c:120-136`,
`main/prop_coproc.c:168-193`).

**Failure scenario:** A fast C6 response arrives between the send and the wait function's
`seen_seq` load. That valid result becomes the baseline and is never consumed, so the probe waits the
full timeout and reports a false timeout. If the callback runs while the waiter copies the struct,
fields can come from different responses.

**Suggested fix:** Snapshot the generation before sending (or make send+wait one transaction),
publish the result under a critical section/mutex, and signal a semaphore or task notification.
Correlate on `req_id` while holding the same synchronization primitive.

### M-03 — FTM close, retry, and stale-table behavior can keep the C6 busy for over a minute

**Defect:** The worker snapshots up to eight due APs and probes all of them without rechecking panel
activity (`main/prop_ftm.c:224-240`); each can block for 11 seconds. Empty/error scans skip stale
pruning (`main/prop_ftm.c:211-222`), the first failure is scheduled at twice the documented base
backoff (`main/prop_ftm.c:57-63`, `main/prop_ftm.c:162-165`), and the deadline comparison is not
tick-wrap safe (`main/prop_ftm.c:230`).

**Failure scenario:** The operator leaves RANGE just after a due snapshot; FTM traffic can continue
for roughly 88 seconds plus the cycle delay, contending with normal Wi-Fi. A run of empty scans keeps
obsolete APs visible and eligible. Around the 32-bit millisecond wrap, direct `now >= deadline`
comparisons can delay or prematurely trigger probes.

**Suggested fix:** Recheck `ftm_should_scan()` between probes and use a cancellation-aware wait;
always prune based on elapsed age, distinguish scan failure from an authoritative empty result,
define whether failure 1 means base or doubled delay, and use signed-delta wrap-safe deadline tests.

### M-04 — Engine observers can receive old state after new state

**Defect:** `publish_locked` releases and reacquires the engine mutex around each observer while
retaining the original snapshot (`main/prop_engine.c:232-250`).

**Failure scenario:** Publisher A releases the lock for its first callback; publisher B acquires it,
publishes newer snapshot B to all observers, then A resumes and sends older snapshot A to its
remaining observers. UI/API clients can regress a scene, status, tick, or link field, and observers
do not all see the same ordered stream.

**Suggested fix:** Copy the observer list and state under the mutex, then enqueue a revision-numbered
snapshot to one serialized notification task. At minimum, serialize all notification loops with a
separate mutex and discard revisions older than the last delivered one.

### M-05 — Slider drag events rebake a full-screen canvas and commit flash on every value

**Defect:** Shared slider construction registers application callbacks on
`LV_EVENT_VALUE_CHANGED` (`main/prop_kit.c:190-207`); display/audio/AIO handlers persist each change
(`main/prop_ui.c:1031-1069`, `main/prop_ui.c:1150-1162`, `main/prop_ui.c:3350-3364`), and every CRT
intensity step repaints 1024×600 ARGB pixels under the LVGL lock before committing NVS
(`main/prop_fx.c:132-179`, `main/prop_fx.c:345-390`). AIO duty updates also commit NVS every step
(`components/bsp_aio/bsp_aio.c:451-460`).

**Failure scenario:** A finger drag generates many events per second, repeatedly consuming the
LVGL task, PSRAM bandwidth, and NVS erase/write budget. Touch feedback becomes uneven and flash wear
scales with slider motion. Reading `fx_trans` from NVS on every navigation transition adds another
synchronous flash-backed lookup (`main/prop_fx.c:456-460`).

**Suggested fix:** Keep live preview values in RAM, rate-limit expensive repainting to at most one
pending update per frame, and persist only on `LV_EVENT_RELEASED` or after a debounce/quiescence
timer. Cache all settings after initialization; use a deferred settings writer.

### M-06 — Selecting an ADC spectrum source permanently reconfigures an AIO pin

**Defect:** The microphone task silently changes a selected GPIO to `AIO_ANALOG_IN`
(`main/prop_mic.c:129-144`), and `bsp_aio_set_mode` releases its previous PWM/interrupt ownership and
persists the new mode (`components/bsp_aio/bsp_aio.c:308-327`).

**Failure scenario:** An operator merely cycles the Spectrum source through a pin currently driving
external equipment. The background task tears down that output and saves analog-input mode, so the
external behavior changes immediately and remains changed after reboot.

**Suggested fix:** Add explicit pin ownership/arbitration. Refuse a busy source or require a
confirmed temporary claim, never persist that temporary mode, and restore the complete previous
mode/options when the source changes away.

### M-07 — MR24HPC1 presence ignores a documented equivalent response command

**Defect:** The parser updates presence only for `ctrl=0x80, cmd=0x01`
(`main/prop_aux_radar.c:197-205`), while the verified protocol permits the human-status response as
either `0x01` or `0x81` (`.agents/skills/sensor-datasheets/references/mr24hpc1.md:91-108`).

**Failure scenario:** A sensor firmware responds with `cmd=0x81`. The generic valid-frame path keeps
refreshing the online watchdog, so the UI says the sensor is online, but presence remains frozen at
its old value.

**Suggested fix:** Match `ctrl == 0x80 && (cmd & 0x7f) == 0x01`; add captured `0x01` and `0x81`
frames to a host parser test with checksum, truncation, and resynchronization cases.

### M-08 — LD2450 configuration error paths can strand the sensor in config mode

**Defect:** After Enable Config succeeds, several restart paths return on a missing command ACK
without attempting End Config (`main/prop_motion.c:419-430`, `main/prop_motion.c:488-495`), and the
ACK parser reads the echoed word before proving `alen >= 2` (`main/prop_motion.c:153-175`). The config
mutex and completion semaphore are also used without checking allocation success
(`main/prop_motion.c:536-542`).

**Failure scenario:** A dropped restart ACK leaves the LD2450 in configuration mode, stopping target
streaming until power-cycle or another successful recovery command. A malformed but tail-valid frame
with `alen` 0 or 1 causes length underflow/out-of-frame interpretation; low memory can turn the first
config exchange into a null semaphore call.

**Suggested fix:** Use one structured config-session cleanup path that best-effort sends End Config
on every failure where the module did not actually reboot. Require the minimum ACK payload before
reading it, check both synchronization allocations, and release all created resources on init
failure.

### M-09 — MPU-6500 self-test can divide by zero during boot

**Defect:** The vendored self-test computes `pack_cnt = fifo_count / 12` and divides six accumulated
offsets by `pack_cnt` without handling zero (`components/mpu6500/driver_mpu6500.c:900-944`);
`prop_imu` invokes this function on every successful chip initialization (`main/prop_imu.c:100-109`).

**Failure scenario:** A short/empty FIFO read that is otherwise a successful I2C transaction yields
fewer than 12 bytes. `pack_cnt` becomes zero and the integer division panics during boot rather than
taking the intended non-fatal “self-test failed, skip bias” path.

**Suggested fix:** Return self-test failure when `pack_cnt == 0` before accumulation/division, and
validate that the FIFO count is a plausible multiple/range before reading packets. Keep the caller's
existing non-fatal fallback.

### M-10 — “Optional” networking can still abort the entire prop

**Defect:** `prop_net_init` uses `ESP_ERROR_CHECK` for netif/event-loop setup and handler
registration (`main/prop_net.c:195-204`) even though its documented contract from the subsequent C6
initialization onward is to log and continue without Wi-Fi (`main/prop_net.c:206-217`); it also
dereferences the STA netif without checking creation (`main/prop_net.c:224-225`).

**Failure scenario:** An allocation failure, duplicate event-loop state, or handler-registration
failure aborts/reboots the whole communicator before the advertised degraded local-only UI can run.
A null STA netif can be passed to the hostname setter.

**Suggested fix:** Replace fatal macros with checked returns and one cleanup/degraded-mode path;
null-check netif creation. Make the optional/fatal boundary explicit and consistent in `app_main`.

### M-11 — LiDAR can become permanently stuck after a synchronous WebSocket start failure

**Defect:** The LiDAR task ignores both event-registration and client-start return values
(`main/prop_lidar.c:1053-1070`) and then enters the session event loop expecting a disconnect/error
event.

**Failure scenario:** `esp_websocket_client_start` fails synchronously before creating its worker or
emitting an event. The bounded event waits repeat forever on the same inert handle, so discovery and
reconnection backoff never restart and the panel remains SEARCHING until reboot.

**Suggested fix:** Check registration/start results, destroy the partial client, set the cached link
state, and return to the existing bounded reconnect backoff. Recheck `s_active` after blocking mDNS
resolution before opening a connection.

## Low

### L-01 — IMU failures leave stale data online and a sustained free-fall repeatedly queues alerts

**Defect:** DMP read failures simply continue without aging or invalidating the cache
(`main/prop_imu.c:249-257`), successful samples preserve the prior `online` bit
(`main/prop_imu.c:312-329`), and every sample after 80 ms of low acceleration republishes a free-fall
event rather than only the threshold crossing.

**Failure scenario:** A disconnected or wedged IMU can show indefinitely fresh-looking orientation.
During a sustained low-g interval the engine consumes repeated free-fall events and queues repeated
alert tones/scenes instead of one episode notification.

**Suggested fix:** Timestamp samples and publish stale/offline after a bounded failure run; edge-
trigger free-fall with re-arm hysteresis. Check the IMU task-creation result at
`main/prop_imu.c:340-353`.

### L-02 — Small cross-task state variables still have C data races

**Defect:** `track_heading_rad` reads `s_dir_phi` outside the mutex used by its writer
(`main/prop_track.c:56-63`, writer at `main/prop_track.c:255-260`), while calibration reads/writes
`s_auto`, `s_threshold`, and `s_reset_req` under inconsistent synchronization
(`main/prop_calib.c:33-42`, `main/prop_calib.c:70-80`, `main/prop_calib.c:108-136`,
`main/prop_calib.c:151-165`).

**Failure scenario:** Concurrent calibration or heading changes create undefined behavior under the
C memory model and can publish a mixed/stale setting. A 32-bit aligned float is likely atomic on this
target, so the practical impact is transient wrong UI/heading rather than memory corruption.

**Suggested fix:** Put every field behind its existing lock/critical section or use ESP-IDF atomic
operations with an explicit publication protocol; `volatile` alone is not synchronization.

### L-03 — The Spectrum UI advertises GPIO53/54 sources that can never produce data

**Defect:** The public enum and UI list six ADC sources through IO54
(`main/include/prop_mic.h:20-28`, `main/prop_ui.c:1515`), but `bsp_aio` deliberately exposes only
IO49..52 because IO53/54 are claimed by the LD2450 (`components/bsp_aio/bsp_aio.c:15-30`,
`main/prop_motion.c:17-19`); the microphone task therefore renders those two choices as silence
(`main/prop_mic.c:129-140`).

**Failure scenario:** Selecting IO53 or IO54 presents a valid-looking source but always clears the
spectrum, leading an operator to diagnose wiring or ADC failure even though the choice is impossible
in this firmware configuration.

**Suggested fix:** Remove/disable those two UI options and correct the public comments, or introduce
an explicit mutually exclusive radar/AIO ownership mode before advertising them.

### L-04 — Merely opening the cue-board web page mutates the scene twice

**Defect:** The page's WebSocket `onopen` sends `next_scene` and then `scene=IDLE`
(`main/prop_api.c:847-864`).

**Failure scenario:** A status-only browser visit triggers scene/audio/LED side effects before
forcing IDLE, interrupting an active on-set cue and creating unnecessary observer broadcasts.

**Suggested fix:** Make connection read-only; the server already pushes state. Send mutations only
from deliberate controls.

### L-05 — CLI options without a value crash with `IndexError`

**Defect:** `_pop_opt` blindly reads `args[i + 1]` (`tools/prop.py:397-403`).

**Failure scenario:** Commands such as `prop.py shot out.png --host` terminate with a Python
traceback instead of a usage error.

**Suggested fix:** Bounds-check the following token and raise the CLI's normal concise argument
error; longer term, use `argparse` subcommands.

### L-06 — Partial initialization frequently leaks resources or reports unavailable work as ready

**Defect:** Several init paths do not unwind prior allocations/peripherals and several ignore worker
or registration creation results: audio (`main/prop_audio.c:173-190`), microphone
(`main/prop_mic.c:182-245`), tracking (`main/prop_track.c:179-203`), BLE
(`main/prop_ble.c:345-372`), battery (`main/prop_battery.c:241-245`), CSI
(`main/prop_csi.c:153-170`), and HTTP API (`main/prop_api.c:938-973`).

**Failure scenario:** Under low-memory/resource pressure a module can leave I2S/NimBLE/kernel/PSRAM
resources allocated for the boot lifetime, or log itself available even though its maintenance,
telemetry, or handler task never started. Since most initializers run once, this usually produces
degraded features rather than an accumulating steady-state leak.

**Suggested fix:** Give each initializer one reverse-order cleanup label, check every task/handler
registration, and set `available` only after all required workers exist. Return a degraded capability
mask where optional sub-workers are genuinely optional.

## Unverified / requires targeted hardware or peer-end testing

These items survived initial review but lack enough evidence for a ranked defect claim.

1. **LiDAR `thin_ready.seq` semantics:** The frame header's server sequence is explicitly ignored
   (`main/prop_lidar.c:616-618`), while `pump_ready` sends the local consumption target for every
   returned credit (`main/prop_lidar.c:971-1008`). One design passage says `thin_ready.seq` echoes the
   server sequence, while a later passage describes local accounting. Capture client/server JSON and
   confirm the roomscanner treats `seq` as diagnostic rather than as the identity of the returned
   frame before changing this code.
2. **Live validation of the corrected STC8 protocol:** The factory driver's access pattern is
   decisive enough to identify H-01, but the exact audited tree was not flashed and registers
   `0x00..0x0A` were not dumped over serial in this session. Log each independently addressed byte
   in the four power combinations listed in H-01 before deleting the current fallback/calibration
   code.
3. **Task stack margins:** Static stack sizes are plausible and the configured main stack remains
   8192, but no `uxTaskGetStackHighWaterMark` data was available for LiDAR JPEG decode, HTTP telemetry,
   BLE host, or IMU DMP worst cases. Instrument high-water marks during simultaneous Wi-Fi, LiDAR,
   BLE, CSI, audio, and panel activity.
4. **Network recovery under a missing/mismatched C6:** Live testing had a functioning radio. Test a
   deliberately absent/incompatible esp-hosted slave to verify every “non-fatal” path after M-10 and
   the initialization cleanup work.
5. **Parser fuzz behavior:** Length checks in the LiDAR, LD2450 target stream, and both auxiliary
   radar parsers were otherwise defensible, but no host fuzz harness exists. Extract pure parser
   functions and feed truncated headers, oversized lengths, overlapping fragments, bad tails,
   checksum failures, and resynchronization noise under ASan/UBSan on a host build.

## Recommended order of work and measurement plan

1. Fix and hardware-validate H-01 first; it directly explains the reported status screen.
2. Close C-01 before placing the prop on any shared or production network.
3. Serialize Wi-Fi scans (H-02), move C6/NVS work out of LVGL callbacks (H-03), and make pending OTA
   failures roll back (H-04).
4. Fix the LiDAR buffer lifetime (M-01) before increasing credits/FPS further; then repair FTM result
   synchronization and cancellation (M-02/M-03).
5. Debounce/cachify settings work (M-05), then rerun the same panel cadence checks plus input latency.

For performance changes, measure rather than infer from the static-panel FPS number:

- Add microsecond counters around `ui_observer`, `paint_canvas`, render-ready, flush completion,
  Wi-Fi configuration jobs, and LiDAR decode/publish.
- Report p50/p95/p99 observer time and render-to-render interval, not only average FPS.
- Record LVGL task high-water mark, internal/PSRAM free and largest block, C6 RSSI/scan/FTM activity,
  LiDAR `tx_fps`, `tx_bytes_per_s`, and dropped frames.
- Use repeatable scenes: Scanner idle and three targets, Spectrum MIC, active CSI, BLE scan, FX slider
  drag, Wi-Fi connect, LiDAR stream plus orbit input, and simultaneous audio.
- Compare with the FX overlay disabled and enabled, then with settings writes deferred. Preserve the
  existing PPA patches and `swap_bytes=false`; neither is implicated by this audit.
