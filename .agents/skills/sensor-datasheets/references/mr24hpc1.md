# Seeed MR24HPC1 — 24 GHz Human Presence mmWave Sensor (reference)

Source: `MR24HPC1 Technology 24GHz_mmWave_Sensor.pdf` under
`docs/datasheets/externalDevices/MR24HPC1 Seeed Technology 24GHz_mmWave_Sensor/`, plus the
vendored `MR24HPC1_ESPHome_external_components-main/` community integration (real UART
protocol ground truth — the PDF itself has no byte-protocol section). Ground truth for what's
wired/working: `main/prop_aux_radar.c` (UART3, GPIO47/48, J2).

## The folder's README.md was wrong (duplicated into the wrong place, not wrong itself)

The old `README.md` here was **byte-for-byte identical** to the one that used to sit in the
`LD2450 HiLink...` folder. The spec tables in it (voltage/current/range/RF numbers) **are**
genuine MR24HPC1 data — they check out against this PDF. The bug was that the *exact same
file* got copy-pasted into the LD2450 folder too, where none of it applies (LD2450 is a
3-target UART tracking radar, not a GPIO presence sensor). See `references/ld2450.md`.

Human presence + motion state (GP1/GP2) is a real MR24HPC1 feature — this project just
doesn't use those GPIO pins; it talks to the sensor over UART instead (see below).

## 1. Identity & scope

Seeed's "24GHz mmWave Sensor – Human Static Presence Module Lite." FMCW human-presence radar:
presence/absence + motion state (+ underlying-mode body-motion amplitude/echo data). **Not a
multi-target tracker** — one binary presence/motion result per read, no per-target coordinates
(that's the LD2450's job).

## 2. Electrical specs

| Parameter | Min | Typ | Max | Unit |
|---|---|---|---|---|
| Operating voltage (VCC) | 4.5 | 5.0 | 6 | V |
| Operating current (ICC) | 95 | 110 | 120 | mA |
| I/O current | – | 8 | 20 | mA |
| Operating temp | -20 | – | +85 | °C |
| Storage temp | -40 | – | +85 | °C |
| Baud rate | – | 115200 | – | – |
| Motion detection range | – | 5 | 5 | m |
| Stationary/micro-movement range | – | 4 | 4 | m |
| Sleep detection range | 3 | 3.5 | – | m |
| Operating frequency | 24.0 | – | 24.25 | GHz |
| Tx power | – | 6 | 8 | dBm |
| Antenna gain | – | 10 | – | dBi |
| Horiz. beam (3dB) | – | 100° | – | – |
| Vert. beam (3dB) | – | 80° | – | – |

(The vendor PDF's own "Feature" bullet says "power <0.5W" while its spec table says "≤5W" —
an inconsistency in Seeed's own document, not introduced by this project.)

**Power-supply gotcha** (PDF FAQ): requires 5–6V with ripple **≤100 mV**; poor grounding/ripple
degrades range and increases false-alarm rate. Check the 5V rail first if this sensor
misbehaves, before suspecting firmware.

## 3. Pinout (Interface 1 — what this board actually wires, via J2)

| Pin | Name | Level | Direction | Notes |
|---|---|---|---|---|
| 1 | 5V | 5.0V | Input | main power |
| 2 | GND | – | – | |
| 3 | RX | 3.3V | Input | UART receive |
| 4 | TX | 3.3V | Output | UART transmit |
| 5 | GP2 | 3.3V/0V | Output | Presence/Absence (high = present) — **unused on this board** |
| 6 | GP1 | 3.3V/0V | Output | Active/Stationary — **unused on this board** |

Interface 2 (expansion: 3V3/GND/SL/SD/GP3–GP6) is unused here too. This project talks to the
sensor purely over UART, not via the GPIO presence pins.

## 4. Real UART protocol (ground truth: ESPHome `seeed_mr24hpc1_constants.h`/`.cpp`, cross-checked
against `prop_aux_radar.c`)

Binary frame, not ASCII:

```
[0]     0x53           header 1
[1]     0x59           header 2
[2]     ctrl           control word
[3]     cmd            command word
[4]     len_h          data length high byte (0x00)
[5]     len_l          data length low byte (0-32)
[6..]   data           payload (len_l bytes)
[N-3]   checksum       sum of bytes [0..N-4] & 0xFF  (plain byte-sum, NOT a real CRC)
[N-2]   0x54           tail 1
[N-1]   0x43           tail 2
```
Total length = 9 + len_l. Firmware comments call the checksum byte "CRC" but it's a modulo-256
byte sum — confirmed by hand-checking the heartbeat frame's checksum.

**Control words:** `0x01` main/heartbeat · `0x02` product info · `0x05` work status (scene
mode/sensitivity/custom mode) · `0x08` underlying-function data (raw echo/energy/distance/speed)
· `0x80` human information (presence, motion, body-movement, unmanned-time, keep-away).

| Query | Bytes | Meaning |
|---|---|---|
| `GET_HEARTBEAT` | `53 59 01 01 00 01 0F BE 54 43` | ctrl 0x01/cmd 0x01 — liveness check |
| `GET_HUMAN_STATUS` | `53 59 80 81 00 01 0F BD 54 43` | ctrl 0x80/cmd 0x81 — presence: data[0] 0=none,1=someone |

Response frames for human info use **either the bare command byte or that byte OR'd with
0x80** interchangeably (e.g. `0x01` or `0x81` both mean the same field) — a from-scratch
parser must accept both, or it will silently miss valid presence replies.

**Firmware's actual query (`prop_aux_radar.c`, `s_seeed_query`):** `53 59 80 01 00 01 0F 3D 54 43`
— ctrl=0x80, cmd=**0x01** (the base variant, not ESPHome's 0x81) — a valid, equivalent
`GET_HUMAN_STATUS`-family query since the parser accepts both.

**Firmware behavior:**
- Polls the query once per second (this is the "must send init each second" behavior noted in
  CLAUDE.md's module map).
- Any well-formed frame (regardless of ctrl/cmd) resets the offline watchdog.
- Only frames with `ctrl==0x80 && cmd==0x01` update cached presence state, read from `data[0]`
  (frame byte index 6): nonzero = present.
- Rolling byte-buffer scanner: resyncs on `0x53 0x59`, validates `len_h==0`, `len_l<=32`, tail
  `0x54 0x43`, and checksum; discards a byte at a time on any mismatch.
- 5000 ms with no valid frame → state falls back to `AUX_OFFLINE`.

## 5. Board wiring (this project)

| Signal | ESP32-P4 GPIO | Connector |
|---|---|---|
| UART3 TX (P4→sensor RX) | GPIO47 | J2 pin 2 |
| UART3 RX (sensor TX→P4) | GPIO48 | J2 pin 1 |
| Baud | 115200 8N1 | — |

Task: `prop_seeed`, core 1, 3072B stack, priority 3, 512B UART RX ring buffer.

## 6. Integration gotchas

- **Query-driven, not free-running** — unlike the LD2450 (continuous unsolicited streaming) or
  the SEN0395 (silent until `sensorStart`), the MR24HPC1 needs an active ~1 Hz poll to get
  presence updates. It can emit unsolicited frames too (e.g. heartbeat replies), and firmware
  treats *any* valid frame as a liveness signal while only presence-tagged frames update cached
  state — so a device that's alive but only answering heartbeats can look "online" yet frozen
  at its last presence value. Check this first if presence reads seem stuck.
- Checksum is an additive sum, not a real CRC — easy to hand-verify with a logic analyzer.
- The datasheet's "Upper Computer Software" sensitivity levels 1-3 / scene modes
  (Small-Area/Area Detection/Maximum Area) are real MR24HPC1 features (`CONTROL_WORK` 0x05,
  commands 0x87/0x88/0x89 per ESPHome constants) but **this firmware doesn't use them** — it
  only polls human-status. Not wired up, not needed for the current "presence" use case.
