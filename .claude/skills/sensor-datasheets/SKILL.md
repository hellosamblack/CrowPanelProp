---
name: sensor-datasheets
description: >
  Verified technical reference for the four external sensor modules documented under
  docs/datasheets/externalDevices/ — HLK-LD2450 (3-target UART tracking radar), MPU-6500
  (IMU/DMP), Seeed MR24HPC1 (UART presence radar), and DFRobot SEN0395 (UART presence radar).
  Use whenever touching main/prop_motion.c, main/prop_imu.c, main/prop_imu_iic.c, or
  main/prop_aux_radar.c, or when asked about any of these sensors' wire protocol, frame
  format, pinout, electrical specs, command set, or datasheet accuracy. Also use before
  trusting any README.md under docs/datasheets/externalDevices/ at face value — two of the
  four were found to contain wrong/duplicated content (see below).
---

# External sensor datasheets — verified reference

`docs/datasheets/externalDevices/` holds vendor PDFs plus a per-device `README.md` the user
generated (likely via an auto-summarization pass over the PDFs). **Two of those four READMEs
were found to be wrong or duplicated** — this skill exists because trusting them blindly would
have propagated bad facts (wrong baud rates, nonexistent GPIO pins, wrong protocol) into
firmware work. Each reference file below is fact-checked against the real PDFs *and* the
firmware code that's actually running on hardware — prefer these over skimming the datasheets
cold.

## Trust table

| Device | Old README.md | Verdict |
|---|---|---|
| **HLK-LD2450** | Described a different sensor's GPIO presence protocol (GP1/GP2, 115200 baud, "Upper Computer" sensitivity levels) | **Wrong — was byte-identical to the MR24HPC1 README, misfiled.** Replaced; see `references/ld2450.md` |
| **MR24HPC1** | Same content as the LD2450's old README | **Correct for MR24HPC1** (specs check out against its real PDF) — it just also got copy-pasted into the LD2450 folder by mistake. See `references/mr24hpc1.md` |
| **SEN0395** | DFRobot command/protocol reference | **Accurate** — verified against PDF + vendor `.h`/`.ino`. See `references/sen0395.md` for a couple of additions (default range/latency values, inter-command timing) |
| **MPU-6500** | Sourced from a PS-MPU-6000A (MPU-6000/6050) datasheet | **Partially reusable** — I2C protocol/address/FS-range facts carry over; VDD voltage range, pin-8 (VLOGIC/VDDIO) story, and DMP feature claims do not. See `references/mpu6500.md` |

## Devices at a glance

- **HLK-LD2450** (`references/ld2450.md`) — 24GHz FMCW, 3-target Cartesian (X/Y/speed) UART
  tracking radar, 256000 baud, 10 Hz, ±60° FOV. Firmware: `main/prop_motion.c`, UART2 on
  GPIO53/54 (J9/J11). **Sign-magnitude encoding, not two's complement** — see the
  `ld2450-sign-magnitude` project memory. Only the read-only data-frame protocol is
  implemented; a documented but unimplemented config-command protocol (zone filtering,
  baud/mode switching) is captured in the reference file if ever needed.
- **MPU-6500** (`references/mpu6500.md`) — I2C IMU (accel+gyro+DMP), address 0x68/0x69, on the
  shared I2C1 bus with the touchscreen (see the `board-io` skill for the bus map). Chip is
  confirmed genuinely MPU-6500 (WHO_AM_I 0x70) via the `imu-is-mpu6500` project memory.
  Firmware: `main/prop_imu.c` + `main/prop_imu_iic.c`, using the vendored LibDriver
  `components/mpu6500/` eMD core for DMP quaternion/tap/pedometer.
- **MR24HPC1** (`references/mr24hpc1.md`) — Seeed 24GHz presence sensor, binary UART
  query/response protocol (header `53 59`, tail `54 43`, additive checksum), 115200 baud,
  query-driven (must poll ~1 Hz). Firmware: `main/prop_aux_radar.c`, UART3 on GPIO47/48 (J2).
- **SEN0395** (`references/sen0395.md`) — DFRobot 24GHz presence sensor, ASCII command
  protocol (`sensorStop`/`sensorStart`/`detRangeCfg`/etc.) + `$JYBSS,...*` output frames,
  115200 baud. Firmware: `main/prop_aux_radar.c`, UART1 on GPIO33/34 (J10).

Both aux-radar sensors (MR24HPC1, SEN0395) are driven by the same file, `prop_aux_radar.c`, as
two independent tasks on core 1 — see each reference file's "Integration gotchas" section for
how their liveness/offline handling differs (SEN0395 self-heals by re-issuing `sensorStart`;
MR24HPC1 just logs and waits for its next poll).

## How to use this skill

1. Read the relevant `references/<device>.md` file before writing or debugging protocol code
   for that sensor — it has the verified frame format, command set, and gotchas the raw PDF
   and/or old README wouldn't reliably give you.
2. If you find yourself about to cite a fact from a `README.md` under
   `docs/datasheets/externalDevices/` that isn't already confirmed in these reference files,
   cross-check it against the real PDF or the firmware code first — the LD2450/MR24HPC1 mixup
   shows those READMEs aren't automatically trustworthy.
3. If a new external device datasheet gets added to that folder, this skill should gain a
   matching `references/<device>.md` — verify its README the same way (compare against the PDF
   and the firmware driver that actually talks to it) before trusting it at face value.

## Companion skills

- **board-io** — GPIO/connector assignments; the canonical source for which pins each of these
  sensors actually uses and what else shares that bus/connector.
