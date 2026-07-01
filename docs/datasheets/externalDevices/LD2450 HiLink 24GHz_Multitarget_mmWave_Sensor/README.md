# HLK-LD2450 24GHz Multi-Target mmWave Tracking Radar

> **Note:** this file previously contained content that was actually the Seeed MR24HPC1's
> spec sheet (GP1/GP2 presence pins, 115200 baud, "Upper Computer" sensitivity levels), copied
> here by mistake — it doesn't describe this sensor at all. Replaced below with verified
> LD2450-specific facts, cross-checked against the real datasheet PDFs in this folder and
> against the working firmware driver (`main/prop_motion.c`). See also the
> `sensor-datasheets` Claude Code skill (`.claude/skills/sensor-datasheets/references/ld2450.md`)
> for the fuller reference including the unimplemented config-command protocol.

### 1. Technical Specifications

| Parameter | Value | Notes |
| --- | --- | --- |
| **Supply Voltage** | 5.0V (4.5–6V) | Module regulates to 3.3V internally |
| **Supply Current** | 95–120 mA typ | Input supply must be >200 mA capacity |
| **IO / UART Logic Level** | 3.3V | |
| **Operating Frequency** | 24.0–24.25 GHz (FMCW) | 250 MHz sweep bandwidth |
| **Transmitted Power** | 6–8 dBm | Antenna gain ~10 dBi |
| **Detection Range** | up to 6 m (spec); ~8 m near boresight per polar plot | Narrows off-axis |
| **Azimuth FOV** | **±60°** | Hard sensor limit |
| **Elevation Beam (3dB)** | ~80° | |
| **Max Simultaneous Targets** | **3** | Protocol always reports 3 fixed target slots |
| **Data Refresh Rate** | **10 Hz** | |
| **Default Baud Rate** | **256000**, 8N1, no parity | Configurable 9600–460800 |
| **Module Size** | 15 × 44 mm | |
| **Operating Temperature** | -40 to +85°C | |

### 2. Pinout & Hardware Interface

The LD2450 has **exactly 4 pins** — no GPIO/expansion header:

| Pin | Name | Level | Type | Description |
| --- | --- | --- | --- | --- |
| 1 | 5V | 5.0V | Input | Main power input |
| 2 | GND | — | Ground | System ground |
| 3 | RX | 3.3V | Input | UART receive |
| 4 | TX | 3.3V | Output | UART transmit |

### 3. Usage & Hardware Connections

| Radar Pin | Target MCU Pin | Description |
| --- | --- | --- |
| 5V | 5V/VCC | Stable 5V supply |
| GND | GND | Common ground |
| RX | MCU TX | 3.3V logic |
| TX | MCU RX | 3.3V logic |

**This board's actual wiring** (`main/prop_motion.c`): UART2, TX=GPIO53→radar RX,
RX=GPIO54←radar TX (J9/J11), 256000 baud 8N1 — matches the factory default exactly.

Mount at 1.5–2 m height, radome must be non-metallic (metal blocks 24 GHz entirely). Avoid
pointing multiple 24 GHz radars directly at each other (mutual interference).

### 4. Data Frame Protocol (implemented by firmware — read-only parsing)

10 Hz, 30-byte fixed-length frames, little-endian:

`AA FF 03 00` (header) · Target1 (8B) · Target2 (8B) · Target3 (8B) · `55 CC` (tail)

Per-target 8 bytes (all 3 slots always present; an inactive slot is all-zero X/Y):

| Field | Type | Bytes | Unit | Notes |
| --- | --- | --- | --- | --- |
| X | sign-magnitude int16 | 0–1 | mm | bit15=sign (1=+), bits0–14=magnitude |
| Y | sign-magnitude int16 | 2–3 | mm | same encoding |
| Speed | sign-magnitude int16 | 4–5 | **cm/s** | same encoding |
| Distance resolution | uint16 | 6–7 | mm | size of the distance gate |

**Sign-magnitude, not two's complement** — decoding as plain int16 corrupts positive values.
See the `ld2450-sign-magnitude` project memory and `main/prop_motion.c`'s decode function.

### 5. Config-Command Protocol (documented, NOT implemented in firmware)

Separate envelope, same UART: `FD FC FB FA` (header) · length(2B) · command word(2B) + value ·
`04 03 02 01` (tail). Must bracket any command between "Enable Config" (`0x00FF`) and "End
Config" (`0x00FE`). Supports single/multi-target mode, baud-rate change (index table,
factory default = `0x0007` = 256000), Bluetooth on/off (**on by default**), and rectangular
zone-filtering (up to 3 zones, include/exclude). Full command table and byte layout in
`.claude/skills/sensor-datasheets/references/ld2450.md`.

### 6. Library / Tooling

Hi-Link ships a PC "Upper Computer" tool (`HLK-LD2450_TOOL.exe`, in the
`HLK-LD2450 Human body tracking detection (3 person)/application software/` subfolder) that
speaks this same 256000-baud protocol — useful for isolating a hardware vs. firmware issue by
plugging the module into a USB-serial adapter directly.
