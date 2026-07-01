# SEN0395 mmWave Radar Sensor - Technical Reference & Integration Guide

This repository contains technical specifications, pinout configurations, serial command protocols, and API references for the [DFRobot mmWave Radar Sensor (SKU: SEN0395)](https://www.dfrobot.com/product-2282.html)[cite: 1].

---

### 1. Technical Specifications

| Parameter | Specification / Value |
| :--- | :--- |
| **Operating Voltage** | 3.6V to 5V [cite: 20] |
| **Operating Current** | 90mA [cite: 21] |
| **Operating Frequency** | 24GHz [cite: 26] |
| **Modulation Mode** | FMCW, CW [cite: 25] |
| **Equivalent Transmit Power** | 13 to 15dBm [cite: 23] |
| **Detection Distance** | 9m [cite: 22] |
| **Beam Angle** | $100 \times 40^{\circ}$ [cite: 24] |
| **Default Baud Rate** | 115200 bps [cite: 28] |
| **Operating Temperature** | -40°C to 85°C [cite: 27] |
| **Dimensions** | 24 x 28 mm / 0.94 x 1.10" [cite: 29] |

---

### 2. Pinout & Hardware Interface

| Num | Label | Type | Description |
| :--- | :--- | :--- | :--- |
| **1** | UART Tx | Output | Sensor UART Transmitting [cite: 38] |
| **2** | UART Rx | Input | Sensor UART Receiving [cite: 38] |
| **3** | GPIO1 | I/O | Universal Input and Output [cite: 39] |
| **4** | GPIO2 | Output | Universal Input and Output. Outputs **High** when presence is detected, otherwise outputs **Low** (Default)[cite: 39]. |
| **5** | GND | Power | Ground [cite: 39] |
| **6** | VCC | Power | Power + (3.6V to 5V) [cite: 20, 39] |
| **7** | NC | — | Reserved, left floating [cite: 39] |
| **8** | NC | — | Reserved, left floating [cite: 39] |

---

### 3. Usage & Hardware Connections

The module can act as a standalone level-triggered switch via `GPIO2` [cite: 40, 60] or communicate via `UART` to dynamically fetch data and modify internal settings[cite: 41, 54].

#### Microcontroller Interconnection Map

| mmWave Radar Pin | Arduino Uno Pin | FireBeetle ESP32 Pin | FireBeetle ESP8266 Pin | Beetle ESP32-C3 Pin |
| :--- | :--- | :--- | :--- | :--- |
| **VCC** | 5V [cite: 216] | 3V3 [cite: 332] | 3V3 [cite: 423] | 3V3 [cite: 506] |
| **GND** | GND [cite: 218] | GND [cite: 332] | GND [cite: 423] | GND [cite: 506] |
| **RX** | D2 (Software Tx) [cite: 220] | D3 (Hardware Tx) [cite: 332] | D5 (Software Tx) [cite: 423] | D3 (Hardware Tx) [cite: 506] |
| **TX** | D3 (Software Rx) [cite: 222] | D2 (Hardware Rx) [cite: 332] | D2 (Software Rx) [cite: 423] | D2 (Hardware Rx) [cite: 506] |

---

### 4. Serial Configuration Commands

#### Serial Interface Configuration
* **Baud Rate:** 115200 bps [cite: 55]
* **Data Bits:** 8 [cite: 55]
* **Stop Bits:** 1 [cite: 55]
* **Parity:** None [cite: 55]
* **Flow Control:** None [cite: 55]
* **Format:** ASCII string ending with a new line (`\n`)[cite: 56]. Space character (` `) separates commands and arguments[cite: 57].

> **CRITICAL RULE:** The sensor **must be in a halted state** (`sensorStop`) prior to editing parameters[cite: 58]. Modifications must be permanently stored to flash using the `saveCfg` string[cite: 59, 152].

#### Command Registry Syntax

* **Stop Sensor Processing**
  * Syntax: `sensorStop` [cite: 145]
  * Response: `Done` (Success) or `Error` (Sensor already stopped)[cite: 150].

* **Start Sensor Processing**
  * Syntax: `sensorStart` [cite: 129]
  * Response: `Done` or `Error`[cite: 134].

* **Configure Detection Area Range**
  * The range boundary spans a maximum grid of 128 indices (0 to 127), with each unit representing **15cm**[cite: 74, 75]. Up to 4 disjoint valid range windows (A, B, C, D) are configurable[cite: 76].
  * Syntax: `detRangeCfg par1 parA_s parA_e parB_s parB_e parC_s parC_e parD_s parD_e` [cite: 77]
  * Parameters: `par1` is hardcoded to `-1`[cite: 78]. Indices must satisfy $0 \le \text{start} < \text{end} \le 127$ in strictly ascending order[cite: 76, 78].
  * Example (1.5m to 3.0m): `detRangeCfg -1 10 20` [cite: 81]

* **Configure Output Delay Latency**
  * Syntax: `outputLatency par1 par2 par3` [cite: 102]
  * Parameters: 
    * `par1`: Hardcoded to `-1`[cite: 103].
    * `par2`: Target detection entry verification buffer delay. Range: `0` to `65535` (Units of **25ms**)[cite: 103].
    * `par3`: Target disappearance dropout confirmation delay. Range: `0` to `65535` (Units of **25ms**)[cite: 103].
  * Example (5s detect, 20s clear): `outputLatency -1 200 800` [cite: 118]

* **Configure Power-on Start-up Mode**
  * Syntax: `sensorCfgStart par1` [cite: 121]
  * Parameters: `0` = Wait for explicit `sensorStart` sequence[cite: 122]; `1` = Auto-run immediately upon initial boot up[cite: 122].

* **Write Configurations to Non-Volatile Flash Memory**
  * Syntax: `saveCfg 0x45670123 0xCDEF89AB 0x956128C6 0xDF54AC89` [cite: 156]
  * Response: `Done` or `Error` (Fails if no parameter modification occurred)[cite: 155].

* **Software System Reset**
  * Syntax: `resetSystem` [cite: 137]
  * Response: *No response string* (Module undergoes immediate system cycle)[cite: 142].

* **Factory Reset**
  * Reset parameters back to default system options[cite: 158].
  * Syntax: `factoryReset 0x45670123 0xCDEF89AB 0x956128C6 0xDF54AC89` [cite: 162]

---

### 5. Raw Serial Output Protocol

When operational, the sensor sends out data packets over UART at a persistent refresh rate of **1Hz** by default[cite: 68].

* **Frame Sentence Structure:** `\$JYBSS,par1,par2,par3,par4*` [cite: 69]
* **String Delimiters:** Starts with `\$`[cite: 66], parameters separated via `,`[cite: 67], ends with an asterisk `*` character[cite: 66].

#### Field Bitmask Mapping
* **par1 (Induction Status Indicator):** [cite: 70]
  * `0`: Area Clear / Vacant [cite: 70]
  * `1`: Active target present (moving, micro-moving, or static/stationary) [cite: 7, 70]
* **par2:** Reserved placeholder space character [cite: 67, 70]
* **par3:** Reserved placeholder space character [cite: 67, 70]
* **par4:** Reserved placeholder space character [cite: 67, 70]

---

### 6. Library API Reference

The following components are exposed by the official underlying `DFRobot_mmWave_Radar` integration driver class[cite: 170, 178]:

| Function Prototype | Argument Descriptions | Return Value | Functional Scope |
| :--- | :--- | :--- | :--- |
| `DFRobot_mmWave_Radar(Stream *s);` | `s`: Reference pointer toward a physical hardware or software serial communication wrapper instance[cite: 177, 178]. | None | Constructor framework instantiation[cite: 176, 178]. |
| `void DetRangeCfg(float parA_s, float parA_e);` | `parA_s`: Segment 1 window starting distance (m)[cite: 182].<br>`parA_e`: Segment 1 window ending distance (m)[cite: 182]. | `void` | Overloaded definition to establish a lone valid monitoring window segment[cite: 181, 183]. |
| `void DetRangeCfg(float parA_s, float parA_e, float parB_s, float parB_e);` | Configures consecutive active boundary monitoring pairs A and B in meters[cite: 187, 188]. | `void` | Configures two active monitoring window segments[cite: 186, 188]. |
| `void DetRangeCfg(float parA_s, float parA_e, float parB_s, float parB_e, float parC_s, float parC_e);` | Configures consecutive active boundary monitoring pairs A, B, and C in meters[cite: 192, 193]. | `void` | Configures three active monitoring window segments[cite: 191, 193]. |
| `void DetRangeCfg(float parA_s, float parA_e, float parB_s, float parB_e, float parC_s, float parC_e, float parD_s, float parD_e);` | Configures consecutive active boundary monitoring pairs A, B, C, and D in meters[cite: 196, 198, 199]. | `void` | Configures all four active monitoring window segments[cite: 195, 199]. |
| `void OutputLatency(float par1, float par2);` | `par1`: Detect filter arrival verification latency window.<br>`par2`: Target loss decay holding pattern duration window. | `void` | Sets the output delay configurations[cite: 206, 209]. |
| `bool readPresenceDetection(void);` | None | `true`: Motion or human presence detected.<br>`false`: Sensing area clear[cite: 202]. | Polls and processes incoming UART frames to retrieve the target status[cite: 201, 203]. |
| `void factoryReset(void);` | None | `void` | Triggers a full system wipe to restore default factory configurations[cite: 212, 213]. |