### 1. Technical Specifications

| Parameter | Minimum | Typical | Maximum | Unit |
| --- | --- | --- | --- | --- |
| **Operating Voltage (VCC)** | 4.5 

 | 5.0 

 | 6 

 | V 

 |
| **Operating Current (ICC)** | 95 

 | 110 

 | 120 

 | mA 

 |
| **IO Input/Output Current ($I_{IO}$)** | — | 8 

 | 20 

 | mA 

 |
| **Operating Frequency** | 24.0 

 | — | 24.25 

 | GHz 

 |
| **Transmitted Power** | — | 6 

 | 8 

 | dBm 

 |
| **Antenna Gain** | — | 10 

 | — | dBi 

 |
| **Horizontal Beam (3dB)** | — | 100 

 | — | degree 

 |
| **Vertical Beam (3dB)** | — | 80 

 | — | degree 

 |
| **Power Consumption** | — | ≤0.5  / ≤5 

 | — | W 

 |
| **Voltage Ripple** | — | — | 100 

 | mV 

 |
| **Operating Temperature** | -20 

 | — | +85 

 | °C 

 |
| **Storage Temperature** | -40 

 | — | +85 

 | °C 

 |
| **Motion Detection Range** | — | 5 

 | 5 

 | m 

 |
| **Static/Micro-movement Range** | — | 4 

 | 4 

 | m 

 |
| **Sleep Detection Range** | 3 

 | 3.5 

 | — | m 

 |

---

### 2. Pinout & Hardware Interface

#### Interface 1 (Main Header)

| Pin | Name | Electrical Level | Type | Detail / Description |
| --- | --- | --- | --- | --- |
| **1** | 5V | 5.0V 

 | Input 

 | Main power input 

 |
| **2** | GND | — | Ground 

 | System ground 

 |
| **3** | RX | 3.3V 

 | Input 

 | UART Receive pin 

 |
| **4** | TX | 3.3V 

 | Output 

 | UART Transmit pin 

 |
| **5** | GP2 | 3.3V / 0V 

 | Output 

 | Presence Output: High = Human Presence, Low = Human Absence 

 |
| **6** | GP1 | 3.3V / 0V 

 | Output 

 | Motion State Output: Active / Stationary 

 |

#### Interface 2 (Expansion Header)

| Pin | Name | Electrical Level | Type | Detail / Description |
| --- | --- | --- | --- | --- |
| **1** | 3V3 | 3.3V 

 | Input 

 | 3.3V Power input 

 |
| **2** | GND | — | Ground 

 | Ground 

 |
| **3** | SL | — | — | Spare Expansion Pin 

 |
| **4** | SD | — | — | Spare Expansion Pin 

 |
| **5** | GP3 | 3.3V 

 | IO 

 | Parameter selection terminal / Redefinable 

 |
| **6** | GP4 | 3.3V 

 | IO 

 | Parameter selection terminal / Redefinable 

 |
| **7** | GP5 | 3.3V 

 | IO 

 | Parameter selection terminal / Redefinable 

 |
| **8** | GP6 | 3.3V 

 | IO 

 | Parameter selection terminal / Redefinable 

 |

---

### 3. Usage & Hardware Connections

#### Recommended Microcontroller Connections

| Radar Module Pin | Target MCU Pin | Description |
| --- | --- | --- |
| **5V** | 5V / VCC | Stable 5V Power Supply (Ripple ≤ 100mV) 

 |
| **GND** | GND | Common System Ground 

 |
| **RX** | TX | MCU UART Transmit (3.3V Logic) 

 |
| **TX** | RX | MCU UART Receive (3.3V Logic) 

 |
| **GP2** | GPIO (Input) | Hardware interrupt/digital read for Presence/Absence 

 |
| **GP1** | GPIO (Input) | Hardware interrupt/digital read for Active/Stationary 

 |

#### Installation Geometry Matrix

* **Horizontal Installation:** Height 1 m to 1.5 m. Horizontal angle deviation $\le\pm5^{\circ}$. Moving detection range $\le 5$ m, sitting/fretting range $\le 4$ m, sleep range $\le 3$ m.


* **Diagonal Installation:** Height 2 m to 2.75 m. Downtilt angle range $10^{\circ}$ to $30^{\circ}$. Moving detection range $\le 4.5$ m, sitting/fretting range $\le 2.5$ m, sleep range $\le 1.8$ m.


* **Top Mounting:** Height $\le 2.75$ m. Horizontal deviation angle $\le 3^{\circ}$. Moving detection range $\le 4.3$ m, sitting/fretting range $\le 4$ m, sleep range $\le 1.8$ m.



---

### 4. Serial Configuration Commands

* **Baud Rate:** 115200 bps 


* **Data Bits:** 8 (Standard UART configuration assumed for standard Upper Computer interface) 


* **Parity:** None 


* **Stop Bits:** 1 



(Note: Specific underlying command strings, hex libraries, and programming syntax structures are handled through the official Upper Computer Software or the Arduino library integration and are not itemized explicitly in this hardware manual ).

---

### 5. Raw Serial Output Protocol

The radar module utilizes two implementation mechanisms to pass data outputs:

1. **Standard Function Mode:** Digital I/O high/low state indicators.


* **GP2:** Outputs `3.3V` (High Level) upon human presence detection and `0V` (Low Level) when no human presence is detected.


* **GP1:** Outputs binary states indicating whether the detected presence is Active or Stationary.




2. **Underlying Open Function Mode:** Delivers internal echo demodulation signals processed by the ADC directly to the MCU. The onboard MCU calculates the parameters below to determine target characteristics:


* Echo Body Motion Amplitude 


* Signal Frequency 


* Signal Phase 





---

### 6. Library API Reference

The hardware document references integration support via two primary debugging options:

* **Upper Computer Software Execution:** For graphical configuration, sensitivity adjustment (Levels 1–3), and operational scene mode selection (Maximum Area, Area Detection, Small-Area Detection).


* **Arduino Library Integration:** Configures dynamic detection ranges and trigger threshold state-change delays over the serial link.



| Feature / Setting Option | Area Mode Parameter | Target Coverage | Base Sensitivity Level |
| --- | --- | --- | --- |
| **Maximum Area Mode** | Wide angle 9m / Narrow angle 6m 

 | Activity: $6\text{m} \times 9\text{m}$ 

 | 3 

 |
| **Area Detection Mode (Default)** | Wide angle 7m / Narrow angle 5m 

 | Stationary: $4\text{m} \times 5\text{m}$ 

 | 2 

 |
| **Small-Area Detection Mode** | Wide angle 5.5m / Narrow angle 4m 

 | Sleep Scanning: $3\text{m} \times 2\text{m}$ 

 | 1 

 |