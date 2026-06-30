\### 1. Technical Specifications



| Parameter | Conditions | Min | Typ | Max | Units |

| --- | --- | --- | --- | --- | --- |

| \*\*VDD Operating Voltage\*\* | Normal Operation | 2.375 | - | 3.46 | V 



&#x20;|

| \*\*VLOGIC Operating Voltage\*\* | MPU-6050 only, must be ≤ VDD | 1.71 | - | VDD | V 



&#x20;|

| \*\*Gyroscope Full-Scale Range\*\* | User-programmable (FS\_SEL=0,1,2,3) | - | ±250, ±500, ±1000, ±2000 | - | °/s 



&#x20;|

| \*\*Gyroscope Sensitivity\*\* | For ±250°/s range | - | 131 | - | LSB/(°/s) 



&#x20;|

| \*\*Accelerometer Full-Scale Range\*\* | User-programmable (AFS\_SEL=0,1,2,3) | - | ±2, ±4, ±8, ±16 | - | g 



&#x20;|

| \*\*Accelerometer Sensitivity\*\* | For ±2g range | - | 16,384 | - | LSB/g 



&#x20;|

| \*\*ADC Word Length\*\* | Gyro \& Accel, Two's complement format | - | 16 | - | bits 



&#x20;|

| \*\*Normal Operating Current\*\* | Gyro + Accel + DMP enabled | - | 3.9 | - | mA 



&#x20;|

| \*\*Idle Mode Supply Current\*\* | Full-Chip idle | - | 5 | - | µA 



&#x20;|

| \*\*Operating Temperature\*\* | Specified Temperature Range | -40 | - | +85 | °C 



&#x20;|



\### 2. Pinout \& Hardware Interface



| Pin | MPU-6000 | MPU-6050 | Description |

| --- | --- | --- | --- |

| 1 | CLKIN | CLKIN | Optional external reference clock input. Connect to GND if unused. 



&#x20;|

| 6 | AUX\_DA | AUX\_DA | I²C master serial data, for connecting to external sensors. 



&#x20;|

| 7 | AUX\_CL | AUX\_CL | I²C Master serial clock, for connecting to external sensors. 



&#x20;|

| 8 | /CS | VLOGIC | MPU-6000: SPI chip select (0=SPI mode). MPU-6050: Digital I/O supply voltage. 



&#x20;|

| 9 | AD0/SDO | AD0 | MPU-6000: I²C Slave Address LSB (AD0); SPI serial data output (SDO). MPU-6050: I²C Slave Address LSB. 



&#x20;|

| 10 | REGOUT | REGOUT | Regulator filter capacitor connection. 



&#x20;|

| 11 | FSYNC | FSYNC | Frame synchronization digital input. Connect to GND if unused. 



&#x20;|

| 12 | INT | INT | Interrupt digital output (totem pole or open-drain). 



&#x20;|

| 13 | VDD | VDD | Power supply voltage and Digital I/O supply voltage. 



&#x20;|

| 18 | GND | GND | Power supply ground. 



&#x20;|

| 20 | CPOUT | CPOUT | Charge pump capacitor connection. 



&#x20;|

| 22 | CLKOUT | CLKOUT | System clock output. 



&#x20;|

| 23 | SCL/SCLK | SCL | MPU-6000: I²C serial clock (SCL); SPI serial clock (SCLK). MPU-6050: I²C serial clock (SCL). 



&#x20;|

| 24 | SDA/SDI | SDA | MPU-6000: I²C serial data (SDA); SPI serial data input (SDI). MPU-6050: I²C serial data (SDA). 



&#x20;|



\### 3. Usage \& Hardware Connections



\* \*\*External Components Required:\*\*

\* \*\*REGOUT (Pin 10):\*\* Ceramic, X7R, 0.1µF ±10%, 2V capacitor to GND. 





\* \*\*VDD (Pin 13):\*\* Ceramic, X7R, 0.1µF ±10%, 4V capacitor to GND. 





\* \*\*CPOUT (Pin 20):\*\* Ceramic, X7R, 2.2nF ±10%, 50V capacitor to GND. 





\* \*\*VLOGIC (Pin 8, MPU-6050 only):\*\* Ceramic, X7R, 10nF ±10%, 4V capacitor to GND. 









\* \*\*I²C Connection Requirements:\*\* SDA and SCL lines are open-drain and bi-directional, requiring pull-up resistors to VDD. 







\### 4. Serial Configuration Commands



\* \*\*I²C Interface:\*\*

\* \*\*Bus Speed:\*\* Up to 400kHz (Fast Mode) or 100kHz (Standard-Mode). 





\* \*\*Addressing:\*\* 7-bit slave address. `b1101000` (if AD0 is logic low) or `b1101001` (if AD0 is logic high). 









\* \*\*SPI Interface (MPU-6000 Only):\*\*

\* \*\*Bus Speed:\*\* 1MHz for reading/writing all registers; 20MHz strictly for reading sensor and interrupt registers. 





\* \*\*Data Delivery:\*\* MSB first, LSB last. 





\* \*\*Clocking:\*\* Data is latched on the rising edge of SCLK and transitioned on the falling edge of SCLK. 





\* \*\*SPI Mode Prevention:\*\* To prevent accidental switching into I²C mode while using SPI, disable the I²C interface by setting the `I2C\_IF\_DIS` configuration bit. 











\### 5. Raw Serial Output Protocol



\* \*\*I²C Data Transfer Protocol:\*\*

\* \*\*Single-Byte Write:\*\* START -> `Slave\_Addr + Write(0)` -> ACK -> `Register\_Addr` -> ACK -> `Data` -> ACK -> STOP. 





\* \*\*Burst Write:\*\* START -> `Slave\_Addr + Write(0)` -> ACK -> `Register\_Addr` -> ACK -> `Data` -> ACK -> `Data` -> ACK -> STOP. 





\* \*\*Single-Byte Read:\*\* START -> `Slave\_Addr + Write(0)` -> ACK -> `Register\_Addr` -> ACK -> START -> `Slave\_Addr + Read(1)` -> ACK -> `Data` -> NACK -> STOP. 





\* \*\*Data Format:\*\* Outputs from the ADCs are provided in 16-bit two's complement format. 









\* \*\*SPI Data Transfer Protocol (MPU-6000 Only):\*\*

\* \*\*Address Format:\*\* The first byte contains the Read/Write bit (1 = Read, 0 = Write) in the MSB, followed by 7 bits containing the Register Address. 











\### 6. Library API Reference



\*Note: The provided datasheet strictly outlines hardware and protocol-level specifications. The exact register map values and software API definitions are explicitly documented by the manufacturer in a separate reference manual ("MPU-6000/MPU-6050 Register Map and Register Descriptions").\* 



| API Feature / Setting | Parameter | Description |

| --- | --- | --- |

| \*\*Gyroscope Full-Scale Selection\*\* | `FS\_SEL` | 0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s. 



&#x20;|

| \*\*Accelerometer Full-Scale Selection\*\* | `AFS\_SEL` | 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g. 



&#x20;|

| \*\*Free Fall Detection Threshold\*\* | `FF\_THR` | Threshold for free-fall detection, set in 1 mg increments. 



&#x20;|

| \*\*Free Fall Detection Duration\*\* | `FF\_DUR` | Duration required to trigger free-fall interrupt, set in 1 ms increments. 



&#x20;|

| \*\*Motion Detection Threshold\*\* | `MOT\_THR` | Acceleration threshold for motion detection interrupt, set in 1 mg increments. 



&#x20;|

| \*\*Zero Motion Threshold\*\* | `ZRMOT\_THR` | Absolute value threshold for zero motion detection, set in 1 mg increments. 



&#x20;|



\### 7. Appendix



\*\*Digital Motion Processor (DMP) Capabilities:\*\*



\* Offloads motion processing computation from the host processor.





\* Acquires and processes data from the internal accelerometer, gyroscope, and 3rd-party external sensors (e.g., magnetometers).





\* Outputs results to dedicated DMP registers or the internal FIFO buffer.





\* Generates hardware interrupts via an external MPU pin.





\* Supports 3D Motion Processing, gesture recognition, and low-power pedometer functions.





\* Typically executes algorithms at 200Hz to guarantee low-latency results regardless of the host's slower update rate.

