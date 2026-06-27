---
name: board-io
description: >
  Definitive IO map and pin-assignment advisor for the CrowPanel ESP32-P4 7-inch board.
  Use whenever you are assigning GPIOs, wiring external peripherals, choosing a bus for
  LoRa/nRF24/UART/I2C/SPI/ADC modules, asking which pins are free, or need to know what
  signals are available on J2/J7/J9/J10/J11/J13 and which share the touchscreen or STC8
  bus. Also trigger for "what pins can I use", "where do I connect X", "is GPIO N free",
  "which header has 5V near UART", "closest unused ADC pin", or any capability-aware
  pin-selection question.
---

# CrowPanel ESP32-P4 7" — IO Map & Pin Advisor

Source of truth: full netlist analysis of `reference/schematic/ESP32-P4 Display 7.0 inch V1.0.net`
plus firmware scan. GPIO numbers confirmed against firmware source.

**Live pin registry:** `docs/gpio_registry.yml` — read this first, update it when you assign a pin.

---

## How to answer "what are the best pins for X?"

1. **Read `docs/gpio_registry.yml`** to get current status of every broken-out GPIO.
2. **Filter by capability** from the table below (ADC? LP-capable? 5V-tolerant?).
3. **Filter by status** — prefer `available`, then `shared` (bsp_aio pins can be reclaimed),
   then `radar`/`radio` (free when those peripherals are disabled).
4. **Prefer physical proximity** — pick pins on the same connector, or adjacent pins on J7.
   Fewer cables = fewer wiring errors. Use the layout maps below.
5. **Check power rail availability** on that connector (3.3 V, 5 V, or neither).
6. **Update `docs/gpio_registry.yml`** with the new assignment before finishing.

---

## GPIO Capability Table — Broken-Out Pins Only

All broken-out GPIOs are in the ESP32-P4's fully-connected GPIO matrix, meaning
**UART, SPI, I2C, PCNT, RMT, LEDC/PWM, MCPWM, I2S** can be routed to any of them in
software. The table focuses on capabilities that are NOT remappable — ADC, touch sensing,
low-power domain, and strapping constraints.

| GPIO | Connector/Pin | Status | ADC | Touch | LP-sleep | Strapping | Special / Notes |
|------|---------------|--------|-----|-------|----------|-----------|-----------------|
| 2 | J7/9 | shared (AIO) | — | CH1 | ✓ | — | JTAG TCK alternate |
| 3 | J7/11 | shared (AIO) | — | CH2 | ✓ | — | JTAG TDI alternate |
| 4 | J7/13 | shared (AIO) | — | CH3 | ✓ | — | JTAG TMS alternate |
| 5 | J7/15 | shared (AIO) | — | CH4 | ✓ | — | JTAG TDO alternate |
| 6 | J9/4 | radio (SPI MOSI) | — | CH5 | ✓ | — | Free without radio module |
| 7 | J9/3 | radio (SPI MISO) | — | CH6 | ✓ | — | Free without radio module |
| 8 | J9/2 | radio (SPI CLK) | — | CH7 | ✓ | — | Free without radio module |
| 9 | J11/5 | radio (IRQ) | — | CH8 | ✓ | — | Free without radio module |
| 10 | J11/6 | radio (NSS) | — | CH9 | ✓ | — | Free without radio module |
| 25 | J7/17 | shared (AIO) | — | — | — | — | HP-domain only |
| 27 | J7/19 | shared (AIO) | — | — | — | — | HP-domain only |
| 28 | J7/20 | shared (AIO) | — | — | — | — | HP-domain only |
| 33 | J10/2 | pending (BTN_MODE) | — | — | — | — | UART3 RX, 5V-tolerant via BSS138 |
| 34 | J10/1 | available | — | — | — | JTAG_SEL | UART3 TX, 5V-tolerant via BSS138; high at boot (pull-up on PCB) |
| 47 | J2/2 | pending (LED_SIGNAL) | — | — | — | — | UART1 TX; verify wiring |
| 48 | J2/1 | pending (LED_POWER) | — | — | — | — | UART1 RX; confirmed broken-out |
| 49 | J7/18 | shared (AIO) | ADC2-CH0 | — | — | — | |
| 50 | J7/16 | shared (AIO) | ADC2-CH1 | — | — | — | |
| 51 | J7/14 | shared (AIO) | ADC2-CH2 | — | — | — | ACOMP0 reference |
| 52 | J7/12 | shared (AIO) | ADC2-CH3 | — | — | — | ACOMP0 non-inverting input |
| 53 | J9/1 | radar (UART2 TX) | ADC2-CH4 | — | — | — | ACOMP1 reference; free without radar |
| 54 | J11/1 | radar (UART2 RX) | ADC2-CH5 | — | — | — | ACOMP1 non-inverting input; free without radar |

**Notes:**
- `LP-sleep ✓` = GPIO stays alive during deep-sleep (VDD_LP powered); all GPIO2–10 qualify.
- `ADC` = dedicated analog-to-digital input, not remappable via GPIO matrix. ADC2 shares
  hardware with Wi-Fi; call `adc_lock_acquire(ADC_UNIT_2)` before sampling.
- `Touch` = capacitive touch sensor channel (hardware touch controller). Only GPIO2–15.
- All `pending` pins have placeholder assignments in `bsp_io.c` — safe to reassign if
  not yet wired to hardware; update both `bsp_io.c` and `gpio_registry.yml`.
- `radio` pins are managed by `bsp_aio` and free when no radio module is fitted.
- `shared (AIO)` pins are controlled by `bsp_aio` — they can be reclaimed for a dedicated
  peripheral but you must remove them from `bsp_aio`'s `s_pins[]` table first.

---

## Physical Layout — Connector Maps

Use these to find adjacent pins that share a connector (important for short wiring runs).

### J7 — Expansion Header (2×12 SMD, 24-pin)

Dual-row, pin 1 top-left (row A odd, row B even):

```
Pin  A1 [+3.3V]  B2 [+5V  ]
     A3 [+3.3V]  B4 [+5V  ]
     A5 [+3.3V]  B6 [+5V  ]
     A7 [GND  ]  B8 [GND  ]
     A9 [GPIO2]  B10[GND  ]
    A11 [GPIO3]  B12[GPIO52]  ← ADC pair (GPIO52 & GPIO3 side-by-side)
    A13 [GPIO4]  B14[GPIO51]  ← ADC pair
    A15 [GPIO5]  B16[GPIO50]  ← ADC pair
    A17 [GPIO25] B18[GPIO49]  ← ADC pair
    A19 [GPIO27] B20[GPIO28]  ← GPIO pair (adjacent, no special caps)
    A21 [GND  ]  B22[GND  ]
    A23 [GND  ]  B24[GND  ]
```

**Power available on J7:** +3.3 V (pins 1,3,5) and nominally +5 V (pins 2,4,6 = `/VDD5V` per netlist).
⚠️ **Field note:** user measured no voltage on J7 5V pins and silkscreen shows NC — the schematic designator "J7" may not match the physical expansion header. Verify with a meter before relying on J7 for 5V.

**Adjacent GPIO groups on J7:**
- `GPIO3 / GPIO52` (A11/B12) — digital + ADC pair
- `GPIO4 / GPIO51` (A13/B14) — digital + ADC pair, ACOMP0-ref
- `GPIO5 / GPIO50` (A15/B16) — digital + ADC pair
- `GPIO25 / GPIO49` (A17/B18) — digital + ADC pair
- `GPIO27 / GPIO28` (A19/B20) — two digital-only, side by side

### J9 — SPI2 + UART2-TX (7-pin 2.54 mm, linear)

```
Pin 1 [GPIO53 UART2-TX]  ← radar when active; ADC2-CH4
Pin 2 [GPIO8  SPI-SCK ]  ← radio when active
Pin 3 [GPIO7  SPI-MISO]  ← radio when active
Pin 4 [GPIO6  SPI-MOSI]  ← radio when active
Pin 5 [+3.3V           ]
Pin 6 [GND             ]
Pin 7 [+5V  VDD5V_W    ]  ← L4 is DNP (not populated), so VDD5V_W = VDD5V same net
```

**Power available on J9:** +3.3 V (pin 5), +5 V (pin 7), GND (pin 6).
Note: J9 pin 7 is the most reliable 5V breakout on the board (netlist-confirmed `/VDD5V_W`, same rail as `/VDD5V` since L4=NC).
GPIO53, 8, 7, 6 are four adjacent signal pins on one connector.

### J11 — Combo Header (7-pin 2.54 mm, linear)

```
Pin 1 [GPIO54 UART2-RX ]  ← radar when active; ADC2-CH5
Pin 2 [I2C1 SCL 3.3V  ]  ← SHARED (touchscreen + STC8) — see warning
Pin 3 [I2C1 SDA 3.3V  ]  ← SHARED
Pin 4 [NC              ]
Pin 5 [GPIO9           ]  ← radio IRQ when active; touch CH8; LP-capable
Pin 6 [GPIO10          ]  ← radio NSS when active; touch CH9; LP-capable
Pin 7 [NC              ]
```

**Power available on J11:** none (signal-only header).
GPIO9 and GPIO10 (pins 5,6) are adjacent; GPIO54 (pin 1) is at the far end.

### J10 — UART3 (XH2.54 4-pin, level-shifted to 5V)

```
Pin 1 [GPIO34 UART3-TX ]  strapping: JTAG_SEL; high at boot (safe)
Pin 2 [GPIO33 UART3-RX ]  btn placeholder; UART3 RX
Pin 3 [+5V_IN          ]  ← /+5V_IN net — see power note below
Pin 4 [GND             ]
```

**Power available on J10:** GND (pin 4) only when board-driven.
**Both signal pins are 5 V-tolerant** via BSS138 level shifters.

⚠️ **J10 pin 3 power direction:** pin 3 is on the `/+5V_IN` net, which connects to the ANODE side of D2 and D4 (Schottky diodes that OR into `/VCC5V` and `/VCC5V_IN`). No internal regulator or rail drives this net — it is an **external power injection point**, not a board-driven 5V output. If you need 5V for a sensor on J10, power it from J9 pin 7 (`/VDD5V_W`) with a separate wire, or use a self-powered sensor. A device that provides its own 5V on J10 pin 3 will OR that voltage into the board rails.

### J2 — UART1 (HY2.0 4-pin, 3.3 V logic)

```
Pin 1 [GPIO48 UART1-RX ]  LED placeholder; confirmed broken-out
Pin 2 [GPIO47 UART1-TX ]  LED placeholder; verify wiring
Pin 3 [+3.3V           ]
Pin 4 [GND             ]
```

**Power available on J2:** +3.3 V (pin 3), GND (pin 4). No 5V.

### J13 — Grove I2C (HY2.0 4-pin)

```
Pin 1 [I2C1 SCL 3.3V  ]  SHARED — see warning below
Pin 2 [I2C1 SDA 3.3V  ]  SHARED
Pin 3 [+3.3V           ]
Pin 4 [GND             ]
```

**Power available on J13:** +3.3 V (pin 3), GND (pin 4). No 5V.

---

## Shared I2C1 Bus Warning

J11 pins 2–3 and J13 pins 1–2 expose the **same I2C1 bus** (3.3V side, via Q8/Q9
BSS138 level shifters from the ESP32-P4's 1.8V VDDPST_5 domain).

Active bus participants:
- **GT911 touchscreen** — always polling; I2C address typically `0x14` or `0x5D`
  (check `CONFIG_TOUCH_I2C_ADDRESS` in sdkconfig)
- **STC8H1K08 co-processor** — connected via R188/R189; may act as I2C master or slave
- Unpopulated ES8311 and ES7210 footprints (no chips, but PCB traces are present)

For new I2C peripherals, prefer assigning a fresh I2C port to a pair of free GPIOs
(`i2c_master_bus_create(I2C_NUM_1, ...)`) to avoid address conflicts and bus contention.

---

## Common Module Wiring Templates

### J10 — UART3 (5V-tolerant logic; external 5V needed for powered sensors)

J10 has 5V-tolerant UART via BSS138 level shifters. The 5V pin (pin 3) is an **external
injection input** (`/+5V_IN`), not a board-driven output. Power your 5V sensor from
J9 pin 7 (`/VDD5V_W`, ~4.5V) or self-power it separately.
Update `gpio_registry.yml` status for GPIO33/GPIO34 from `pending` to your module name.

```
Module → Board (J10)                 Board → Sensor 5V
TX   → Pin 2  (GPIO33, UART3 RX)    VCC  ← J9/7  (+5V VDD5V_W, separate wire)
RX   → Pin 1  (GPIO34, UART3 TX)    GND  ← Pin 4 (GND)
GND  → Pin 4  (GND)
```

ESP-IDF: `uart_driver_install(UART_NUM_3, ...)`, `uart_set_pin(UART_NUM_3, 34, 33, ...)`.
Remove GPIO33 from `bsp_io.c` `button_table[]` before using it as UART RX.

### J2 — UART device needing 3.3V power

```
Module → Board (J2)
TX   → Pin 1  (GPIO48, UART1 RX)
RX   → Pin 2  (GPIO47, UART1 TX)
VCC  → Pin 3  (+3.3V)
GND  → Pin 4  (GND)
```

ESP-IDF: `uart_driver_install(UART_NUM_1, ...)`, `uart_set_pin(UART_NUM_1, 47, 48, ...)`.
Remove GPIO47/GPIO48 from `bsp_io.c` `led_table[]` before use.

### J9 — SPI peripheral (radio module slot)

```
Module → Board (J9)
SCK  → Pin 2  (GPIO8)
MISO → Pin 3  (GPIO7)
MOSI → Pin 4  (GPIO6)
CS   → J7/nn  (any free GPIO — GPIO25/27/28 or ADC pins)
IRQ  → J11/5  (GPIO9, touch CH8)
VCC  → Pin 5  (+3.3V) or Pin 7 (+5V)
GND  → Pin 6
```

ESP-IDF: `spi_bus_initialize(SPI2_HOST, ...)`. Multiple CS devices share the bus —
call `spi_bus_add_device()` per chip.

### J7 — ADC analog inputs

GPIO49–52 (J7 pins 18,16,14,12) are ADC2 channels 0–3. Formula: `ch = gpio - 49`.

```c
adc_oneshot_unit_handle_t adc2;
adc_oneshot_unit_init_cfg_t cfg = { .unit_id = ADC_UNIT_2 };
adc_oneshot_new_unit(&cfg, &adc2);
adc_oneshot_chan_cfg_t ch_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
adc_oneshot_config_channel(adc2, ADC_CHANNEL_0, &ch_cfg);  // GPIO49
```

ADC2 shares hardware with Wi-Fi. Call `adc_lock_acquire(ADC_UNIT_2)` or use the
`esp_adc/adc_lock.h` API when ADC2 and Wi-Fi are used concurrently.

### Deep-sleep wake pins

Only LP-domain GPIOs (GPIO2–10 in our header set) survive deep-sleep.
Best candidates for wake-on-signal: GPIO9 (J11/5) and GPIO10 (J11/6) — currently
radio IRQ/NSS, free without radio. Touch channels on GPIO2–10 can also wake via
touch controller in LP mode.

### J3 — Battery (JST-PH 2.0mm 2-pin)

```
Pin 1 [VBAT +    ]  ← single-cell LiPo positive
Pin 2 [GND       ]
```

**Charger:** TP4059 (U2), 500 mA (R16 = 2 kΩ), overcharge cutoff at 4.2 V built-in.
Charging input comes from `/VCC5V_IN` (USB 5V post-Schottky) — charges whenever USB is connected.

**Discharge path:** Q3 (NCE20P45Q dual P-channel FET) auto-switches:
- USB present → Q3 off (VBAT isolated, no backfeed)
- USB absent → Q3 on → VBAT passes to `/VCC5V` → `/VDD5V`

**No boost converter** in the battery path. System runs at raw battery voltage (3.5–4.2 V) on battery-only — USB-derived 5V is NOT regenerated. External 5V accessories need USB or a separate boost module.

**No hardware overdischarge or overcurrent protection** on the discharge path — use a **protected LiPo** (one with built-in PCM). Charging is safe with unprotected cells. Battery voltage monitored by ADC: R164 (39 kΩ) + R186 (100 kΩ) divider → STC8H1K08 pin 13 + ESP32-P4 ADC.

---

## Updating the Registry

When you assign a GPIO to a new peripheral:

1. Open `docs/gpio_registry.yml`
2. Find the entry for that GPIO
3. Change `status:` to the peripheral name (e.g., `lora_sx1262`)
4. Update `function:`, `file:`, and `notes:` fields
5. If the GPIO was `shared (AIO)`, also remove it from `bsp_aio.c`'s `s_pins[]` table

When a peripheral is removed or disabled:
1. Revert status to `available` (or back to `shared` / `radio` / `radar` if returning it)
2. Clear `function:` and `file:`
