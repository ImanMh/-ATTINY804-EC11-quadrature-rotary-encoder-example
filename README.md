# ATtiny804 Dual Rotary Encoder I2C Slave

An ATtiny804 firmware that reads two quadrature rotary encoders and exposes their positions and button states over I2C as a slave device.

Uses a proven timer-based polling approach (10kHz via TCB0) for reliable, bounce-free encoder reading — the same technique tested successfully on the ATtiny202.

## Features

- **Two independent quadrature encoders** with push-button support
- **I2C slave interface** with register-based read protocol
- **10kHz timer polling** — no missed ticks, no hardware debouncing needed
- **Position reset commands** via I2C writes
- **Optional heartbeat LED** for visual confirmation the device is running
- **Portable encoder library** — works on ATmega328P, ATmega32U4, and all megaTinyCore tinyAVR boards

## Pin Assignment (14-pin SOIC)

```
         ┌──────────┐
   VDD ──┤ 1    14 ├── GND
ENC2_A ──┤ 2    13 ├── ENC1_BTN   (PA3)
  (PA4)  │          │
ENC2_B ──┤ 3    12 ├── ENC1_B     (PA2)
  (PA5)  │          │
ENC2_BTN─┤ 4    11 ├── ENC1_A     (PA1)
  (PA6)  │          │
LED    ──┤ 5    10 ├── UPDI       (PA0)
  (PA7)  │          │
Free   ──┤ 6     9 ├── I2C SCL    (PB0)
  (PB3)  │          │
Free   ──┤ 7     8 ├── I2C SDA    (PB1)
  (PB2)  └──────────┘
```

| Pin | Port | Function | Notes |
|-----|------|----------|-------|
| 1 | VDD | Power | 3.3V or 5V |
| 2 | PA4 | Encoder 2 Channel A | Internal pull-up enabled |
| 3 | PA5 | Encoder 2 Channel B | Internal pull-up enabled |
| 4 | PA6 | Encoder 2 Push Button | Internal pull-up enabled, active low |
| 5 | PA7 | Status LED | Optional heartbeat, 1Hz blink |
| 6 | PB3 | Free | Available for future use |
| 7 | PB2 | Free | Available for future use |
| 8 | PB1 | I2C SDA | TWI0 default pin |
| 9 | PB0 | I2C SCL | TWI0 default pin |
| 10 | PA0 | UPDI | Programming/debug only |
| 11 | PA1 | Encoder 1 Channel A | Internal pull-up enabled |
| 12 | PA2 | Encoder 1 Channel B | Internal pull-up enabled |
| 13 | PA3 | Encoder 1 Push Button | Internal pull-up enabled, active low |
| 14 | GND | Ground | |

## I2C Interface

### Slave Address

Default: **0x40** (configurable via `I2C_ADDRESS` in the sketch).

### Register Map

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| 0x00 | ENC1_POS_L | R | Encoder 1 position, low byte |
| 0x01 | ENC1_POS_H | R | Encoder 1 position, high byte |
| 0x02 | ENC2_POS_L | R | Encoder 2 position, low byte |
| 0x03 | ENC2_POS_H | R | Encoder 2 position, high byte |
| 0x04 | BUTTONS | R | Button states (bit 0 = enc1, bit 1 = enc2; 1 = pressed) |
| 0x05 | STATUS | R | Firmware version (currently 0x01) |

Positions are signed 16-bit integers (`int16_t`), stored little-endian (low byte first). Range: -32768 to +32767.

### Read Protocol

1. Master writes 1 byte: the register address to start reading from.
2. Master reads N bytes: data starting from that register, auto-incrementing.

```
Master TX: [I2C_ADDR+W] [reg_addr]
Master RX: [I2C_ADDR+R] [byte0] [byte1] ...
```

**Example — Read both encoder positions (4 bytes from register 0x00):**

```c
Wire.beginTransmission(0x40);
Wire.write(0x00);              // set register pointer to 0x00
Wire.endTransmission(false);   // repeated start
Wire.requestFrom(0x40, 4);     // read 4 bytes

int16_t enc1 = Wire.read() | (Wire.read() << 8);
int16_t enc2 = Wire.read() | (Wire.read() << 8);
```

**Example — Read button states (1 byte from register 0x04):**

```c
Wire.beginTransmission(0x40);
Wire.write(0x04);
Wire.endTransmission(false);
Wire.requestFrom(0x40, 1);

uint8_t buttons = Wire.read();
bool enc1_pressed = buttons & 0x01;
bool enc2_pressed = buttons & 0x02;
```

### Write Protocol (Commands)

Write 2 bytes: `[0x10, command]` to the slave.

| Command | Effect |
|---------|--------|
| 0x01 | Reset encoder 1 position to zero |
| 0x02 | Reset encoder 2 position to zero |
| 0x03 | Reset both encoder positions to zero |

```c
Wire.beginTransmission(0x40);
Wire.write(0x10);   // command register
Wire.write(0x03);   // reset both encoders
Wire.endTransmission();
```

## Encoder Resolution

Each encoder detent produces **4 raw quadrature transitions**. The I2C register reports these raw counts for maximum resolution:

| Value | Meaning |
|-------|---------|
| raw position | 4 counts per detent |
| raw / 2 | 2 counts per detent (half-step) |
| raw / 4 | 1 count per detent |

Divide on the host side to get the resolution you need.

## How It Works

### Timer-Based Polling

Instead of pin-change interrupts (which are unreliable with noisy mechanical encoders), a hardware timer (TCB0) fires at **10kHz** (every 0.1ms). Each ISR invocation reads both encoders' pin states, runs them through a debouncer (3 consecutive identical samples required), and updates the position using a Gray code state machine.

This gives:
- **Natural debouncing** without external components
- **No missed ticks** regardless of how busy the main loop is
- **Deterministic sampling** independent of application code

### Quadrature State Machine

Valid Gray code transitions for clockwise rotation: `0→2→3→1→0`
Valid transitions for counter-clockwise: `0→1→3→2→0`

Any other transition (e.g., both channels changing simultaneously) is ignored as noise.

### I2C Architecture

- **TWI ISR** (Wire library) handles address matching, data send/receive
- **TCB0 ISR** polls encoder pins at 10kHz — independent of I2C activity
- **Main loop** accumulates encoder deltas into position counters and updates a shared register buffer atomically (interrupts disabled for ~6 byte copies)
- **Reset commands** received via I2C are deferred to the main loop via a flag, avoiding ISR-context access to encoder internals

### Interrupt Safety

The register buffer is written by the main loop inside a `noInterrupts()` / `interrupts()` block, ensuring the I2C callback always reads a consistent snapshot. The TCB0 ISR only touches encoder-internal volatile state, which is consumed by `delta()` in the main loop.

## Configuration

All configurable values are `#define`s at the top of the `.ino` file:

| Define | Default | Description |
|--------|---------|-------------|
| `I2C_ADDRESS` | `0x40` | 7-bit I2C slave address |
| `ENC1_A` | `PIN_PA1` | Encoder 1 channel A pin |
| `ENC1_B` | `PIN_PA2` | Encoder 1 channel B pin |
| `ENC1_BTN` | `PIN_PA3` | Encoder 1 button pin |
| `ENC2_A` | `PIN_PA4` | Encoder 2 channel A pin |
| `ENC2_B` | `PIN_PA5` | Encoder 2 channel B pin |
| `ENC2_BTN` | `PIN_PA6` | Encoder 2 button pin |
| `STATUS_LED` | `PIN_PA7` | Heartbeat LED pin |
| `LED_ENABLED` | `true` | Set to `false` to disable LED |
| `HEARTBEAT_MS` | `1000` | Heartbeat toggle interval (ms) |

To change the sampling frequency, edit `INTERRUPT_PERIOD` in `Encoder.h`:

| INTERRUPT_PERIOD | Frequency | Period |
|------------------|-----------|--------|
| 12 | 20kHz | 0.05ms |
| **25** | **10kHz** | **0.1ms** |
| 50 | 5kHz | 0.2ms |
| 100 | 2.5kHz | 0.4ms |

## Compilation

- **Board package:** [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore) 2.6.x or later
- **Board:** ATtiny804
- **Clock:** 20MHz internal
- **Programmer:** SerialUPDI (or any UPDI programmer)

## File Structure

```
ATTiny804-double-encouder-IIC/
├── ATTiny804-double-encouder-IIC.ino   Main sketch
├── Encoder.h                           Encoder library header
├── Encoder.cpp                         Encoder library implementation
└── README.md                           This file
```

## Library Credit

The encoder algorithm is based on the **John-Lluch Encoder library** which uses timer-based polling instead of pin-change interrupts.

- **Original:** [John-Lluch/Encoder](https://github.com/John-Lluch/Encoder)
- **Author:** John Lluch

## Author

**Iman Mohammadi**
Email: iman.mohamadi.dev@gmail.com
