/*
 * ATtiny804 Triple Rotary Encoder I2C Slave
 *
 * Reads three quadrature rotary encoders using timer-based polling (10kHz)
 * and exposes their positions + button states over I2C as a slave device.
 * An interrupt output pin signals the host when encoder state changes.
 *
 * Hardware (14-pin SOIC):
 *   Pin 1  (VDD)  - Power
 *   Pin 2  (PA4)  - Encoder 2 Channel A
 *   Pin 3  (PA5)  - Encoder 2 Channel B
 *   Pin 4  (PA6)  - Encoder 2 Push Button
 *   Pin 5  (PA7)  - Encoder 3 Channel A
 *   Pin 6  (PB3)  - Encoder 3 Channel B
 *   Pin 7  (PB2)  - Interrupt Output (push-pull, active-low default)
 *   Pin 8  (PB1)  - I2C SDA
 *   Pin 9  (PB0)  - I2C SCL
 *   Pin 10 (PA0)  - UPDI (programming only)
 *   Pin 11 (PA1)  - Encoder 1 Channel A
 *   Pin 12 (PA2)  - Encoder 1 Channel B
 *   Pin 13 (PA3)  - Encoder 1 Push Button
 *   Pin 14 (GND)  - Ground
 *
 * I2C Slave Address: 0x40 (configurable below)
 *
 * Register Map (read by master):
 *   0x00  ENC1_POS_L   Encoder 1 position, low byte  (int16_t, little-endian)
 *   0x01  ENC1_POS_H   Encoder 1 position, high byte
 *   0x02  ENC2_POS_L   Encoder 2 position, low byte
 *   0x03  ENC2_POS_H   Encoder 2 position, high byte
 *   0x04  ENC3_POS_L   Encoder 3 position, low byte
 *   0x05  ENC3_POS_H   Encoder 3 position, high byte
 *   0x06  BUTTONS      Button states: bit0 = enc1, bit1 = enc2 (1 = pressed)
 *                       Encoder 3 has no button pin.
 *   0x07  STATUS       Device status / firmware version (0x02)
 *
 * Interrupt Output (PB2):
 *   Directly driven push-pull. Asserted when any encoder position changes
 *   (or button state changes). De-asserted automatically when the master
 *   performs an I2C read. Polarity is configurable:
 *     Active-low  (default): idle HIGH, asserted LOW
 *     Active-high:           idle LOW,  asserted HIGH
 *
 * I2C Protocol:
 *   Read:  Master writes 1 byte (register address), then reads N bytes.
 *          Register pointer auto-increments. Reading clears the interrupt.
 *   Write: Master writes [register, data]:
 *          [0x10, cmd]  - Reset commands (OR-able):
 *                         0x01 = reset encoder 1
 *                         0x02 = reset encoder 2
 *                         0x04 = reset encoder 3
 *                         0x07 = reset all three
 *          [0x11, cfg]  - Interrupt config:
 *                         bit 0: polarity (0 = active-low, 1 = active-high)
 *
 * Resolution: 4 counts per encoder detent (raw quadrature transitions).
 *             Use deltaTick2() for 1 count per detent.
 *
 * Host-Side Delta Tracking:
 *   Position registers hold a running counter (int16_t) that wraps at
 *   +/-32768. The host should compute movement as:
 *     int16_t delta = (int16_t)((uint16_t)current - (uint16_t)previous);
 *   Unsigned subtraction yields the correct signed delta even across
 *   wraparound, as long as |delta| < 32768 between polls.
 *
 * Based on the John-Lluch Encoder library timer-based polling approach.
 * Author: Iman Mohammadi (iman.mohamadi.dev@gmail.com)
 */

#include <Wire.h>
#include "Encoder.h"

// ============================================================================
// Configuration
// ============================================================================
#define I2C_ADDRESS     0x40

#define ENC1_A          PIN_PA1
#define ENC1_B          PIN_PA2
#define ENC1_BTN        PIN_PA3

#define ENC2_A          PIN_PA4
#define ENC2_B          PIN_PA5
#define ENC2_BTN        PIN_PA6

#define ENC3_A          PIN_PA7
#define ENC3_B          PIN_PB3

#define INT_PIN         PIN_PB2

// ============================================================================
// Register Map Definitions
// ============================================================================
#define REG_MAP_SIZE    8

#define REG_ENC1_POS_L  0x00
#define REG_ENC1_POS_H  0x01
#define REG_ENC2_POS_L  0x02
#define REG_ENC2_POS_H  0x03
#define REG_ENC3_POS_L  0x04
#define REG_ENC3_POS_H  0x05
#define REG_BUTTONS     0x06
#define REG_STATUS      0x07

#define REG_CMD         0x10
#define REG_INT_CFG     0x11

#define CMD_RESET_ENC1  0x01
#define CMD_RESET_ENC2  0x02
#define CMD_RESET_ENC3  0x04

#define FW_VERSION      0x03

// ============================================================================
// Encoder Objects (encoder 3 has no button — pass 255)
// ============================================================================
Encoder encoder1(ENC1_A, ENC1_B, ENC1_BTN);
Encoder encoder2(ENC2_A, ENC2_B, ENC2_BTN);
Encoder encoder3(ENC3_A, ENC3_B, 255);
Encoder *encoderArray[] = { &encoder1, &encoder2, &encoder3 };

// ============================================================================
// Shared State
// ============================================================================
volatile int16_t enc1Position = 0;
volatile int16_t enc2Position = 0;
volatile int16_t enc3Position = 0;

volatile uint8_t regPointer = 0;
volatile uint8_t regData[REG_MAP_SIZE];
volatile uint8_t pendingCmd = 0;

// Interrupt output state
volatile bool intAsserted  = false;
volatile bool intActiveHigh = false;  // default: active-low

static inline void writeIntPin()
{
  digitalWrite(INT_PIN, intAsserted == intActiveHigh ? HIGH : LOW);
}

// ============================================================================
// I2C Callbacks (called from TWI ISR context)
// ============================================================================
void onI2CRequest()
{
  uint8_t start = regPointer;
  for (uint8_t i = start; i < REG_MAP_SIZE; i++) {
    Wire.write(regData[i]);
  }

  intAsserted = false;
  writeIntPin();
}

void onI2CReceive(int numBytes)
{
  if (numBytes < 1) return;

  uint8_t reg = Wire.read();

  if (reg == REG_CMD && Wire.available()) {
    pendingCmd = Wire.read();
  } else if (reg == REG_INT_CFG && Wire.available()) {
    intActiveHigh = Wire.read() & 0x01;
    writeIntPin();
  } else {
    regPointer = (reg < REG_MAP_SIZE) ? reg : 0;
  }

  while (Wire.available()) Wire.read();
}

// ============================================================================
// Setup
// ============================================================================
void setup()
{
  pinMode(INT_PIN, OUTPUT);
  intAsserted = false;
  writeIntPin();

  regData[REG_STATUS] = FW_VERSION;

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);

  EncoderInterrupt.begin(encoderArray, 3);

  // Discard any spurious counts from power-up pin noise
  encoder1.deltaTick2();
  encoder2.deltaTick2();
  encoder3.deltaTick2();
}

// ============================================================================
// Main Loop
// ============================================================================
void loop()
{
  uint8_t cmd = pendingCmd;
  if (cmd) {
    pendingCmd = 0;
    if (cmd & CMD_RESET_ENC1) {
      encoder1.delta();
      enc1Position = 0;
    }
    if (cmd & CMD_RESET_ENC2) {
      encoder2.delta();
      enc2Position = 0;
    }
    if (cmd & CMD_RESET_ENC3) {
      encoder3.delta();
      enc3Position = 0;
    }
  }

  int16_t d1 = encoder1.deltaTick2();
  int16_t d2 = encoder2.deltaTick2();
  int16_t d3 = encoder3.deltaTick2();

  enc1Position += d1;
  enc2Position += d2;
  enc3Position += d3;

  uint8_t buttons = 0;
  if (encoder1.button()) buttons |= 0x01;
  if (encoder2.button()) buttons |= 0x02;

  static uint8_t prevButtons = 0;

  noInterrupts();
  regData[REG_ENC1_POS_L] = enc1Position & 0xFF;
  regData[REG_ENC1_POS_H] = (enc1Position >> 8) & 0xFF;
  regData[REG_ENC2_POS_L] = enc2Position & 0xFF;
  regData[REG_ENC2_POS_H] = (enc2Position >> 8) & 0xFF;
  regData[REG_ENC3_POS_L] = enc3Position & 0xFF;
  regData[REG_ENC3_POS_H] = (enc3Position >> 8) & 0xFF;
  regData[REG_BUTTONS]    = buttons;
  interrupts();

  if (d1 != 0 || d2 != 0 || d3 != 0 || buttons != prevButtons) {
    intAsserted = true;
    writeIntPin();
  }
  prevButtons = buttons;
}
