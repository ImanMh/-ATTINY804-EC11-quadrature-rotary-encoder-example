/*
 * ATtiny804 Dual Rotary Encoder I2C Slave
 *
 * Reads two quadrature rotary encoders using timer-based polling (10kHz)
 * and exposes their positions + button states over I2C as a slave device.
 *
 * Hardware (14-pin SOIC):
 *   Pin 1  (VDD)  - Power
 *   Pin 2  (PA4)  - Encoder 2 Channel A
 *   Pin 3  (PA5)  - Encoder 2 Channel B
 *   Pin 4  (PA6)  - Encoder 2 Push Button
 *   Pin 5  (PA7)  - Status LED (optional heartbeat)
 *   Pin 6  (PB3)  - Free
 *   Pin 7  (PB2)  - Free
 *   Pin 8  (PB1)  - I2C SDA
 *   Pin 9  (PB0)  - I2C SCL
 *   Pin 10 (PA0)  - UPDI (programming)
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
 *   0x02  ENC2_POS_L   Encoder 2 position, low byte  (int16_t, little-endian)
 *   0x03  ENC2_POS_H   Encoder 2 position, high byte
 *   0x04  BUTTONS      Button states: bit0 = enc1, bit1 = enc2 (1 = pressed)
 *   0x05  STATUS       Device status / firmware version (0x01)
 *
 * I2C Protocol:
 *   Read:  Master writes 1 byte (register address), then reads N bytes.
 *          Register pointer auto-increments on each byte read.
 *   Write: Master writes [0x10, command_byte] to issue commands:
 *          0x01 = reset encoder 1 position to zero
 *          0x02 = reset encoder 2 position to zero
 *          0x03 = reset both encoder positions to zero
 *
 * Resolution: 4 counts per encoder detent (raw quadrature transitions).
 *             The quadrature decoder tracks all 4 state transitions per
 *             detent for maximum resolution. Use deltaTick2() in the
 *             Encoder library for 1 count per detent if preferred.
 *
 * Host-Side Delta Tracking:
 *   Position registers hold a running counter (int16_t) that wraps at
 *   ±32768. The host should compute movement as:
 *     int16_t delta = (int16_t)((uint16_t)current - (uint16_t)previous);
 *   Unsigned subtraction yields the correct signed delta even across
 *   wraparound, as long as |delta| < 32768 between polls — easily
 *   satisfied at any reasonable poll rate for human-operated encoders.
 *   This eliminates the need for reset commands to avoid overflow.
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

#define STATUS_LED      PIN_PA7
#define LED_ENABLED     true
#define HEARTBEAT_MS    1000

// ============================================================================
// Register Map Definitions
// ============================================================================
#define REG_MAP_SIZE    6

#define REG_ENC1_POS_L  0x00
#define REG_ENC1_POS_H  0x01
#define REG_ENC2_POS_L  0x02
#define REG_ENC2_POS_H  0x03
#define REG_BUTTONS     0x04
#define REG_STATUS      0x05

#define REG_CMD         0x10
#define CMD_RESET_ENC1  0x01
#define CMD_RESET_ENC2  0x02

#define FW_VERSION      0x01

// ============================================================================
// Encoder Objects
// ============================================================================
Encoder encoder1(ENC1_A, ENC1_B, ENC1_BTN);
Encoder encoder2(ENC2_A, ENC2_B, ENC2_BTN);
Encoder *encoderArray[] = { &encoder1, &encoder2 };

// ============================================================================
// Shared State
// ============================================================================
volatile int16_t enc1Position = 0;
volatile int16_t enc2Position = 0;

volatile uint8_t regPointer = 0;
volatile uint8_t regData[REG_MAP_SIZE];
volatile uint8_t pendingCmd = 0;

// ============================================================================
// I2C Callbacks (called from TWI ISR context)
// ============================================================================
void onI2CRequest()
{
  uint8_t start = regPointer;
  for (uint8_t i = start; i < REG_MAP_SIZE; i++) {
    Wire.write(regData[i]);
  }
}

void onI2CReceive(int numBytes)
{
  if (numBytes < 1) return;

  uint8_t reg = Wire.read();

  if (reg == REG_CMD && Wire.available()) {
    pendingCmd = Wire.read();
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
  if (LED_ENABLED) {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
  }

  regData[REG_STATUS] = FW_VERSION;

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);

  EncoderInterrupt.begin(encoderArray, 2);
}

// ============================================================================
// Main Loop
// ============================================================================
void loop()
{
  // Handle pending reset commands (set by I2C write, processed here to
  // avoid accessing encoder internals from ISR context)
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
  }

  enc1Position += encoder1.deltaTick2();
  enc2Position += encoder2.deltaTick2();

  uint8_t buttons = 0;
  if (encoder1.button()) buttons |= 0x01;
  if (encoder2.button()) buttons |= 0x02;

  noInterrupts();
  regData[REG_ENC1_POS_L] = enc1Position & 0xFF;
  regData[REG_ENC1_POS_H] = (enc1Position >> 8) & 0xFF;
  regData[REG_ENC2_POS_L] = enc2Position & 0xFF;
  regData[REG_ENC2_POS_H] = (enc2Position >> 8) & 0xFF;
  regData[REG_BUTTONS]    = buttons;
  interrupts();

  // Heartbeat LED
  if (LED_ENABLED) {
    static uint32_t lastBlink = 0;
    uint32_t now = millis();
    if (now - lastBlink >= HEARTBEAT_MS) {
      lastBlink = now;
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
  }
}
