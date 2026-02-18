/*
 * Teensy 4.0 I2C Host — ATtiny804 Dual Encoder Reader
 *
 * Reads two encoder positions and button states from the ATtiny804 I2C slave
 * and prints deltas to Serial.
 *
 * Uses unsigned subtraction to compute deltas from the slave's running
 * counter, which correctly handles int16_t wraparound without needing
 * reset commands.
 *
 * Connections:
 *   Teensy Pin 18 (SDA) <-> ATtiny804 Pin 8 (PB1/SDA) <-> 4.7kΩ pull-up to 3.3V
 *   Teensy Pin 19 (SCL) <-> ATtiny804 Pin 9 (PB0/SCL) <-> 4.7kΩ pull-up to 3.3V
 *   GND <-> GND
 *   3.3V <-> VDD (ATtiny804)
 *
 * NOTE: Teensy 4.0 is 3.3V logic — use 3.3V pull-ups, not 5V.
 */

#include <Wire.h>

#define SLAVE_ADDR      0x40
#define REG_ENC1_POS_L  0x00
#define REG_BUTTONS     0x04
#define REG_STATUS      0x05
#define POLL_INTERVAL   50

uint16_t rawEnc1 = 0;
uint16_t rawEnc2 = 0;
uint8_t  buttons = 0;

uint16_t prevEnc1 = 0;
uint16_t prevEnc2 = 0;

bool readRegisters(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom((uint8_t)SLAVE_ADDR, len);
  if (received != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool readEncoders() {
  uint8_t buf[5];
  if (!readRegisters(REG_ENC1_POS_L, buf, 5)) return false;

  rawEnc1 = (uint16_t)(buf[0] | (buf[1] << 8));
  rawEnc2 = (uint16_t)(buf[2] | (buf[3] << 8));
  buttons = buf[4];
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  delay(100);

  uint8_t status;
  if (readRegisters(REG_STATUS, &status, 1)) {
    Serial.print("ATtiny804 encoder slave found, firmware v");
    Serial.println(status);
  } else {
    Serial.println("ERROR: no response from slave at 0x40");
  }

  readEncoders();
  prevEnc1 = rawEnc1;
  prevEnc2 = rawEnc2;
}

void loop() {
  if (readEncoders()) {
    int16_t delta1 = (int16_t)(rawEnc1 - prevEnc1);
    int16_t delta2 = (int16_t)(rawEnc2 - prevEnc2);
    prevEnc1 = rawEnc1;
    prevEnc2 = rawEnc2;

    if (delta1 != 0 || delta2 != 0 || buttons != 0) {
      Serial.print("dE1: ");
      Serial.print(delta1);
      Serial.print("  dE2: ");
      Serial.print(delta2);
      Serial.print("  BTN: ");
      Serial.print(buttons & 0x01 ? "1" : "0");
      Serial.print(",");
      Serial.println(buttons & 0x02 ? "1" : "0");
    }
  } else {
    Serial.println("I2C read failed");
  }

  delay(POLL_INTERVAL);
}
