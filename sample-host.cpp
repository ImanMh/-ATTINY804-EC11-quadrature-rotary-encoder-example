/*
* Teensy 4.0 I2C Master - Read Encoder Counter from ATTiny202
 *
 * Connections:
 * - Teensy Pin 18 (SDA) -> ATTiny202 physical pin 5 (PA2/SDA) -> 5.1kΩ -> 3.3V
 * - Teensy Pin 19 (SCL) -> ATTiny202 physical pin 4 (PA1/SCL) -> 5.1kΩ -> 3.3V
 * - GND -> GND
 * - 3.3V -> VCC (ATTiny202)
 *
 * NOTE: Use 3.3V pull-ups, not 5V (Teensy 4.0 is 3.3V logic)
 */

#include <Wire.h>

#define SLAVE_ADDRESS  0x40

void setup() {
    Serial.begin(115200);
    Serial.print("... ");
}

void loop() {
    Wire.requestFrom(SLAVE_ADDRESS, 4);

    if (Wire.available() >= 4) {
        uint8_t bytes[4];
        Wire.readBytes(bytes, 4);

        // Reconstruct int32_t from bytes (little-endian)
        int32_t counter = (int32_t)((uint32_t)bytes[0] |
                                    ((uint32_t)bytes[1] << 8) |
                                    ((uint32_t)bytes[2] << 16) |
                                    ((uint32_t)bytes[3] << 24));

        Serial.println(counter);
    } else {
        Serial.println('nothing');
    }

    delay(100);
}
