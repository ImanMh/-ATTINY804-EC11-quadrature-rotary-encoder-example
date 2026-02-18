/*
 * Timer-Based Rotary Encoder Library
 * 
 * High-frequency timer interrupt polling for reliable quadrature encoder reading.
 * Uses TCB0 on megaTinyCore (tinyAVR 0/1/2-series) or Timer2/3 on classic AVR.
 *
 * Based on the John-Lluch Encoder library:
 *   https://github.com/John-Lluch/Encoder
 *
 * Adapted for ATtiny804 dual-encoder I2C slave by Iman Mohammadi.
 */

#ifndef Encoder_H
#define Encoder_H

#include <Arduino.h>

#define DEBOUNCE_COUNT    3
#define INTERRUPT_PERIOD  25  // 10kHz on 16MHz/64 reference clock
                              //   25 -> 10kHz (0.1ms) - default
                              //   50 ->  5kHz (0.2ms)
                              //   12 -> 20kHz (0.05ms)

class Debouncer {
public:
  Debouncer() : _keepValue(0), _bounce(0) {}
  bool isDebounced(byte value, byte debounceCount);

private:
  byte _keepValue;
  byte _bounce;
};

class Encoder {
public:
  Encoder(byte pinA, byte pinB, byte pinP)
    : _pinA(pinA), _pinB(pinB), _pinP(pinP),
      _encoderValueVL(0), _encoderTickVL(0), _encoderTick2VL(0),
      _encoderButtonVL(false),
      _encoderValue(0), _encoderTick(0), _encoderTick2(0) {}

  int delta();
  int deltaTick();
  int deltaTick2();
  bool button() const { return _encoderButtonVL; }

private:
  void init();
  void compute();

  byte _pinA, _pinB, _pinP;

  volatile unsigned int _encoderValueVL;
  volatile unsigned int _encoderTickVL;
  volatile unsigned int _encoderTick2VL;
  volatile bool _encoderButtonVL;
  volatile byte _encoderStVL;

  unsigned int _encoderValue;
  unsigned int _encoderTick;
  unsigned int _encoderTick2;

  Debouncer stDebouncer, pDebouncer;

  friend class EncoderInterruptClass;
};

class EncoderInterruptClass {
public:
  EncoderInterruptClass() : _numEncoders(0), _encoderArray(nullptr), _singleEncoder(nullptr) {}

  void begin(Encoder **encoderRefs, int numEncoders);
  void begin(Encoder *encoder);

private:
  int _numEncoders;
  Encoder **_encoderArray;
  Encoder *_singleEncoder;
  void computeAll();

  friend void computeEncoder();
};

extern EncoderInterruptClass EncoderInterrupt;

#endif
