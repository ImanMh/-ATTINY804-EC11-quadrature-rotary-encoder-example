/*
 * Timer-Based Rotary Encoder Library - Implementation
 *
 * Based on the John-Lluch Encoder library:
 *   https://github.com/John-Lluch/Encoder
 *
 * Adapted for ATtiny804 dual-encoder I2C slave by Iman Mohammadi.
 */

#include <Arduino.h>
#include "Encoder.h"

EncoderInterruptClass EncoderInterrupt;
void computeEncoder();

// ============================================================================
// Platform-Specific Timer Setup
// ============================================================================

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

void setupTimerInterrupt()
{
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  OCR3AH = 0;
  OCR3AL = INTERRUPT_PERIOD;
  TIMSK3 = (1 << OCIE3A);
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS30) | (1 << CS31);
}

SIGNAL(TIMER3_COMPA_vect)
{
  computeEncoder();
}

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

void setupTimerInterrupt()
{
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = INTERRUPT_PERIOD;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);
}

SIGNAL(TIMER2_COMPA_vect)
{
  computeEncoder();
}

#elif defined(MEGATINYCORE)
// All tinyAVR 0/1/2-series: ATtiny202, ATtiny804, ATtiny1604, etc.
// Uses TCB0 in periodic interrupt mode. TCA0 is reserved for millis().
//
// Timer clock = F_CPU / 2 (CLK_PER/2 prescaler)
// INTERRUPT_PERIOD is defined for a 16MHz/64 = 250kHz reference clock.
// Conversion: timerPeriod = INTERRUPT_PERIOD * (F_CPU / 500000)
//   At 20MHz: 25 * 40 = 1000 -> 10kHz
//   At 16MHz: 25 * 32 =  800 -> 10kHz

void setupTimerInterrupt()
{
  uint16_t period = (uint16_t)((uint32_t)INTERRUPT_PERIOD * (F_CPU / 500000UL));

  TCB0.CTRLA = 0;
  TCB0.CTRLB = 0;
  TCB0.CCMP  = period - 1;
  TCB0.CNT   = 0;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
}

ISR(TCB0_INT_vect)
{
  computeEncoder();
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

#else
#error "Unsupported board. Supported: ATmega328P/168, ATmega32U4/16U4/1280/2560, megaTinyCore tinyAVR."
#endif

// ============================================================================
// Encoder Core Algorithm
// ============================================================================

void Encoder::init()
{
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  if (_pinP != 255) {
    pinMode(_pinP, INPUT_PULLUP);
  }

  byte eA = !digitalRead(_pinA);
  byte eB = !digitalRead(_pinB);
  _encoderStVL = eB << 1 | eA;
}

void Encoder::compute()
{
  byte eA = !digitalRead(_pinA);
  byte eB = !digitalRead(_pinB);
  bool pb = (_pinP != 255) ? !digitalRead(_pinP) : false;

  byte st = ((byte)eB << 1) | (byte)eA;

  if (stDebouncer.isDebounced(st, DEBOUNCE_COUNT))
  {
    byte encoderSt = _encoderStVL;
    if (encoderSt != st)
    {
      bool wentDown = ((encoderSt == 0 && st == 2) ||
                       (encoderSt == 2 && st == 3) ||
                       (encoderSt == 3 && st == 1) ||
                       (encoderSt == 1 && st == 0));

      if (wentDown) _encoderValueVL--;
      else          _encoderValueVL++;

      _encoderTickVL  = _encoderValueVL / 2;
      _encoderTick2VL = _encoderTickVL / 2;

      _encoderStVL = st;
    }
  }

  if (_pinP != 255 && pDebouncer.isDebounced(pb, DEBOUNCE_COUNT) && _encoderButtonVL != pb)
  {
    _encoderButtonVL = pb;
  }
}

int Encoder::delta()
{
  noInterrupts();
  unsigned int v = _encoderValueVL;
  interrupts();
  int d = v - _encoderValue;
  _encoderValue = v;
  return d;
}

int Encoder::deltaTick()
{
  noInterrupts();
  unsigned int v = _encoderTickVL;
  interrupts();
  int d = v - _encoderTick;
  _encoderTick = v;
  return d;
}

int Encoder::deltaTick2()
{
  noInterrupts();
  unsigned int v = _encoderTick2VL;
  interrupts();
  int d = v - _encoderTick2;
  _encoderTick2 = v;
  return d;
}

// ============================================================================
// Interrupt Management
// ============================================================================

void EncoderInterruptClass::begin(Encoder **encoderRefs, int numEncoders)
{
  _encoderArray = encoderRefs;
  _numEncoders = numEncoders;
  for (int i = 0; i < _numEncoders; i++) {
    _encoderArray[i]->init();
  }
  setupTimerInterrupt();
}

void EncoderInterruptClass::begin(Encoder *encoder)
{
  _singleEncoder = encoder;
  begin(&_singleEncoder, 1);
}

void EncoderInterruptClass::computeAll()
{
  for (int i = 0; i < _numEncoders; i++) {
    _encoderArray[i]->compute();
  }
}

void computeEncoder()
{
  EncoderInterrupt.computeAll();
}

bool Debouncer::isDebounced(byte value, byte debounceCount)
{
  if (_keepValue != value) {
    _bounce = debounceCount;
    _keepValue = value;
  } else if (_bounce > 0) {
    _bounce--;
  }
  return _bounce == 0;
}
