#pragma once
// Minimal native stand-in for FastLED - just enough for LEDControl
// (led_control.h) to declare `CRGB leds[NUM_LEDS];` and for callers to pass
// CRGB::Red/Orange/etc. constants, compiling under the native test
// environment. No real LED output.

#include <cstdint>

struct CRGB {
  uint8_t r = 0, g = 0, b = 0;
  CRGB() = default;
  CRGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}

  static const CRGB Black;
  static const CRGB Red;
  static const CRGB Orange;
  static const CRGB Yellow;
  static const CRGB Green;
  static const CRGB Blue;
  static const CRGB White;
};

inline const CRGB CRGB::Black  = CRGB(0, 0, 0);
inline const CRGB CRGB::Red    = CRGB(255, 0, 0);
inline const CRGB CRGB::Orange = CRGB(255, 165, 0);
inline const CRGB CRGB::Yellow = CRGB(255, 255, 0);
inline const CRGB CRGB::Green  = CRGB(0, 255, 0);
inline const CRGB CRGB::Blue   = CRGB(0, 0, 255);
inline const CRGB CRGB::White  = CRGB(255, 255, 255);
