#pragma once
// Minimal native (host) stand-in for the Arduino core API, providing just
// enough surface for the pure logic in config.h / pid_controller.h /
// calibration_storage.cpp / safety_functions.cpp to compile and run under
// PlatformIO's "native" platform (see [env:native_test] in platformio.ini).
// This intentionally does NOT emulate real hardware timing, I/O, or serial -
// it is a compile/link shim for unit tests, not a simulator.

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

using std::size_t;

#ifndef PI
#define PI 3.14159265358979323846
#endif

// ---- constrain / map: identical semantics to the real Arduino macros ----
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
#ifndef map
#define map(x, in_min, in_max, out_min, out_max) \
  (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
#endif

using std::abs;
using std::sqrt;
using std::pow;
// The "f"-suffixed C99 float variants (fabsf/sqrtf/atan2f/sinf/cosf) are
// pulled in globally by <math.h> - not guaranteed to exist under std:: - so
// they're used unqualified, exactly as Arduino code does.

using std::max;
using std::min;

// Forward-declared only: several declared-but-unused-by-tests signatures
// (SensorRedundancySystem::get_safety_report(), etc.) mention Arduino's
// String type. None of those methods are exercised here, so a forward
// declaration is enough to let those headers parse without pulling in a
// full String implementation.
class String;

// ---- timing: tests control this explicitly via test_set_millis(), they
// don't rely on wall-clock time. `inline` variables (C++17) keep this
// header-only - no companion .cpp needed for the native test build. ----
inline unsigned long g_native_shim_millis = 0;
inline unsigned long millis() { return g_native_shim_millis; }
inline unsigned long micros() { return g_native_shim_millis * 1000UL; }
inline void test_set_millis(unsigned long ms) { g_native_shim_millis = ms; }

// ---- Serial: discarded output, just enough overloads to satisfy callers ----
struct NativeSerial {
  void begin(unsigned long) {}
  void print(const char*) {}
  void print(int) {}
  void print(unsigned int) {}
  void print(long) {}
  void print(unsigned long) {}
  void print(float, int = 2) {}
  void print(double, int = 2) {}
  void print(char) {}
  void println() {}
  void println(const char*) {}
  void println(int) {}
  void println(unsigned int) {}
  void println(long) {}
  void println(unsigned long) {}
  void println(float, int = 2) {}
  void println(double, int = 2) {}
  void println(char) {}
  bool available() { return false; }
  int read() { return -1; }
  explicit operator bool() const { return true; }
};
inline NativeSerial Serial;
inline NativeSerial Serial1;
inline NativeSerial Serial2;

// ---- GPIO / analog stubs: not used by the logic under test, but referenced
// transitively by some headers. ----
#define INPUT 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return LOW; }

// Tests that exercise threshold logic depending on analogRead() set this.
inline int g_native_shim_analog_value = 0;
inline int analogRead(int) { return g_native_shim_analog_value; }

inline void tone(int, int, int = 0) {}
inline void noTone(int) {}
