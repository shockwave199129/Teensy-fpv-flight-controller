#pragma once
// Minimal native stand-in for the Arduino Wire (I2C) library - just enough
// for sensors.h/sensor_redundancy.h to parse under the native test
// environment. No real I2C transactions.

#include <cstdint>

class TwoWire {
public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {}
  uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
  size_t write(uint8_t) { return 0; }
  int available() { return 0; }
  int read() { return -1; }
};

inline TwoWire Wire;
