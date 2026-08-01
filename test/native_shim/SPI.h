#pragma once
// Minimal native stand-in for the Arduino SPI library - just enough for
// sensors.h to parse under the native test environment. No real SPI I/O.

#include <cstdint>

class SPIClass {
public:
  void begin() {}
  uint8_t transfer(uint8_t) { return 0; }
};

inline SPIClass SPI;
