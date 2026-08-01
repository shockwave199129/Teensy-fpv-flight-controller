#pragma once
// Minimal native stand-in for the Teensy/Arduino EEPROM library, backed by
// an in-memory byte array instead of real non-volatile storage. Lets
// calibration_storage.cpp's save()/load() round-trip be unit tested without
// hardware. Size matches Teensy 4.1's emulated EEPROM (4284 bytes).

#include <cstdint>
#include <cstddef>
#include <cstring>

class NativeEEPROMClass {
public:
  static const size_t kSize = 4284;

  uint8_t read(int address) {
    return (address >= 0 && (size_t)address < kSize) ? data_[address] : 0;
  }

  void write(int address, uint8_t value) {
    if (address >= 0 && (size_t)address < kSize) data_[address] = value;
  }

  template <typename T>
  T& get(int address, T& t) {
    if (address >= 0 && (size_t)address + sizeof(T) <= kSize) {
      memcpy(&t, &data_[address], sizeof(T));
    }
    return t;
  }

  template <typename T>
  const T& put(int address, const T& t) {
    if (address >= 0 && (size_t)address + sizeof(T) <= kSize) {
      memcpy(&data_[address], &t, sizeof(T));
    }
    return t;
  }

  void test_clear() { memset(data_, 0, kSize); }

private:
  uint8_t data_[kSize] = {0};
};

inline NativeEEPROMClass EEPROM;
