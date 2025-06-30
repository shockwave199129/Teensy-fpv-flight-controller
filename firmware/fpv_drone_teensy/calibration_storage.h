#pragma once

#include "config.h"
#include <Arduino.h>

namespace CalibrationStorage {
  bool save(const CalibrationData& data);
  bool load(CalibrationData& data);
} 