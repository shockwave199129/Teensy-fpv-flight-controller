#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <Arduino.h>
#include "config.h"

class OpticalFlowSensor {
private:
  OpticalFlowData flow_data;
  int sensor_type;
  bool initialized;
  
  // Sensor-specific initialization methods
  bool init_pmw3901();
  bool init_paa5100();
  bool init_adns3080();
  
  // Sensor-specific data reading methods
  bool read_pmw3901_data();
  bool read_paa5100_data();
  bool read_adns3080_data();
  
  // SPI communication methods
  uint8_t read_register(uint8_t reg);
  void write_register(uint8_t reg, uint8_t value);
  
public:
  OpticalFlowSensor();
  void init(int type);
  void update();
  OpticalFlowData get_data();
  bool is_healthy();
};

#endif // OPTICAL_FLOW_H 