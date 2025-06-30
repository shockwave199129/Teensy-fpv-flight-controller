#ifndef FLIGHT_MODES_H
#define FLIGHT_MODES_H

#include "config.h"

class FlightModes {
private:
  FlightMode current_mode;
  FlightMode previous_mode;
  unsigned long mode_change_time;
  
  // Home position for RTH
  float home_latitude;
  float home_longitude;
  float home_altitude;
  bool home_set;
  
public:
  void init();
  void update(RcData& rc_data, SensorData& sensor_data);
  void check_mode_switch(RcData& rc_data, SensorData& sensor_data);
  FlightMode get_current_mode() { return current_mode; }
  void set_home_position(float lat, float lon, float alt);
  void return_to_home(SensorData& sensor_data, RcData& rc_data);
};

#endif // FLIGHT_MODES_H
