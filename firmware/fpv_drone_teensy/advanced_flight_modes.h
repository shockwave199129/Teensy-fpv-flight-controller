#ifndef ADVANCED_FLIGHT_MODES_H
#define ADVANCED_FLIGHT_MODES_H

#include "config.h"

class AdvancedFlightModes {
private:
  AdvancedFlightMode current_mode;
  bool mode_active;
  unsigned long mode_entry_time;
  
  // Mode-specific parameters
  float acro_plus_recovery_angle;
  float sport_rate_multiplier;
  float cinematic_expo_factor;
  float gps_rescue_climb_rate;
  float turtle_throttle_limit;
  
  // Mode state flags
  bool recovery_mode_active;
  bool launch_detected;
  bool landing_sequence_active;
  
  // Private methods for each mode
  void check_mode_activation(const RcData& rc);
  void update_acro_plus_mode(SensorData& sensors, RcData& rc, FlightState& state);
  void update_sport_mode(SensorData& sensors, RcData& rc, FlightState& state);
  void update_cinematic_mode(SensorData& sensors, RcData& rc, FlightState& state);
  void update_gps_rescue_mode(SensorData& sensors, RcData& rc, FlightState& state);
  void update_turtle_mode(SensorData& sensors, RcData& rc, FlightState& state);
  void update_launch_assist_mode(SensorData& sensors, RcData& rc, FlightState& state);
  void update_land_assist_mode(SensorData& sensors, RcData& rc, FlightState& state);
  
public:
  AdvancedFlightModes();
  void init();
  void update(SensorData& sensors, RcData& rc, FlightState& state);
  void set_mode(AdvancedFlightMode mode);
  AdvancedFlightMode get_mode();
};

#endif // ADVANCED_FLIGHT_MODES_H 