#ifndef SENSOR_REDUNDANCY_H
#define SENSOR_REDUNDANCY_H

#include <Arduino.h>
#include "config.h"
#include "sensors.h"

// Forward declarations - types are now defined in config.h
// Remove duplicate definitions to avoid redefinition errors

// Use the types from config.h:
// - FlightCapability
// - SyntheticSensorData 
// - SensorRedundancySystem

// SensorStatus is now defined in config.h to avoid circular dependencies

// Sensor cross-validation and synthetic data generation
class SensorValidation {
public:
  static bool validate_imu_data(const IMUData& data1, const IMUData& data2);
  static bool validate_gps_data(const GPSData& gps, const BarometerData& baro);
  static float calculate_synthetic_gps_confidence(const SensorData& sensors);
  static SyntheticSensorData generate_synthetic_data(const SensorData& sensors);
};

class SensorRedundancySystem {
public:
  SensorRedundancySystem();
  void init();
  void update(SensorData& sensors);
  void calculate_sensor_health();
  void update_synthetic_gps();
  void update_synthetic_magnetometer();
  void update_synthetic_barometer();
  void determine_flight_capability();
  void adjust_flight_characteristics();
  void handle_sensor_failures();
  bool is_flight_safe();
  bool is_arming_safe();
  String get_safety_report();
  String get_sensor_alerts();
  SensorStatus get_sensor_status();
  void enable_fallback(String sensor_type, bool enable);
  void set_quality_threshold(String sensor_type, float threshold);
  void force_synthetic_mode(String sensor_type, bool enable);
  FlightCapability get_flight_capability();
  FlightTuning get_adaptive_tuning();
  SyntheticSensorData get_synthetic_data();
  // ---------- NEW: synthetic sensor helpers ----------
  // These helpers are implemented in sensor_redundancy.cpp to provide fallback data when dedicated sensors fail.
  void update_synthetic_gps(const SensorData &sensors);
  void update_synthetic_magnetometer(const SensorData &sensors);
  void update_synthetic_barometer(const SensorData &sensors);
  // ---------------------------------------------------

private:
    FlightCapability current_capability;
    SensorStatus status;
    FlightTuning tuning;
    SensorRedundancyConfig config;
    SyntheticSensorData synthetic_data;
};

extern SensorRedundancySystem sensor_redundancy;

#endif // SENSOR_REDUNDANCY_H 