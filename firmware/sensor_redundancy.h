#ifndef SENSOR_REDUNDANCY_H
#define SENSOR_REDUNDANCY_H

#include "config.h"

// Sensor Priority Levels
enum SensorPriority {
  SENSOR_CRITICAL = 0,    // IMU - Must have for flight
  SENSOR_HIGH,            // Battery, RC - Required for safe operation
  SENSOR_MEDIUM,          // Barometer, Magnetometer - Degraded performance without
  SENSOR_LOW              // GPS, Sonar, Optical Flow - Optional features
};

// Sensor Health States
enum SensorHealth {
  SENSOR_HEALTHY = 0,
  SENSOR_DEGRADED,        // Working but unreliable
  SENSOR_FAILED,          // Not working
  SENSOR_MISSING          // Not detected
};

// Flight Mode Capability Levels
enum FlightCapability {
  FLIGHT_FULL_CAPABILITY = 0,     // All sensors working
  FLIGHT_DEGRADED_GPS,            // No GPS - use IMU/Baro/Mag for position estimation
  FLIGHT_DEGRADED_MAG,            // No Magnetometer - use GPS heading or gyro integration
  FLIGHT_DEGRADED_BARO,           // No Barometer - use GPS altitude or sonar
  FLIGHT_MINIMAL,                 // Only IMU + RC - basic stabilization only
  FLIGHT_EMERGENCY                // Critical sensor failure - emergency landing mode
};

// Sensor Redundancy Configuration
struct SensorRedundancyConfig {
  // Fallback strategies
  bool enable_imu_mag_fallback;           // Use IMU compass heading if magnetometer fails
  bool enable_gps_baro_fallback;          // Use GPS altitude if barometer fails
  bool enable_sonar_baro_fallback;        // Use sonar for low altitude if barometer fails
  bool enable_optical_flow_gps_fallback;  // Use optical flow if GPS fails
  bool enable_synthetic_gps;              // Generate synthetic GPS from IMU/Mag/Baro
  
  // Quality thresholds
  float imu_health_threshold;             // Minimum IMU quality (0-1)
  float gps_accuracy_threshold;           // Maximum GPS error (meters)
  float mag_deviation_threshold;          // Maximum mag deviation (degrees)
  float baro_noise_threshold;             // Maximum barometer noise (Pa)
  
  // Redundancy timeouts
  unsigned long sensor_timeout_ms;        // Sensor considered failed after this time
  unsigned long fallback_switch_delay_ms; // Delay before switching to fallback
  unsigned long recovery_verification_ms; // Time to verify sensor recovery
};

// Sensor Status Tracking
struct SensorStatus {
  SensorHealth imu_health;
  SensorHealth gps_health;
  SensorHealth mag_health;
  SensorHealth baro_health;
  SensorHealth sonar_health;
  SensorHealth optical_flow_health;
  SensorHealth battery_health;
  SensorHealth rc_health;
  
  // Failure counters
  uint16_t imu_failures;
  uint16_t gps_failures;
  uint16_t mag_failures;
  uint16_t baro_failures;
  uint16_t sonar_failures;
  uint16_t optical_flow_failures;
  
  // Last update times
  unsigned long imu_last_update;
  unsigned long gps_last_update;
  unsigned long mag_last_update;
  unsigned long baro_last_update;
  unsigned long sonar_last_update;
  unsigned long optical_flow_last_update;
  
  // Quality metrics
  float imu_quality;        // 0-1
  float gps_accuracy;       // meters
  float mag_calibration;    // 0-1
  float baro_stability;     // 0-1
};

// Synthetic Sensor Data
struct SyntheticSensorData {
  // Synthetic GPS from IMU/Mag/Baro
  float synthetic_lat, synthetic_lon;
  float synthetic_altitude;
  float synthetic_speed;
  float synthetic_heading;
  bool synthetic_gps_valid;
  
  // Synthetic magnetometer from GPS/IMU
  float synthetic_mag_heading;
  bool synthetic_mag_valid;
  
  // Synthetic barometer from GPS/Sonar
  float synthetic_baro_altitude;
  bool synthetic_baro_valid;
  
  // Confidence levels
  float gps_confidence;     // 0-1
  float mag_confidence;     // 0-1
  float baro_confidence;    // 0-1
};

// Flight Characteristic Improvements
struct FlightTuning {
  // Adaptive PID gains based on sensor availability
  float stability_gain_multiplier;      // Increase gains when sensors missing
  float aggressiveness_reduction;       // Reduce response when degraded
  float smoothing_factor;               // Increase smoothing when noisy
  
  // Dynamic filtering
  bool enable_adaptive_filtering;
  float vibration_compensation;
  float noise_reduction_level;
  
  // Emergency modes
  bool emergency_stabilization_mode;
  float emergency_descent_rate;
  float emergency_landing_sensitivity;
};

// Global redundancy system
class SensorRedundancySystem {
private:
  SensorRedundancyConfig config;
  SensorStatus status;
  SyntheticSensorData synthetic;
  FlightTuning tuning;
  FlightCapability current_capability;
  
  // Internal state
  unsigned long last_update;
  bool system_initialized;
  String last_alert_message;
  unsigned long alert_timestamp;
  
  // Fallback methods
  void update_synthetic_gps();
  void update_synthetic_magnetometer();
  void update_synthetic_barometer();
  void calculate_sensor_health();
  void determine_flight_capability();
  void adjust_flight_characteristics();
  void handle_sensor_failures();
  
public:
  void init();
  void update(SensorData& sensors);
  FlightCapability get_flight_capability() { return current_capability; }
  SensorStatus get_sensor_status() { return status; }
  SyntheticSensorData get_synthetic_data() { return synthetic; }
  
  // Critical sensor checks
  bool is_flight_safe();
  bool is_arming_safe();
  String get_safety_report();
  String get_sensor_alerts();
  
  // Redundancy controls
  void enable_fallback(String sensor_type, bool enable);
  void set_quality_threshold(String sensor_type, float threshold);
  void force_synthetic_mode(String sensor_type, bool enable);
  void reset_sensor_failures();
  
  // Flight tuning
  FlightTuning get_adaptive_tuning();
  void apply_emergency_mode();
  void recovery_mode();
};

// External references
extern SensorData sensor_data;
extern SensorRedundancySystem sensor_redundancy;

#endif // SENSOR_REDUNDANCY_H 