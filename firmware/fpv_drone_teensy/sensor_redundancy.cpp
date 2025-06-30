#include "sensor_redundancy.h"
#include "sensors.h"
#include "config.h"
#include "optical_flow.h"
#include <math.h>  // NEW for cos()
#include "ekf.h"

// Global instance (since it's referenced in main.ino)
SensorRedundancySystem sensor_redundancy;

// External global optical flow instance declared in main .ino
extern OpticalFlowSensor optical_flow;

// Convenience constants
#define ONE_DEG_LAT_M 111320.0f  // metres per degree of latitude (approx.)

SensorRedundancySystem::SensorRedundancySystem() {
  // Initialize with conservative defaults
  current_capability = FLIGHT_CAPABILITY_ENHANCED;
  
  // Initialize status
  status.imu_health = SENSOR_HEALTHY;
  status.gps_health = SENSOR_UNAVAILABLE;
  status.mag_health = SENSOR_UNAVAILABLE;
  status.baro_health = SENSOR_UNAVAILABLE;
  status.quality_score = 1.0f;
  status.redundancy_active = false;
  status.last_update = 0;
  
  // Initialize old boolean fields for compatibility
  status.primary_imu_ok = true;
  status.secondary_imu_ok = false;
  status.gps_ok = false;
  status.mag_ok = false;
  status.baro_ok = false;
  status.synthetic_gps_available = false;
  status.synthetic_mag_available = false;
  status.synthetic_baro_available = false;
  
  // Initialize tuning with defaults
  tuning.stability_gain_multiplier = 1.0f;
  tuning.aggressiveness_factor = 1.0f;
  tuning.response_time = 1.0f;
  
  // Initialize config with defaults  
  config.dual_imu_enabled = false;
  config.synthetic_sensors_enabled = true;
  config.failover_threshold = 0.5f;
  config.validation_timeout_ms = 1000;
  config.auto_failover_enabled = true;
}

void SensorRedundancySystem::init() {
  Serial.println("Sensor Redundancy System initialized");
}

void SensorRedundancySystem::update(SensorData& sensors) {
  // Existing basic status update
  status.primary_imu_ok = sensors.imu.healthy;
  status.gps_ok = sensors.gps.healthy && sensors.gps.fix;
  status.mag_ok = sensors.mag.healthy;
  status.baro_ok = sensors.baro.healthy;

  status.imu_health = sensors.imu.healthy ? SENSOR_HEALTHY : SENSOR_FAILED;
  status.gps_health = status.gps_ok ? SENSOR_HEALTHY : SENSOR_UNAVAILABLE;
  status.mag_health = status.mag_ok ? SENSOR_HEALTHY : SENSOR_UNAVAILABLE;
  status.baro_health = status.baro_ok ? SENSOR_HEALTHY : SENSOR_UNAVAILABLE;

  // -------- NEW: generate synthetic sensors where required --------
  update_synthetic_gps(sensors);
  update_synthetic_magnetometer(sensors);
  update_synthetic_barometer(sensors);
  // ---------------------------------------------------------------

  calculate_sensor_health();
  determine_flight_capability();
  adjust_flight_characteristics();

  // EKF predict step using accelerometer (convert body to NED simplistically)
  EKF::predict(sensors.imu.accel_x, sensors.imu.accel_y, 0.0005f); // dt placeholder

  // GPS update
  if (sensors.gps.healthy && sensors.gps.fix) {
      EKF::update_gps(sensors.gps.latitude, sensors.gps.longitude, sensors.gps.speed*cosf(sensors.gps.heading*PI/180.0f), sensors.gps.speed*sinf(sensors.gps.heading*PI/180.0f));
  }

  // Optical flow velocity update
  if (optical_flow.get_data().data_valid) {
      EKF::update_optflow(optical_flow.get_data().velocity_x, optical_flow.get_data().velocity_y);
  }

  float fLat,fLon,fVn,fVe; EKF::get_state(fLat,fLon,fVn,fVe);
  // Could store fused position back into sensors struct or separate.
  status.last_update = millis();
}

void SensorRedundancySystem::calculate_sensor_health() {
  // Simplified health calculation
  float health_score = 0.0f;
  int sensor_count = 0;
  
  if (status.imu_health == SENSOR_HEALTHY) {
    health_score += 1.0f;
  }
  sensor_count++;
  
  if (status.gps_health == SENSOR_HEALTHY) {
    health_score += 1.0f;
  }
  sensor_count++;
  
  if (status.mag_health == SENSOR_HEALTHY) {
    health_score += 1.0f;
  }
  sensor_count++;
  
  if (status.baro_health == SENSOR_HEALTHY) {
    health_score += 1.0f;
  }
  sensor_count++;
  
  status.quality_score = health_score / sensor_count;
}

void SensorRedundancySystem::update_synthetic_gps(const SensorData &sensors) {
  // If dedicated GPS healthy + fix, clear synthetic flag
  if (sensors.gps.healthy && sensors.gps.fix) {
    status.synthetic_gps_available = false;
    synthetic_data.synthetic_gps_valid = false;
    return;
  }
  // Use optical-flow dead-reckoning as fallback
  OpticalFlowData flow = optical_flow.get_data();
  if (!flow.data_valid || flow.surface_quality < OPTICAL_FLOW_MIN_SURFACE_QUALITY) {
    status.synthetic_gps_available = false;
    synthetic_data.synthetic_gps_valid = false;
    return;
  }
  static bool origin_set = false;
  static float origin_lat = 0, origin_lon = 0;
  if (!origin_set) {
    // Use last known GPS if we ever had one
    if (sensors.gps.healthy) {
      origin_lat = sensors.gps.latitude;
      origin_lon = sensors.gps.longitude;
    }
    origin_set = true;
  }
  // Convert displacement (m) to lat/lon diff (deg). Assume small angles.
  float d_lat_deg = flow.displacement_y / ONE_DEG_LAT_M;        // N is +Y in flow frame
  float metres_per_deg_lon = ONE_DEG_LAT_M * cos(origin_lat * (PI/180.0f));
  if (metres_per_deg_lon < 1) metres_per_deg_lon = ONE_DEG_LAT_M; // safety
  float d_lon_deg = flow.displacement_x / metres_per_deg_lon;    // E is +X

  synthetic_data.synthetic_lat = origin_lat + d_lat_deg;
  synthetic_data.synthetic_lon = origin_lon + d_lon_deg;
  synthetic_data.gps_confidence = constrain(flow.surface_quality / 255.0f, 0.0f, 1.0f);

  synthetic_data.synthetic_gps_valid = true;
  status.synthetic_gps_available = true;
}

void SensorRedundancySystem::update_synthetic_magnetometer(const SensorData &sensors) {
  if (sensors.mag.healthy) {
    status.synthetic_mag_available = false;
    synthetic_data.synthetic_mag_valid = false;
    return;
  }
  // Use fused yaw estimate as heading
  synthetic_data.synthetic_heading = sensors.yaw;
  synthetic_data.mag_confidence = 0.5f; // crude confidence
  synthetic_data.synthetic_mag_valid = true;
  status.synthetic_mag_available = true;
}

void SensorRedundancySystem::update_synthetic_barometer(const SensorData &sensors) {
  if (sensors.baro.healthy) {
    status.synthetic_baro_available = false;
    synthetic_data.synthetic_baro_valid = false;
    return;
  }
  if (sensors.sonar.healthy) {
    synthetic_data.synthetic_altitude = sensors.sonar.distance; // metres
    synthetic_data.baro_confidence = 0.6f;
    synthetic_data.synthetic_baro_valid = true;
    status.synthetic_baro_available = true;
  }
}

void SensorRedundancySystem::determine_flight_capability() {
  // Simplified capability determination
  if (status.imu_health != SENSOR_HEALTHY) {
    current_capability = FLIGHT_CAPABILITY_NONE;
  } else if (status.gps_health == SENSOR_HEALTHY) {
    current_capability = FLIGHT_CAPABILITY_FULL_GPS;
  } else if (status.mag_health == SENSOR_HEALTHY || status.baro_health == SENSOR_HEALTHY) {
    current_capability = FLIGHT_CAPABILITY_ENHANCED;
  } else {
    current_capability = FLIGHT_CAPABILITY_BASIC;
  }
}

void SensorRedundancySystem::adjust_flight_characteristics() {
  // Adjust tuning based on capability
  switch (current_capability) {
    case FLIGHT_CAPABILITY_FULL_GPS:
      tuning.aggressiveness_factor = 1.0f;
      tuning.stability_gain_multiplier = 1.0f;
      break;
    case FLIGHT_CAPABILITY_ENHANCED:
      tuning.aggressiveness_factor = 0.8f;
      tuning.stability_gain_multiplier = 1.2f;
      break;
    case FLIGHT_CAPABILITY_BASIC:
      tuning.aggressiveness_factor = 0.6f;
      tuning.stability_gain_multiplier = 1.5f;
      break;
    case FLIGHT_CAPABILITY_NONE:
      tuning.aggressiveness_factor = 0.3f;
      tuning.stability_gain_multiplier = 2.0f;
      break;
  }
}

void SensorRedundancySystem::handle_sensor_failures() {
  // Simplified failure handling
  if (status.imu_health != SENSOR_HEALTHY) {
    Serial.println("WARNING: IMU sensor failure detected");
  }
}

bool SensorRedundancySystem::is_flight_safe() {
  return (current_capability != FLIGHT_CAPABILITY_NONE) && (status.imu_health == SENSOR_HEALTHY);
}

bool SensorRedundancySystem::is_arming_safe() {
  return is_flight_safe() && (status.quality_score > 0.5f);
}

String SensorRedundancySystem::get_safety_report() {
  String report = "Flight Capability: ";
  switch (current_capability) {
    case FLIGHT_CAPABILITY_FULL_GPS: report += "FULL GPS\n"; break;
    case FLIGHT_CAPABILITY_ENHANCED: report += "ENHANCED\n"; break;
    case FLIGHT_CAPABILITY_BASIC: report += "BASIC\n"; break;
    case FLIGHT_CAPABILITY_NONE: report += "NONE (UNSAFE)\n"; break;
  }
  
  report += "Sensor Status:\n";
  report += "  IMU: " + String(status.imu_health == SENSOR_HEALTHY ? "OK" : "FAIL") + "\n";
  report += "  GPS: " + String(status.gps_health == SENSOR_HEALTHY ? "OK" : "UNAVAILABLE") + "\n";
  report += "  MAG: " + String(status.mag_health == SENSOR_HEALTHY ? "OK" : "UNAVAILABLE") + "\n";
  report += "  BARO: " + String(status.baro_health == SENSOR_HEALTHY ? "OK" : "UNAVAILABLE") + "\n";
  
  return report;
}

String SensorRedundancySystem::get_sensor_alerts() {
  String alerts = "";
  
  if (status.imu_health != SENSOR_HEALTHY) {
    alerts += "ALERT: IMU sensor failure\n";
  }
  if (status.gps_health != SENSOR_HEALTHY && current_capability == FLIGHT_CAPABILITY_FULL_GPS) {
    alerts += "WARNING: GPS unavailable\n";
  }
  
  return alerts.length() > 0 ? alerts : "No sensor alerts";
}

SensorStatus SensorRedundancySystem::get_sensor_status() {
  return status;
}

void SensorRedundancySystem::enable_fallback(String sensor_type, bool enable) {
  // Simplified fallback configuration
  Serial.println("Fallback " + sensor_type + ": " + (enable ? "enabled" : "disabled"));
}

void SensorRedundancySystem::set_quality_threshold(String sensor_type, float threshold) {
  // Simplified threshold setting
  Serial.println("Quality threshold for " + sensor_type + " set to " + String(threshold));
}

void SensorRedundancySystem::force_synthetic_mode(String sensor_type, bool enable) {
  // Simplified synthetic mode forcing
  Serial.println("Synthetic " + sensor_type + ": " + (enable ? "forced on" : "auto"));
}

FlightCapability SensorRedundancySystem::get_flight_capability() {
  return current_capability;
}

FlightTuning SensorRedundancySystem::get_adaptive_tuning() {
  return tuning;
}

SyntheticSensorData SensorRedundancySystem::get_synthetic_data() {
  return synthetic_data;
} 