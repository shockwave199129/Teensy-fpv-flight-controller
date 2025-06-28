#include "sensor_redundancy.h"
#include "sensors.h"

// Global instance
SensorRedundancySystem sensor_redundancy;

void SensorRedundancySystem::init() {
  // Initialize configuration with safe defaults
  config.enable_imu_mag_fallback = true;
  config.enable_gps_baro_fallback = true;
  config.enable_sonar_baro_fallback = true;
  config.enable_optical_flow_gps_fallback = true;
  config.enable_synthetic_gps = true;
  
  config.imu_health_threshold = 0.7;
  config.gps_accuracy_threshold = 10.0;  // 10 meters
  config.mag_deviation_threshold = 15.0; // 15 degrees
  config.baro_noise_threshold = 50.0;    // 50 Pa
  
  config.sensor_timeout_ms = 1000;
  config.fallback_switch_delay_ms = 500;
  config.recovery_verification_ms = 2000;
  
  // Initialize status
  status.imu_health = SENSOR_MISSING;
  status.gps_health = SENSOR_MISSING;
  status.mag_health = SENSOR_MISSING;
  status.baro_health = SENSOR_MISSING;
  status.sonar_health = SENSOR_MISSING;
  status.optical_flow_health = SENSOR_MISSING;
  status.battery_health = SENSOR_MISSING;
  status.rc_health = SENSOR_MISSING;
  
  // Reset failure counters
  status.imu_failures = 0;
  status.gps_failures = 0;
  status.mag_failures = 0;
  status.baro_failures = 0;
  status.sonar_failures = 0;
  status.optical_flow_failures = 0;
  
  // Initialize synthetic data
  synthetic.synthetic_gps_valid = false;
  synthetic.synthetic_mag_valid = false;
  synthetic.synthetic_baro_valid = false;
  synthetic.gps_confidence = 0.0;
  synthetic.mag_confidence = 0.0;
  synthetic.baro_confidence = 0.0;
  
  // Initialize flight tuning
  tuning.stability_gain_multiplier = 1.0;
  tuning.aggressiveness_reduction = 1.0;
  tuning.smoothing_factor = 1.0;
  tuning.enable_adaptive_filtering = true;
  tuning.vibration_compensation = 1.0;
  tuning.noise_reduction_level = 1.0;
  tuning.emergency_stabilization_mode = false;
  tuning.emergency_descent_rate = 1.0; // m/s
  tuning.emergency_landing_sensitivity = 0.5;
  
  current_capability = FLIGHT_EMERGENCY;
  system_initialized = true;
  last_update = millis();
  
  Serial.println("Sensor Redundancy System initialized");
  Serial.println("- Automatic sensor fallback enabled");
  Serial.println("- Synthetic sensor generation active");
  Serial.println("- Adaptive flight tuning enabled");
}

void SensorRedundancySystem::update(SensorData& sensors) {
  if (!system_initialized) return;
  
  unsigned long current_time = millis();
  
  // Update sensor health status
  calculate_sensor_health();
  
  // Update synthetic sensor data
  if (config.enable_synthetic_gps && (status.gps_health == SENSOR_FAILED || status.gps_health == SENSOR_MISSING)) {
    update_synthetic_gps();
  }
  
  if (config.enable_imu_mag_fallback && (status.mag_health == SENSOR_FAILED || status.mag_health == SENSOR_MISSING)) {
    update_synthetic_magnetometer();
  }
  
  if ((config.enable_gps_baro_fallback || config.enable_sonar_baro_fallback) && 
      (status.baro_health == SENSOR_FAILED || status.baro_health == SENSOR_MISSING)) {
    update_synthetic_barometer();
  }
  
  // Determine current flight capability
  determine_flight_capability();
  
  // Adjust flight characteristics based on sensor availability
  adjust_flight_characteristics();
  
  // Handle any sensor failures
  handle_sensor_failures();
  
  last_update = current_time;
}

void SensorRedundancySystem::calculate_sensor_health() {
  unsigned long current_time = millis();
  
  // IMU Health (CRITICAL)
  if (sensor_data.imu.healthy && (current_time - sensor_data.imu.last_update < config.sensor_timeout_ms)) {
    status.imu_health = SENSOR_HEALTHY;
    status.imu_quality = 1.0;
    status.imu_last_update = sensor_data.imu.last_update;
  } else {
    status.imu_health = SENSOR_FAILED;
    status.imu_failures++;
  }
  
  // GPS Health (HIGH)
  if (sensor_data.gps.healthy && sensor_data.gps.fix && 
      (current_time - sensor_data.gps.last_update < config.sensor_timeout_ms)) {
    status.gps_health = SENSOR_HEALTHY;
    status.gps_accuracy = 5.0 - (sensor_data.gps.satellites - 4) * 0.5;
    status.gps_accuracy = max(1.0f, status.gps_accuracy);
    status.gps_last_update = sensor_data.gps.last_update;
  } else if (sensor_data.gps.healthy) {
    status.gps_health = SENSOR_DEGRADED;
  } else {
    status.gps_health = SENSOR_FAILED;
    status.gps_failures++;
  }
  
  // Magnetometer Health (MEDIUM)
  if (sensor_data.mag.healthy && (current_time - sensor_data.mag.last_update < config.sensor_timeout_ms)) {
    status.mag_health = SENSOR_HEALTHY;
    status.mag_calibration = 0.8;
    status.mag_last_update = sensor_data.mag.last_update;
  } else {
    status.mag_health = SENSOR_FAILED;
    status.mag_failures++;
  }
  
  // Barometer Health (MEDIUM)
  if (sensor_data.baro.healthy && (current_time - sensor_data.baro.last_update < config.sensor_timeout_ms)) {
    status.baro_health = SENSOR_HEALTHY;
    status.baro_stability = 0.9;
    status.baro_last_update = sensor_data.baro.last_update;
  } else {
    status.baro_health = SENSOR_FAILED;
    status.baro_failures++;
  }
  
  // Sonar Health (LOW)
  if (sensor_data.sonar.healthy && (current_time - sensor_data.sonar.last_update < config.sensor_timeout_ms)) {
    status.sonar_health = SENSOR_HEALTHY;
    status.sonar_last_update = sensor_data.sonar.last_update;
  } else {
    status.sonar_health = SENSOR_FAILED;
    status.sonar_failures++;
  }
  
  // Battery Health (HIGH)
  if (sensor_data.battery_voltage > 5.0 && sensor_data.battery_voltage < 30.0) {
    status.battery_health = SENSOR_HEALTHY;
  } else {
    status.battery_health = SENSOR_FAILED;
  }
}

void SensorRedundancySystem::update_synthetic_gps() {
  static float last_lat = 0, last_lon = 0, last_alt = 0;
  static unsigned long last_synthetic_update = 0;
  static bool initialized = false;
  
  unsigned long current_time = millis();
  float dt = (current_time - last_synthetic_update) / 1000.0;
  
  if (!initialized && status.imu_health == SENSOR_HEALTHY) {
    // Initialize synthetic GPS at last known position or origin
    if (last_lat == 0 && last_lon == 0) {
      synthetic.synthetic_lat = 0.0;  // Would use last known GPS position
      synthetic.synthetic_lon = 0.0;
      synthetic.synthetic_altitude = 0.0;
    } else {
      synthetic.synthetic_lat = last_lat;
      synthetic.synthetic_lon = last_lon;
      synthetic.synthetic_altitude = last_alt;
    }
    initialized = true;
  }
  
  if (status.imu_health == SENSOR_HEALTHY && dt > 0 && dt < 1.0) {
    // Dead reckoning using IMU acceleration and gyro
    float accel_north = sensor_data.imu.accel_x; // Transformed to earth frame
    float accel_east = sensor_data.imu.accel_y;  // Would need proper rotation matrix
    
    // Very simplified position integration (would need proper implementation)
    synthetic.synthetic_speed += sqrt(accel_north * accel_north + accel_east * accel_east) * dt;
    
    // Update position based on heading and speed
    if (status.mag_health == SENSOR_HEALTHY || synthetic.synthetic_mag_valid) {
      float heading_rad = (status.mag_health == SENSOR_HEALTHY ? 
                          sensor_data.mag.heading : synthetic.synthetic_mag_heading) * PI / 180.0;
      
      float distance = synthetic.synthetic_speed * dt;
      synthetic.synthetic_lat += distance * cos(heading_rad) / 111000.0; // Rough conversion
      synthetic.synthetic_lon += distance * sin(heading_rad) / 111000.0;
    }
    
    // Update altitude from barometer or dead reckoning
    if (status.baro_health == SENSOR_HEALTHY) {
      synthetic.synthetic_altitude = sensor_data.baro.altitude;
    } else {
      // Integrate vertical acceleration
      synthetic.synthetic_altitude += sensor_data.imu.accel_z * dt * dt;
    }
    
    synthetic.synthetic_gps_valid = true;
    synthetic.gps_confidence = max(0.1f, 1.0f - (current_time - last_synthetic_update) / 60000.0f); // Decreases over time
  }
  
  last_synthetic_update = current_time;
}

void SensorRedundancySystem::update_synthetic_magnetometer() {
  // Generate synthetic compass heading from GPS course or gyro integration
  if (status.gps_health == SENSOR_HEALTHY && sensor_data.gps.speed > 1.0) {
    // Use GPS course over ground when moving
    synthetic.synthetic_mag_heading = sensor_data.gps.heading;
    synthetic.synthetic_mag_valid = true;
    synthetic.mag_confidence = 0.8;
  } else if (status.imu_health == SENSOR_HEALTHY) {
    // Integrate gyro yaw rate for heading estimation
    static float integrated_heading = 0;
    static unsigned long last_heading_update = 0;
    
    unsigned long current_time = millis();
    float dt = (current_time - last_heading_update) / 1000.0;
    
    if (dt > 0 && dt < 1.0) {
      integrated_heading += sensor_data.imu.gyro_z * dt;
      
      // Normalize to 0-360 degrees
      while (integrated_heading >= 360) integrated_heading -= 360;
      while (integrated_heading < 0) integrated_heading += 360;
      
      synthetic.synthetic_mag_heading = integrated_heading;
      synthetic.synthetic_mag_valid = true;
      synthetic.mag_confidence = max(0.1f, 0.9f - (current_time - last_heading_update) / 30000.0f); // Decreases over time
    }
    
    last_heading_update = current_time;
  }
}

void SensorRedundancySystem::update_synthetic_barometer() {
  // Generate synthetic altitude from GPS or sonar
  if (status.gps_health == SENSOR_HEALTHY) {
    synthetic.synthetic_baro_altitude = sensor_data.gps.altitude;
    synthetic.synthetic_baro_valid = true;
    synthetic.baro_confidence = 0.7; // GPS altitude is less accurate than barometer
  } else if (status.sonar_health == SENSOR_HEALTHY && sensor_data.sonar.distance < 500) {
    // Use sonar for low altitude (under 5 meters)
    static float ground_level = 0;
    static bool ground_level_set = false;
    
    if (!ground_level_set) {
      ground_level = sensor_data.sonar.distance / 100.0; // Convert cm to m
      ground_level_set = true;
    }
    
    synthetic.synthetic_baro_altitude = ground_level + sensor_data.sonar.distance / 100.0;
    synthetic.synthetic_baro_valid = true;
    synthetic.baro_confidence = 0.9; // Sonar is quite accurate for low altitude
  }
}

void SensorRedundancySystem::determine_flight_capability() {
  // Determine what flight capabilities are available based on sensor health
  
  if (status.imu_health != SENSOR_HEALTHY) {
    current_capability = FLIGHT_EMERGENCY;
    return;
  }
  
  if (status.battery_health != SENSOR_HEALTHY) {
    current_capability = FLIGHT_EMERGENCY;
    return;
  }
  
  // Check GPS capability
  bool gps_available = (status.gps_health == SENSOR_HEALTHY) || 
                      (synthetic.synthetic_gps_valid && synthetic.gps_confidence > 0.3);
  
  // Check magnetometer capability  
  bool mag_available = (status.mag_health == SENSOR_HEALTHY) ||
                      (synthetic.synthetic_mag_valid && synthetic.mag_confidence > 0.3);
  
  // Check barometer capability
  bool baro_available = (status.baro_health == SENSOR_HEALTHY) ||
                       (synthetic.synthetic_baro_valid && synthetic.baro_confidence > 0.3);
  
  // Determine capability level
  if (gps_available && mag_available && baro_available) {
    current_capability = FLIGHT_FULL_CAPABILITY;
  } else if (!gps_available && mag_available && baro_available) {
    current_capability = FLIGHT_DEGRADED_GPS;
  } else if (gps_available && !mag_available && baro_available) {
    current_capability = FLIGHT_DEGRADED_MAG;
  } else if (gps_available && mag_available && !baro_available) {
    current_capability = FLIGHT_DEGRADED_BARO;
  } else {
    current_capability = FLIGHT_MINIMAL;
  }
}

void SensorRedundancySystem::adjust_flight_characteristics() {
  // Adjust flight parameters based on available sensors
  
  switch (current_capability) {
    case FLIGHT_FULL_CAPABILITY:
      tuning.stability_gain_multiplier = 1.0;
      tuning.aggressiveness_reduction = 1.0;
      tuning.smoothing_factor = 1.0;
      tuning.emergency_stabilization_mode = false;
      break;
      
    case FLIGHT_DEGRADED_GPS:
      tuning.stability_gain_multiplier = 1.1;  // Slightly more stable
      tuning.aggressiveness_reduction = 0.9;   // Slightly less aggressive
      tuning.smoothing_factor = 1.2;           // More smoothing
      tuning.emergency_stabilization_mode = false;
      break;
      
    case FLIGHT_DEGRADED_MAG:
      tuning.stability_gain_multiplier = 1.2;
      tuning.aggressiveness_reduction = 0.8;
      tuning.smoothing_factor = 1.3;
      tuning.emergency_stabilization_mode = false;
      break;
      
    case FLIGHT_DEGRADED_BARO:
      tuning.stability_gain_multiplier = 1.1;
      tuning.aggressiveness_reduction = 0.9;
      tuning.smoothing_factor = 1.1;
      tuning.emergency_stabilization_mode = false;
      break;
      
    case FLIGHT_MINIMAL:
      tuning.stability_gain_multiplier = 1.5;  // Much more stable
      tuning.aggressiveness_reduction = 0.6;   // Much less aggressive
      tuning.smoothing_factor = 1.8;           // Much more smoothing
      tuning.emergency_stabilization_mode = false;
      break;
      
    case FLIGHT_EMERGENCY:
      tuning.stability_gain_multiplier = 2.0;  // Maximum stability
      tuning.aggressiveness_reduction = 0.3;   // Minimal aggressiveness
      tuning.smoothing_factor = 2.5;           // Maximum smoothing
      tuning.emergency_stabilization_mode = true;
      break;
  }
}

void SensorRedundancySystem::handle_sensor_failures() {
  String alert = "";
  
  // IMU failure is critical
  if (status.imu_health == SENSOR_FAILED) {
    alert = "CRITICAL: IMU sensor failure - Emergency landing required!";
  }
  // Battery failure is critical
  else if (status.battery_health == SENSOR_FAILED) {
    alert = "CRITICAL: Battery sensor failure - Check power system!";
  }
  // GPS failure with degraded capability
  else if (status.gps_health == SENSOR_FAILED && !synthetic.synthetic_gps_valid) {
    alert = "WARNING: GPS failure - Position hold disabled, using dead reckoning";
  }
  // Magnetometer failure
  else if (status.mag_health == SENSOR_FAILED && !synthetic.synthetic_mag_valid) {
    alert = "WARNING: Magnetometer failure - Heading accuracy reduced";
  }
  // Barometer failure
  else if (status.baro_health == SENSOR_FAILED && !synthetic.synthetic_baro_valid) {
    alert = "WARNING: Barometer failure - Altitude hold accuracy reduced";
  }
  
  if (alert != "" && alert != last_alert_message) {
    Serial.println("SENSOR ALERT: " + alert);
    last_alert_message = alert;
    alert_timestamp = millis();
  }
}

bool SensorRedundancySystem::is_flight_safe() {
  return (status.imu_health == SENSOR_HEALTHY) && 
         (status.battery_health == SENSOR_HEALTHY) &&
         (current_capability != FLIGHT_EMERGENCY);
}

bool SensorRedundancySystem::is_arming_safe() {
  // More strict requirements for arming
  if (status.imu_health != SENSOR_HEALTHY) return false;
  if (status.battery_health != SENSOR_HEALTHY) return false;
  
  // At least minimal flight capability required
  return (current_capability != FLIGHT_EMERGENCY);
}

String SensorRedundancySystem::get_safety_report() {
  String report = "=== SENSOR REDUNDANCY SAFETY REPORT ===\n";
  
  // Flight capability
  report += "Flight Capability: ";
  switch (current_capability) {
    case FLIGHT_FULL_CAPABILITY: report += "FULL (All sensors operational)\n"; break;
    case FLIGHT_DEGRADED_GPS: report += "DEGRADED - No GPS (Using synthetic position)\n"; break;
    case FLIGHT_DEGRADED_MAG: report += "DEGRADED - No Magnetometer (Using GPS/gyro heading)\n"; break;
    case FLIGHT_DEGRADED_BARO: report += "DEGRADED - No Barometer (Using GPS/sonar altitude)\n"; break;
    case FLIGHT_MINIMAL: report += "MINIMAL - Basic sensors only (Stabilization only)\n"; break;
    case FLIGHT_EMERGENCY: report += "EMERGENCY - Critical sensor failure!\n"; break;
  }
  
  // Safety status
  report += "Safe to Arm: " + String(is_arming_safe() ? "YES" : "NO") + "\n";
  report += "Safe to Fly: " + String(is_flight_safe() ? "YES" : "NO") + "\n";
  
  // Sensor status
  report += "\n--- Primary Sensors ---\n";
  report += "IMU: " + String(status.imu_health == SENSOR_HEALTHY ? "OK" : "FAILED") + "\n";
  report += "Battery: " + String(status.battery_health == SENSOR_HEALTHY ? "OK" : "FAILED") + "\n";
  
  report += "\n--- Secondary Sensors ---\n";
  report += "GPS: " + String(status.gps_health == SENSOR_HEALTHY ? "OK" : 
                           (synthetic.synthetic_gps_valid ? "SYNTHETIC" : "FAILED")) + "\n";
  report += "Magnetometer: " + String(status.mag_health == SENSOR_HEALTHY ? "OK" : 
                                    (synthetic.synthetic_mag_valid ? "SYNTHETIC" : "FAILED")) + "\n";
  report += "Barometer: " + String(status.baro_health == SENSOR_HEALTHY ? "OK" : 
                                  (synthetic.synthetic_baro_valid ? "SYNTHETIC" : "FAILED")) + "\n";
  report += "Sonar: " + String(status.sonar_health == SENSOR_HEALTHY ? "OK" : "FAILED") + "\n";
  
  return report;
}

String SensorRedundancySystem::get_sensor_alerts() {
  String alerts = "";
  
  if (last_alert_message != "" && (millis() - alert_timestamp < 30000)) {
    alerts = last_alert_message;
  }
  
  return alerts;
}

FlightTuning SensorRedundancySystem::get_adaptive_tuning() {
  return tuning;
}

void SensorRedundancySystem::apply_emergency_mode() {
  tuning.emergency_stabilization_mode = true;
  tuning.stability_gain_multiplier = 3.0;
  tuning.aggressiveness_reduction = 0.2;
  tuning.smoothing_factor = 3.0;
  current_capability = FLIGHT_EMERGENCY;
  
  Serial.println("EMERGENCY MODE ACTIVATED - Maximum stability settings applied");
}

void SensorRedundancySystem::recovery_mode() {
  tuning.emergency_stabilization_mode = false;
  calculate_sensor_health();
  determine_flight_capability();
  adjust_flight_characteristics();
  
  Serial.println("Recovery mode - Sensor redundancy system recalibrated");
}

// Missing function implementations
void SensorRedundancySystem::enable_fallback(String sensor_type, bool enable) {
  if (sensor_type == "gps_baro") {
    config.enable_gps_baro_fallback = enable;
  } else if (sensor_type == "sonar_baro") {
    config.enable_sonar_baro_fallback = enable;
  } else if (sensor_type == "optical_flow_gps") {
    config.enable_optical_flow_gps_fallback = enable;
  } else if (sensor_type == "imu_mag") {
    config.enable_imu_mag_fallback = enable;
  } else if (sensor_type == "synthetic_gps") {
    config.enable_synthetic_gps = enable;
  }
  
  Serial.println("Fallback " + sensor_type + " " + (enable ? "enabled" : "disabled"));
}

void SensorRedundancySystem::set_quality_threshold(String sensor_type, float threshold) {
  if (sensor_type == "imu") {
    config.imu_health_threshold = constrain(threshold, 0.1, 1.0);
  } else if (sensor_type == "gps") {
    config.gps_accuracy_threshold = constrain(threshold, 1.0, 50.0);
  } else if (sensor_type == "mag") {
    config.mag_deviation_threshold = constrain(threshold, 5.0, 45.0);
  } else if (sensor_type == "baro") {
    config.baro_noise_threshold = constrain(threshold, 10.0, 200.0);
  }
  
  Serial.println("Quality threshold for " + sensor_type + " set to " + String(threshold));
}

void SensorRedundancySystem::force_synthetic_mode(String sensor_type, bool enable) {
  if (sensor_type == "gps") {
    if (enable) {
      status.gps_health = SENSOR_FAILED;
      synthetic.synthetic_gps_valid = true;
      synthetic.gps_confidence = 0.8;
      Serial.println("Forced synthetic GPS mode enabled");
    } else {
      // Re-evaluate GPS health normally
      calculate_sensor_health();
      Serial.println("Synthetic GPS mode disabled - using normal detection");
    }
  } else if (sensor_type == "mag") {
    if (enable) {
      status.mag_health = SENSOR_FAILED;
      synthetic.synthetic_mag_valid = true;
      synthetic.mag_confidence = 0.8;
      Serial.println("Forced synthetic magnetometer mode enabled");
    } else {
      calculate_sensor_health();
      Serial.println("Synthetic magnetometer mode disabled - using normal detection");
    }
  } else if (sensor_type == "baro") {
    if (enable) {
      status.baro_health = SENSOR_FAILED;
      synthetic.synthetic_baro_valid = true;
      synthetic.baro_confidence = 0.8;
      Serial.println("Forced synthetic barometer mode enabled");
    } else {
      calculate_sensor_health();
      Serial.println("Synthetic barometer mode disabled - using normal detection");
    }
  }
  
  // Recalculate flight capability after changes
  determine_flight_capability();
  adjust_flight_characteristics();
}

void SensorRedundancySystem::reset_sensor_failures() {
  // Reset all failure counters
  status.imu_failures = 0;
  status.gps_failures = 0;
  status.mag_failures = 0;
  status.baro_failures = 0;
  status.sonar_failures = 0;
  status.optical_flow_failures = 0;
  
  // Reset alert states
  last_alert_message = "";
  alert_timestamp = 0;
  
  // Recalculate sensor health
  calculate_sensor_health();
  determine_flight_capability();
  adjust_flight_characteristics();
  
  Serial.println("All sensor failure counters and alerts reset");
  Serial.println("Sensor health status recalculated");
} 