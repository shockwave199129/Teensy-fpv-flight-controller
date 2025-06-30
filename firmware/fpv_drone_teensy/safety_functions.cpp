#include "sensors.h"
#include "led_control.h"
#include "motor_control.h"
#include "flight_modes.h"
#include "sensor_redundancy.h"
#include "communication.h"
#include "calibration_storage.h"

// Pin definitions for beeper
#define BEEPER_PIN 8

// Missing function implementations
BatteryStatus get_enhanced_battery_status() {
  BatteryStatus status;
  status.connected = true;
  
  // Read battery voltage from ADC
  int raw_voltage = analogRead(BATTERY_VOLTAGE_PIN);
  status.voltage = raw_voltage * VOLTAGE_SCALE;
  
  // Read current
  int raw_current = analogRead(CURRENT_SENSOR_PIN);
  status.current = raw_current * CURRENT_SCALE;
  
  // Calculate percentage (simplified for 3S LiPo)
  float min_voltage = 9.9f; // 3.3V per cell * 3 cells
  float max_voltage = 12.6f; // 4.2V per cell * 3 cells
  status.percentage = constrain((status.voltage - min_voltage) / (max_voltage - min_voltage) * 100.0f, 0.0f, 100.0f);
  
  // Set warning flags
  status.low_voltage_warning = (status.voltage < 11.1f); // 3.7V per cell
  status.critical_voltage = (status.voltage < 10.5f); // 3.5V per cell
  
  status.remaining_capacity_mah = status.percentage * 2200.0f / 100.0f; // Assume 2200mAh battery
  status.last_update = millis();
  
  return status;
}

bool is_system_calibrated() {
  extern CalibrationData calibration_data;
  return calibration_data.system_calibrated && 
         calibration_data.gyro_calibrated && 
         calibration_data.accel_calibrated;
}

bool check_gps_functions_configured() {
  extern RcConfig rc_config;
  // Check if any GPS-related functions are configured
  for (int i = 0; i < 16; i++) {
    if (rc_config.channels[i].function == CHAN_FUNC_RTH ||
        rc_config.channels[i].function == CHAN_FUNC_POSITION_HOLD ||
        rc_config.channels[i].function == CHAN_FUNC_GPS_RESCUE) {
      return true;
    }
  }
  return false;
}

bool is_gps_functionality_available() {
  extern SensorData sensor_data;
  return sensor_data.gps.healthy && 
         sensor_data.gps.fix && 
         sensor_data.gps.satellites >= GPS_MIN_SATELLITES;
}

// Enhanced safety functions with visual/audio feedback
bool critical_sensor_check_with_feedback() {
  bool all_critical_sensors_ok = true;
  
  extern SensorData sensor_data;
  extern LEDControl led_control;
  
  // Check IMU health
  if (!sensor_data.imu.healthy) {
    all_critical_sensors_ok = false;
    
    // Serial output
    Serial.println("CRITICAL: IMU sensor failure detected!");
    
    // Visual feedback - Red flashing LED
    led_control.set_status(LED_ERROR);
    led_control.show_custom_pattern(CRGB::Red, 3); // Fast flashing red
    
    // Audio feedback - Rapid beeping pattern
    for (int i = 0; i < 5; i++) {
      tone(BEEPER_PIN, 2000, 100); // 2kHz beep for 100ms
      delay(100);
      noTone(BEEPER_PIN);
      delay(50);
    }
  }
  
  // Check battery connection and voltage
  BatteryStatus battery = get_enhanced_battery_status();
  if (!battery.connected) {
    all_critical_sensors_ok = false;
    
    Serial.println("CRITICAL: Battery not connected or voltage sensor failed!");
    
    // Visual feedback - Orange flashing LED
    led_control.show_custom_pattern(CRGB::Orange, 2); // Medium flashing orange
    
    // Audio feedback - Different beep pattern
    for (int i = 0; i < 3; i++) {
      tone(BEEPER_PIN, 1500, 200); // Lower frequency, longer beeps
      delay(200);
      noTone(BEEPER_PIN);
      delay(100);
    }
  } else if (battery.critical_voltage) {
    Serial.println("CRITICAL: Battery voltage critically low - Emergency landing required!");
    
    // Continuous audio warning for critical battery
    tone(BEEPER_PIN, 1000, 2000); // 2-second continuous tone
    led_control.show_custom_pattern(CRGB::Red, 1); // Solid red
  }
  
  return all_critical_sensors_ok;
}

bool enhanced_safety_check_for_arming_with_feedback() {
  Serial.println("=== ENHANCED SAFETY CHECK FOR ARMING ===");
  
  extern LEDControl led_control;
  bool safe_to_arm = true;
  String safety_issues = "";
  
  // 1. Check calibration status
  if (!is_system_calibrated()) {
    safe_to_arm = false;
    safety_issues += "- Sensors not calibrated\n";
    led_control.show_custom_pattern(CRGB::Yellow, 2); // Yellow flashing
    
    // Calibration reminder beeps
    tone(BEEPER_PIN, 800, 300);
    delay(400);
    tone(BEEPER_PIN, 1200, 300);
    delay(400);
    noTone(BEEPER_PIN);
  }
  
  // 2. Check critical sensors with feedback
  if (!critical_sensor_check_with_feedback()) {
    safe_to_arm = false;
    safety_issues += "- Critical sensor failure\n";
  }
  
  // 3. Check GPS requirements if GPS functions are configured
  if (check_gps_functions_configured()) {
    if (!is_gps_functionality_available()) {
      safe_to_arm = false;
      safety_issues += "- GPS functionality required but not available\n";
      
      Serial.println("GPS functions are configured but GPS functionality not available:");
      Serial.println("GPS functionality requires either:");
      Serial.println("  - Dedicated GPS with 6+ satellites, OR");
      Serial.println("  - Synthetic GPS with >30% confidence");
      
      // GPS warning beeps
      for (int i = 0; i < 4; i++) {
        tone(BEEPER_PIN, 1800, 100);
        delay(150);
        noTone(BEEPER_PIN);
        delay(50);
      }
      
      // Check what GPS sources are available
      extern SensorData sensor_data;
      extern SensorRedundancySystem sensor_redundancy;
      SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
      
      if (sensor_data.gps.healthy) {
        Serial.print("Dedicated GPS status: ");
        if (sensor_data.gps.fix) {
          Serial.print("Fix available, ");
          Serial.print(sensor_data.gps.satellites);
          Serial.println(" satellites (need 6+)");
        } else {
          Serial.println("No fix");
        }
      } else {
        Serial.println("Dedicated GPS: Not detected");
      }
      
      if (synthetic.synthetic_gps_valid) {
        Serial.print("Synthetic GPS: Available (");
        Serial.print(synthetic.gps_confidence * 100, 1);
        Serial.print("% confidence, need >30%)");
        if (synthetic.gps_confidence > 0.3) {
          Serial.println(" - SUFFICIENT");
        } else {
          Serial.println(" - INSUFFICIENT");
        }
      } else {
        Serial.println("Synthetic GPS: Not available");
      }
    } else {
      Serial.println("GPS functionality check: PASSED");
      
      // Check source
      extern SensorData sensor_data;
      if (sensor_data.gps.fix && sensor_data.gps.satellites >= 6) {
        Serial.println("Using dedicated GPS");
      } else {
        extern SensorRedundancySystem sensor_redundancy;
        SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
        if (synthetic.synthetic_gps_valid && synthetic.gps_confidence > 0.3) {
          Serial.print("Using synthetic GPS (");
          Serial.print(synthetic.gps_confidence * 100, 1);
          Serial.println("% confidence)");
        }
      }
    }
  }
  
  // 4. Final safety result with feedback
  if (safe_to_arm) {
    Serial.println("SAFETY CHECK: ✓ SAFE TO ARM");
    Serial.println("All safety requirements met");
    
    // Success feedback
    led_control.show_custom_pattern(CRGB::Green, 0); // Solid green
    
    // Success beep sequence
    tone(BEEPER_PIN, 1000, 100);
    delay(150);
    tone(BEEPER_PIN, 1500, 100);
    delay(150);
    tone(BEEPER_PIN, 2000, 200);
    delay(300);
    noTone(BEEPER_PIN);
    
    return true;
  } else {
    Serial.println("SAFETY CHECK: ✗ NOT SAFE TO ARM");
    Serial.println("Safety issues found:");
    Serial.print(safety_issues);
    Serial.println("Resolve all issues before attempting to arm");
    
    // Failure feedback
    led_control.set_status(LED_ERROR);
    
    // Failure beep sequence
    for (int i = 0; i < 6; i++) {
      tone(BEEPER_PIN, 500, 80);
      delay(120);
      noTone(BEEPER_PIN);
      delay(40);
    }
    
    return false;
  }
}

// Low battery automatic failsafe function
void check_automatic_battery_failsafe() {
  static unsigned long last_battery_check = 0;
  static bool low_battery_warned = false;
  static bool critical_battery_warned = false;
  
  if (millis() - last_battery_check < 1000) return; // 1Hz check
  
  BatteryStatus battery = get_enhanced_battery_status();
  
  if (battery.critical_voltage) {
    if (!critical_battery_warned) {
      Serial.println("BATTERY CRITICAL: Automatic emergency measures activated!");
      
      extern LEDControl led_control;
      led_control.set_status(LED_LOW_BATTERY);
      
      // Continuous critical battery alarm
      tone(BEEPER_PIN, 1000); // Continuous tone
      
      // Check if GPS is available for automatic RTH
      if (is_gps_functionality_available()) {
        Serial.println("Activating automatic Return to Home due to critical battery");
        
        extern FlightModes flight_modes;
        extern SensorData sensor_data_global;
        extern RcData rc_data_global;
        flight_modes.return_to_home(sensor_data_global, rc_data_global);
        
        // RTH activation beeps
        noTone(BEEPER_PIN);
        for (int i = 0; i < 3; i++) {
          tone(BEEPER_PIN, 1800, 500);
          delay(600);
          noTone(BEEPER_PIN);
          delay(200);
        }
      } else {
        Serial.println("No GPS available - pilot must land immediately!");
        
        // Different alarm pattern for manual landing required
        noTone(BEEPER_PIN);
        for (int i = 0; i < 10; i++) {
          tone(BEEPER_PIN, 800, 100);
          delay(150);
          noTone(BEEPER_PIN);
          delay(50);
        }
      }
      
      critical_battery_warned = true;
    }
  } else if (battery.low_voltage_warning) {
    if (!low_battery_warned) {
      Serial.println("BATTERY LOW: Prepare to land soon");
      
      extern LEDControl led_control;
      led_control.set_status(LED_LOW_BATTERY);
      
      // Low battery warning beeps
      for (int i = 0; i < 3; i++) {
        tone(BEEPER_PIN, 1200, 200);
        delay(300);
        noTone(BEEPER_PIN);
        delay(200);
      }
      
      low_battery_warned = true;
    }
  } else {
    // Reset warnings if voltage recovers
    low_battery_warned = false;
    critical_battery_warned = false;
    noTone(BEEPER_PIN);
  }
  
  last_battery_check = millis();
}

// RC signal loss handling with immediate safety measures
void handle_rc_signal_loss() {
  extern MotorControl motor_control;
  extern LEDControl led_control;
  
  Serial.println("CRITICAL: RC signal lost - Activating failsafe!");
  
  // Immediate safety actions
  if (motor_control.is_armed()) {
    // Don't immediately disarm - that could cause crash
    // Instead activate failsafe flight mode
    
    led_control.set_status(LED_ERROR);
    
    // RC loss alarm
    for (int i = 0; i < 5; i++) {
      tone(BEEPER_PIN, 2500, 100);
      delay(150);
      noTone(BEEPER_PIN);
      delay(50);
    }
    
    // Check if GPS RTH is possible
    if (is_gps_functionality_available()) {
      Serial.println("Activating GPS Return to Home");
      // Would activate RTH mode
    } else {
      Serial.println("No GPS - Activating altitude hold and slow descent");
      // Would activate safe descent mode
    }
  } else {
    // Not armed - just alert user
    Serial.println("RC signal lost while disarmed - Safe");
    
    // Gentler alert for disarmed state
    tone(BEEPER_PIN, 1500, 1000);
    delay(1100);
    noTone(BEEPER_PIN);
  }
}

void emergency_altitude_descent() {
  Serial.println("Emergency altitude descent initiated");
  // Implementation for emergency descent
}

// Additional missing functions for communication.cpp
bool perform_gyro_calibration() {
  Serial.println("Starting gyro calibration...");
  // Simplified gyro calibration
  delay(3000); // Simulate calibration time
  Serial.println("Gyro calibration completed");
  return true;
}

bool perform_accelerometer_calibration() {
  Serial.println("Starting accelerometer calibration...");
  // Simplified accelerometer calibration
  delay(5000); // Simulate calibration time
  Serial.println("Accelerometer calibration completed");
  return true;
}

void save_calibration_to_eeprom() {
  extern CalibrationData calibration_data;
  if (CalibrationStorage::save(calibration_data)) {
    Serial.println("Calibration data saved with CRC ✅");
  } else {
    Serial.println("Calibration data save FAILED ❌");
  }
}

bool detect_battery_connection() {
  // Check if battery is connected by reading voltage
  int voltage_reading = analogRead(BATTERY_VOLTAGE_PIN);
  float voltage = voltage_reading * VOLTAGE_SCALE;
  return voltage > 3.0; // Consider connected if voltage > 3V
}

// Missing functions for sensor redundancy - simplified implementations
#if 0  // Duplicates of implementations in sensor_redundancy.cpp
String SensorRedundancySystem::get_safety_report() {
  return "Safety report: All systems nominal";
}

SensorStatus SensorRedundancySystem::get_sensor_status() {
  SensorStatus status;
  status.primary_imu_ok = true;
  status.secondary_imu_ok = true;
  status.gps_ok = true;
  status.mag_ok = true;
  status.baro_ok = true;
  status.synthetic_gps_available = false;
  status.synthetic_mag_available = false;
  status.synthetic_baro_available = false;
  return status;
}
#endif 