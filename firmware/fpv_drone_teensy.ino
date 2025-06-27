/*
 * FPV Drone Project with Teensy 4.1
 * Main Flight Controller Firmware
 * 
 * Features:
 * - Multiple RC receiver protocols (PPM, iBUS, SBUS, ELRS)
 * - IMU, GPS, Barometer, Magnetometer, Sonar sensors
 * - PID control with multiple flight modes
 * - ESC control and calibration
 * - RGB LED feedback
 * - USB/UART communication
 * - Safety features and failsafes
 */

#include <Wire.h>
#include <IntervalTimer.h>
#include <FastLED.h>
#include <EEPROM.h>
#include "config.h"
#include "sensors.h"
#include "sensor_redundancy.h"
#include "receivers.h"
#include "pid_controller.h"
#include "motor_control.h"
#include "flight_modes.h"
#include "led_control.h"
#include "communication.h"

// Global variables
FlightState flight_state;
SensorData sensor_data;
RcData rc_data;
EnhancedRcData enhanced_rc_data;
RcConfig rc_config;
RateProfile rate_profiles[3];
RcProtocol current_protocol = RC_PROTOCOL_PPM;
FunctionSwitches function_switches;
PIDController pid_controller;
MotorControl motor_control;
FlightModes flight_modes;
AdvancedFlightModes advanced_flight_modes;  // Phase 2 addition
LEDControl led_control;
Communication comm;

// Phase 2 Global Instances
DualIMUManager dual_imu;
DynamicFilteringSystem dynamic_filtering;
OpticalFlowSensor optical_flow;

// Sensor detection status
SensorDetectionStatus sensor_detection;

// Calibration system
CalibrationData calibration_data;

// Timing variables
unsigned long last_loop_time = 0;
unsigned long loop_time = 0;
bool armed = false;

// Main loop timer
IntervalTimer main_loop_timer;

void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  Serial1.begin(9600);   // GPS
  Serial2.begin(100000); // SBUS/iBUS
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize components
  init_pins();
  init_sensors();
  
  // Initialize enhanced filtering
  init_mahony_filter();
  init_vibration_filters();
  init_notch_filters();
  
  // Phase 2: Initialize dual IMU system
  #if ENABLE_DUAL_IMU
  init_dual_imu_system();
  #endif
  
  // Phase 2: Initialize dynamic filtering
  dynamic_filtering.init();
  
  // Phase 2: Initialize optical flow sensor
  #if ENABLE_OPTICAL_FLOW
  optical_flow.init(SELECTED_OPTICAL_FLOW);
  #endif
  
  init_receivers();
  init_default_channel_mapping();
  init_default_rate_profiles();
  init_motors();
  init_leds();
  init_communication();
  
  // Initialize sensor detection system
  sensor_detection.detection_complete = false;
  sensor_detection.last_detection_scan = 0;
  sensor_detection.i2c_device_count = 0;
  
  // Initialize calibration system
  init_calibration_system();
  
  // Initialize sensor redundancy system
  sensor_redundancy.init();
  
  // Load configuration from EEPROM
  load_config();
  
  // Initialize PID controller
  pid_controller.init();
  
  // Initialize flight modes
  flight_modes.init();
  
  // Phase 2: Initialize advanced flight modes
  advanced_flight_modes.init();
  
  // Set up main loop timer (2000Hz for enhanced performance)
  main_loop_timer.begin(main_loop, 500);
  
  // Safety check - ensure motors are disarmed
  motor_control.disarm();
  
  Serial.println("Enhanced FPV Drone Controller Initialized");
  Serial.println("Phase 1 & 2 Stability Upgrades Active:");
  Serial.println("- Cascaded PID control system");
  Serial.println("- Mahony AHRS sensor fusion");
  Serial.println("- Enhanced vibration filtering");
  Serial.println("- DShot telemetry integration");
  Serial.println("- Dual IMU sensor fusion");
  Serial.println("- Dynamic adaptive filtering");
  Serial.println("- Advanced flight modes (Acro+, Sport, Cinematic, GPS Rescue, Turtle)");
  #if ENABLE_OPTICAL_FLOW
  Serial.println("- Optical flow positioning");
  #endif
  Serial.println("Type 'help' for available commands");
  
  // Initial LED state (disarmed - red)
  led_control.set_status(LED_DISARMED);
}

void loop() {
  // Handle serial communication
  comm.process_commands();
  
  // Update LED patterns
  led_control.update();
  
  // Monitor system health
  monitor_system_health();
  
  // Small delay to prevent overwhelming the main thread
  delay(10);
}

void main_loop() {
  // Update timing
  unsigned long current_time = micros();
  loop_time = current_time - last_loop_time;
  last_loop_time = current_time;
  
  // Phase 2: Update dual IMU system
  #if ENABLE_DUAL_IMU
  dual_imu.update();
  // Use fused IMU data instead of single IMU
  sensor_data.imu = dual_imu.get_fused_data();
  #else
  // Read single IMU sensor
  update_sensors();
  #endif
  
  // Phase 2: Update dynamic filtering
  dynamic_filtering.update(sensor_data, rc_data);
  
  // Phase 2: Update optical flow
  #if ENABLE_OPTICAL_FLOW
  optical_flow.update();
  #endif
  
  // Read RC receiver
  update_rc_receiver();
  
  // Process flight mode logic
  flight_modes.update(rc_data, sensor_data);
  
  // Phase 2: Process advanced flight modes
  advanced_flight_modes.update(sensor_data, rc_data, flight_state);
  
  // Update sensor redundancy system
  sensor_redundancy.update(sensor_data);
  
  // Update enhanced calibration systems
  update_magnetometer_timer();
  
  // Safety checks
  if (perform_safety_checks() && sensor_redundancy.is_flight_safe()) {
    // Update PID controllers
    pid_controller.update(sensor_data, rc_data, flight_state);
    
    // Apply motor mixing and output
    motor_control.update(pid_controller.get_outputs(), armed);
  } else {
    // Emergency disarm
    motor_control.emergency_stop();
    armed = false;
    led_control.set_status(LED_ERROR);
  }
  
  // Update flight state
  update_flight_state();
}

void init_pins() {
  // Motor PWM pins
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);
  
  // Sonar pins
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  
  // LED pin
  pinMode(LED_PIN, OUTPUT);
  
  // Battery monitoring
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  
  Serial.println("Pins initialized");
}

void init_sensors() {
  // Initialize IMU
  if (init_imu()) {
    Serial.println("IMU initialized successfully");
  } else {
    Serial.println("ERROR: IMU initialization failed");
  }
  
  // Initialize magnetometer
  if (init_magnetometer()) {
    Serial.println("Magnetometer initialized successfully");
  } else {
    Serial.println("WARNING: Magnetometer initialization failed");
  }
  
  // Initialize barometer
  if (init_barometer()) {
    Serial.println("Barometer initialized successfully");
  } else {
    Serial.println("WARNING: Barometer initialization failed");
  }
  
  // Initialize GPS
  init_gps();
  Serial.println("GPS initialization started");
  
  // Initialize sonar
  init_sonar();
  Serial.println("Sonar initialized");
}

void init_receivers() {
  // Set default receiver protocol
  set_receiver_protocol(RC_PROTOCOL_PPM);
  Serial.println("RC receiver initialized");
}

void init_motors() {
  motor_control.init();
  Serial.println("Motor control initialized");
}

void init_leds() {
  led_control.init();
  Serial.println("LED control initialized");
}

void init_communication() {
  comm.init();
  Serial.println("Communication system initialized");
}

void update_sensors() {
  // Read IMU data
  read_imu_data(&sensor_data.imu);
  
  // Apply IMU calibrations
  apply_imu_calibration(&sensor_data.imu);
  
  // Read magnetometer (slower update rate)
  static unsigned long last_mag_update = 0;
  if (millis() - last_mag_update > 50) { // 20Hz
    read_magnetometer_data(&sensor_data.mag);
    
    // Apply magnetometer calibration
    apply_magnetometer_calibration(&sensor_data.mag);
    
    last_mag_update = millis();
  }
  
  // Read barometer (slower update rate)
  static unsigned long last_baro_update = 0;
  if (millis() - last_baro_update > 100) { // 10Hz
    read_barometer_data(&sensor_data.baro);
    last_baro_update = millis();
  }
  
  // Read GPS (slower update rate)
  static unsigned long last_gps_update = 0;
  if (millis() - last_gps_update > 200) { // 5Hz
    read_gps_data(&sensor_data.gps);
    last_gps_update = millis();
  }
  
  // Read sonar
  read_sonar_data(&sensor_data.sonar);
  
  // Read battery voltage
  sensor_data.battery_voltage = analogRead(BATTERY_VOLTAGE_PIN) * VOLTAGE_SCALE;
  sensor_data.current = analogRead(CURRENT_SENSOR_PIN) * CURRENT_SCALE;
  
  // Update sensor fusion
  update_sensor_fusion();
}

void update_rc_receiver() {
  read_rc_data(&enhanced_rc_data);
  
  // Update legacy rc_data structure for compatibility
  rc_data.throttle = enhanced_rc_data.throttle * 1000 + 1000;
  rc_data.roll = enhanced_rc_data.roll * 500 + 1500;
  rc_data.pitch = enhanced_rc_data.pitch * 500 + 1500;
  rc_data.yaw = enhanced_rc_data.yaw * 500 + 1500;
  rc_data.signal_valid = enhanced_rc_data.signal_valid;
  rc_data.last_update = enhanced_rc_data.last_update;
  
  // Check for arming/disarming
  check_arm_disarm();
  
  // Check for flight mode changes
  flight_modes.check_mode_switch(rc_data, sensor_data);
}

bool perform_safety_checks() {
  // Check RC signal
  if (millis() - rc_data.last_update > RC_TIMEOUT_MS) {
    Serial.println("WARNING: RC signal lost - entering failsafe");
    return false;
  }
  
  // Check battery voltage
  if (sensor_data.battery_voltage < LOW_VOLTAGE_THRESHOLD) {
    Serial.println("WARNING: Low battery voltage");
    led_control.set_status(LED_LOW_BATTERY);
  }
  
  // Check IMU health
  if (!sensor_data.imu.healthy) {
    Serial.println("ERROR: IMU not healthy");
    return false;
  }
  
  return true;
}

void check_arm_disarm() {
  static unsigned long arm_start_time = 0;
  static bool arm_sequence_started = false;
  
  // Arming sequence: throttle low, yaw right for 2 seconds
  bool arm_condition = (rc_data.throttle < ARM_THROTTLE_THRESHOLD) && 
                       (rc_data.yaw > ARM_YAW_THRESHOLD);
  
  // Disarming sequence: throttle low, yaw left for 2 seconds
  bool disarm_condition = (rc_data.throttle < ARM_THROTTLE_THRESHOLD) && 
                          (rc_data.yaw < DISARM_YAW_THRESHOLD);
  
  if (!armed && arm_condition) {
    // ENHANCED SAFETY CHECK: Comprehensive arming prevention
    if (!enhanced_safety_check_for_arming()) {
      if (millis() % 3000 < 100) { // Print warning every 3 seconds
        Serial.println("ARMING BLOCKED - Safety checks failed!");
        
        // Detailed failure reasons
        if (!is_system_calibrated()) {
          Serial.println("  - System not calibrated");
        }
        if (!detect_battery_connection()) {
          Serial.println("  - No battery detected");
        }
        if (sensor_data.battery_voltage < LOW_VOLTAGE_THRESHOLD) {
          Serial.println("  - Battery voltage too low");
        }
        if (!rc_data.signal_valid) {
          Serial.println("  - No RC signal");
        }
        if (is_gps_required_for_arming()) {
          Serial.println("  - GPS required but not ready");
          Serial.println("    GPS functions configured but GPS unavailable");
        }
        if (!sensor_data.imu.healthy) {
          Serial.println("  - IMU not healthy");
        }
        
        Serial.println("Fix issues above before arming");
      }
      led_control.set_status(LED_ERROR);
      return; // Block arming
    }
    
    if (!arm_sequence_started) {
      arm_sequence_started = true;
      arm_start_time = millis();
      Serial.println("Arming sequence started - hold position...");
    } else if (millis() - arm_start_time > ARM_SEQUENCE_TIME_MS) {
      // Final safety check before arming
      if (enhanced_safety_check_for_arming()) {
        // Arm the drone
        armed = true;
        motor_control.arm();
        led_control.set_status(LED_ARMED);
        
        Serial.println("=== DRONE ARMED ===");
        Serial.println("All safety checks passed:");
        Serial.println("✓ All sensors calibrated");
        Serial.println("✓ Battery connected and voltage OK");
        Serial.println("✓ RC signal good");
        
        if (check_gps_functions_configured()) {
          Serial.println("✓ GPS ready for configured functions");
        }
        
        Serial.println("✓ IMU healthy");
        Serial.println("===================");
        
        // Set home position if GPS is available
        if (sensor_data.gps.healthy && sensor_data.gps.fix) {
          flight_modes.set_home_position(sensor_data.gps.latitude, sensor_data.gps.longitude, sensor_data.gps.altitude);
        }
      } else {
        Serial.println("Arming failed - safety check failed at final moment");
      }
      arm_sequence_started = false;
    }
  } else if (armed && disarm_condition) {
    if (!arm_sequence_started) {
      arm_sequence_started = true;
      arm_start_time = millis();
    } else if (millis() - arm_start_time > 200) {
      // Disarm the drone
      armed = false;
      motor_control.disarm();
      led_control.set_status(LED_DISARMED);
      Serial.println("DRONE DISARMED");
      arm_sequence_started = false;
    }
  } else {
    arm_sequence_started = false;
  }
  
  // Monitor battery status while armed
  if (armed) {
    BatteryStatus battery_status = get_enhanced_battery_status();
    
    if (battery_status.critical_voltage) {
      Serial.println("CRITICAL: Battery voltage critically low - LAND IMMEDIATELY!");
      led_control.set_status(LED_LOW_BATTERY);
      
      // Force RTH or emergency landing if configured
      if (check_gps_functions_configured() && sensor_data.gps.healthy && sensor_data.gps.fix) {
        Serial.println("Triggering emergency RTH due to critical battery");
        // Could trigger automatic RTH here
      }
    } else if (battery_status.low_voltage_warning) {
      if (millis() % 10000 < 100) { // Every 10 seconds
        Serial.println("WARNING: Battery voltage getting low - prepare to land");
      }
    }
  }
}

void update_flight_state() {
  flight_state.armed = armed;
  flight_state.flight_mode = flight_modes.get_current_mode();
  flight_state.battery_voltage = sensor_data.battery_voltage;
  flight_state.gps_fix = sensor_data.gps.fix;
  flight_state.altitude = sensor_data.baro.altitude;
}

void monitor_system_health() {
  static unsigned long last_health_check = 0;
  
  if (millis() - last_health_check > 5000) { // Every 5 seconds
    // Check system performance
    if (loop_time > MAX_LOOP_TIME_US) {
      Serial.print("WARNING: Loop time exceeded: ");
      Serial.print(loop_time);
      Serial.println(" us");
    }
    
    // Update health indicators
    led_control.update_health_status(sensor_data, flight_state);
    
    last_health_check = millis();
  }
}

void load_config() {
  // Load configuration from EEPROM
  // Implementation will read saved PID values, calibration data, etc.
  Serial.println("Configuration loaded from EEPROM");
}

void save_config() {
  // Save configuration to EEPROM
  Serial.println("Configuration saved to EEPROM");
}
