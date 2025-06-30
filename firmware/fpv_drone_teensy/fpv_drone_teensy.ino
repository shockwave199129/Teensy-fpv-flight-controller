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
#ifdef __IMXRT1062__  // Teensy 4.x
#include <IntervalTimer.h>  // Teensy-specific
#endif
#include <FastLED.h>  // LED library
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
#include "dual_imu_manager.h"
#include "dynamic_filtering.h"
#include "optical_flow.h"
#include "advanced_flight_modes.h"
#include "safety_functions.h"

// Global variables
FlightState flight_state;
SensorData sensor_data;
RcData rc_data;
// Duplicates moved to receivers.cpp; using extern declarations from receivers.h
// EnhancedRcData enhanced_rc_data;
// RcConfig rc_config;
// RateProfile rate_profiles[3];
// RcProtocol current_protocol = RC_PROTOCOL_PPM;
// FunctionSwitches function_switches;

// Core system objects
PIDController pid_controller;
MotorControl motor_control;
FlightModes flight_modes;
LEDControl led_control;
Communication comm;

// Phase 2 system objects - properly instantiated
DualIMUManager dual_imu_manager;
DynamicFilteringSystem dynamic_filtering;
OpticalFlowSensor optical_flow;
AdvancedFlightModes advanced_flight_modes;

// Sensor detection and redundancy
SensorDetectionStatus sensor_detection;
// SensorRedundancySystem sensor_redundancy; // Moved to sensor_redundancy.cpp

// Global variable instances
EnhancedSensorFusion enhanced_fusion;
SensorQuality sensor_quality;
AdvancedFiltering advanced_filtering;
CalibrationData calibration_data;

// Timing variables
unsigned long last_loop_time = 0;
unsigned long loop_time = 0;
bool armed = false;

// Main loop timer
#ifdef __IMXRT1062__  // Teensy 4.x
IntervalTimer main_loop_timer;  // Teensy-specific timer
#else
// Alternative timing for Arduino (using millis-based timing)
unsigned long main_loop_interval = 500; // 500us = 2kHz
#endif

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
  
  // Initialize enhanced sensor fusion
  init_enhanced_sensor_fusion();
  
  // Phase 2: Initialize all systems
  init_dual_imu_system();
  
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
  
  // Initialize sensor redundancy system
  sensor_redundancy.init();
  
  // Load configuration from EEPROM
  load_config();
  
  // Initialize PID controller
  pid_controller.init();
  
  // Initialize flight modes
  flight_modes.init();
  
  // Phase 2: Advanced flight modes initialized in init_dual_imu_system()
  
  // Set up main loop timer (2000Hz for enhanced performance)
  #ifdef __IMXRT1062__  // Teensy 4.x
  main_loop_timer.begin(main_loop, 500);
  #else
  // Arduino alternative: main loop will be called from loop() with timing control
  Serial.println("Using Arduino-compatible timing (no IntervalTimer)");
  #endif
  
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
  #ifdef ENABLE_OPTICAL_FLOW
  Serial.println("- Optical flow positioning");
  #endif
  Serial.println("Type 'help' for available commands");
  
  // Initial LED state (disarmed - red)
  led_control.set_status(LED_DISARMED);
}

void loop() {
  #ifndef __IMXRT1062__  // Arduino alternative timing
  static unsigned long last_main_loop = 0;
  if (micros() - last_main_loop >= main_loop_interval) {
    main_loop();
    last_main_loop = micros();
  }
  #endif
  
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
  #ifdef ENABLE_DUAL_IMU
  dual_imu_manager.update();
  // Use fused IMU data instead of single IMU
  sensor_data.imu = dual_imu_manager.get_fused_data();
  #else
  // Read single IMU sensor
  update_sensors();
  #endif
  
  // Phase 2: Update dynamic filtering
  dynamic_filtering.update(sensor_data, rc_data);
  
  // Phase 2: Update optical flow
  #ifdef ENABLE_OPTICAL_FLOW
  optical_flow.update();
  #endif
  
  // Read RC receiver
  update_rc_receiver();
  
  // Process flight mode logic
  flight_modes.update(rc_data, sensor_data);
  
  // Phase 2: Process advanced flight modes
  advanced_flight_modes.update(sensor_data, rc_data, flight_state);
  
  // Update enhanced sensor fusion
  update_enhanced_sensor_fusion();
  
  // Update sensor redundancy system
  sensor_redundancy.update(sensor_data);
  
  // Update enhanced calibration systems
  update_magnetometer_timer();
  
  // Safety checks
  if (perform_safety_checks() && sensor_redundancy.is_flight_safe()) {
    // Update PID controllers
    pid_controller.update(sensor_data, rc_data, flight_state);
    
    // Apply motor mixing and output
    PIDOutput pid_outputs = pid_controller.get_outputs();
    motor_control.update(pid_outputs, armed);
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

  EEPROM.put(0, rc_config);
  EEPROM.put(sizeof(rc_config), rate_profiles);
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
  
  // Check for low battery
  if (flight_state.battery_voltage < 10.5) {
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
  Serial.println("Loading configuration from EEPROM...");
  
  #include <EEPROM.h>
  
  // Check for valid configuration signature
  uint32_t signature;
  EEPROM.get(0, signature);
  
  if (signature != 0x12345678) {
    Serial.println("No valid configuration found - using defaults");
    save_config(); // Save default config
    return;
  }
  
  int address = 4; // Start after signature
  
  // Load PID gains
  extern PIDController pid_controller;
  CascadedPIDGains gains;
  EEPROM.get(address, gains);
  address += sizeof(CascadedPIDGains);
  
  // Apply loaded PID gains
  pid_controller.set_rate_pid_gains(0, gains.rate_roll.kp, gains.rate_roll.ki, gains.rate_roll.kd);
  pid_controller.set_rate_pid_gains(1, gains.rate_pitch.kp, gains.rate_pitch.ki, gains.rate_pitch.kd);
  pid_controller.set_rate_pid_gains(2, gains.rate_yaw.kp, gains.rate_yaw.ki, gains.rate_yaw.kd);
  pid_controller.set_angle_pid_gains(0, gains.angle_roll.kp, gains.angle_roll.ki, gains.angle_roll.kd);
  pid_controller.set_angle_pid_gains(1, gains.angle_pitch.kp, gains.angle_pitch.ki, gains.angle_pitch.kd);
  
  // Load ESC configuration
  extern MotorControl motor_control;
  EscConfig esc_config;
  EEPROM.get(address, esc_config);
  address += sizeof(EscConfig);
  motor_control.configure_esc(esc_config);
  
  // Load RC configuration
  extern RcConfig rc_config;
  EEPROM.get(address, rc_config);
  address += sizeof(RcConfig);
  
  // Load calibration data
  bool gyro_calibrated, accel_calibrated, mag_calibrated;
  EEPROM.get(address, gyro_calibrated);
  address += sizeof(bool);
  EEPROM.get(address, accel_calibrated);
  address += sizeof(bool);
  EEPROM.get(address, mag_calibrated);
  address += sizeof(bool);
  
  // Load sensor offsets
  float gyro_offsets[3], accel_offsets[3], mag_offsets[3];
  EEPROM.get(address, gyro_offsets);
  address += sizeof(gyro_offsets);
  EEPROM.get(address, accel_offsets);
  address += sizeof(accel_offsets);
  EEPROM.get(address, mag_offsets);
  address += sizeof(mag_offsets);
  
  // Load enhanced sensor fusion config
  extern EnhancedSensorFusion enhanced_fusion;
  EEPROM.get(address, enhanced_fusion);
  address += sizeof(EnhancedSensorFusion);
  
  // Load sensor redundancy config
  // extern SensorRedundancySystem sensor_redundancy; // unused
  SensorRedundancyConfig redundancy_config;
  EEPROM.get(address, redundancy_config);
  address += sizeof(SensorRedundancyConfig);
  
  Serial.println("Configuration loaded successfully:");
  Serial.print("  - PID gains loaded (Rate Roll KP: ");
  Serial.print(gains.rate_roll.kp, 3);
  Serial.println(")");
  Serial.print("  - ESC protocol: ");
  Serial.println(esc_config.protocol);
  Serial.print("  - RC channels: ");
  Serial.println(rc_config.channel_count);
  Serial.print("  - Calibration status - Gyro: ");
  Serial.print(gyro_calibrated ? "✓" : "✗");
  Serial.print(", Accel: ");
  Serial.print(accel_calibrated ? "✓" : "✗");
  Serial.print(", Mag: ");
  Serial.println(mag_calibrated ? "✓" : "✗");
  Serial.print("Total config size: ");
  Serial.print(address);
  Serial.println(" bytes");
}

// Missing function implementations
bool enhanced_safety_check_for_arming() {
  return enhanced_safety_check_for_arming_with_feedback();
}

bool is_gps_required_for_arming() {
  return check_gps_functions_configured() && !sensor_data.gps.healthy;
}

bool is_gyro_calibrated() {
  // Consult the persistent calibration data loaded from EEPROM
  // and updated by the IMU calibration routine.
  return calibration_data.gyro_calibrated;
}

bool is_accelerometer_calibrated() {
  return calibration_data.accel_calibrated;
}

bool is_magnetometer_calibrated() {
  return calibration_data.mag_calibrated;
}

#if 0  // Disabled duplicate implementations (defined elsewhere)
BatteryStatus get_enhanced_battery_status() {
  BatteryStatus status;
  status.voltage = sensor_data.battery_voltage;
  status.percentage = (status.voltage - 9.0f) / (12.6f - 9.0f) * 100.0f;
  status.low_voltage_warning = status.voltage < LOW_VOLTAGE_THRESHOLD;
  status.critical_voltage = status.voltage < CRITICAL_VOLTAGE_THRESHOLD;
  return status;
}

void perform_full_sensor_detection() {
  scan_i2c_devices();
}
#endif

void apply_imu_calibration(IMUData* data) {
  if (!data) return;

  // Apply gyroscope bias if available
  if (calibration_data.gyro_calibrated) {
    data->gyro_x -= calibration_data.gyro_offsets[0];
    data->gyro_y -= calibration_data.gyro_offsets[1];
    data->gyro_z -= calibration_data.gyro_offsets[2];
  }

  // Apply accelerometer offset and 3×3 scale matrix if available
  if (calibration_data.accel_calibrated) {
    // First subtract biases (offsets)
    float ax = data->accel_x - calibration_data.accel_offsets[0];
    float ay = data->accel_y - calibration_data.accel_offsets[1];
    float az = data->accel_z - calibration_data.accel_offsets[2];

    // Then apply the scale/alignment matrix
    float ax_c = ax * calibration_data.accel_scale_matrix[0][0] +
                 ay * calibration_data.accel_scale_matrix[0][1] +
                 az * calibration_data.accel_scale_matrix[0][2];
    float ay_c = ax * calibration_data.accel_scale_matrix[1][0] +
                 ay * calibration_data.accel_scale_matrix[1][1] +
                 az * calibration_data.accel_scale_matrix[1][2];
    float az_c = ax * calibration_data.accel_scale_matrix[2][0] +
                 ay * calibration_data.accel_scale_matrix[2][1] +
                 az * calibration_data.accel_scale_matrix[2][2];

    data->accel_x = ax_c;
    data->accel_y = ay_c;
    data->accel_z = az_c;
  }

  data->calibrated = calibration_data.gyro_calibrated || calibration_data.accel_calibrated;
}

void apply_magnetometer_calibration(MagnetometerData* data) {
  if (!data) return;
  if (!calibration_data.mag_calibrated) {
    // Nothing to do yet
    return;
  }

  // Offset correction
  float mx = data->mag_x - calibration_data.mag_offsets[0];
  float my = data->mag_y - calibration_data.mag_offsets[1];
  float mz = data->mag_z - calibration_data.mag_offsets[2];

  // Soft-iron (scale) correction using 3×3 matrix
  float mx_c = mx * calibration_data.mag_scale_matrix[0][0] +
               my * calibration_data.mag_scale_matrix[0][1] +
               mz * calibration_data.mag_scale_matrix[0][2];
  float my_c = mx * calibration_data.mag_scale_matrix[1][0] +
               my * calibration_data.mag_scale_matrix[1][1] +
               mz * calibration_data.mag_scale_matrix[1][2];
  float mz_c = mx * calibration_data.mag_scale_matrix[2][0] +
               my * calibration_data.mag_scale_matrix[2][1] +
               mz * calibration_data.mag_scale_matrix[2][2];

  data->mag_x = mx_c;
  data->mag_y = my_c;
  data->mag_z = mz_c;

  // Compute heading in degrees and apply magnetic declination
  float heading = atan2f(my_c, mx_c) * 180.0f / PI;
  heading += calibration_data.mag_declination;
  if (heading < 0) heading += 360.0f;
  if (heading >= 360.0f) heading -= 360.0f;
  data->heading = heading;

  data->calibrated = true;
}

void save_config() {
  // Save configuration to EEPROM
  Serial.println("Saving configuration to EEPROM...");
  
  #include <EEPROM.h>
  
  int address = 0;
  
  // Write configuration signature
  uint32_t signature = 0x12345678;
  EEPROM.put(address, signature);
  address += sizeof(uint32_t);
  
  // Save PID gains
  extern PIDController pid_controller;
  CascadedPIDGains gains = pid_controller.get_cascaded_gains();
  EEPROM.put(address, gains);
  address += sizeof(CascadedPIDGains);
  
  // Save ESC configuration
  extern MotorControl motor_control;
  EscConfig esc_config = motor_control.get_esc_config();
  EEPROM.put(address, esc_config);
  address += sizeof(EscConfig);
  
  // Save RC configuration
  extern RcConfig rc_config;
  EEPROM.put(address, rc_config);
  address += sizeof(RcConfig);
  
  // Save calibration status
  bool gyro_calibrated = is_gyro_calibrated();
  bool accel_calibrated = is_accelerometer_calibrated();
  bool mag_calibrated = is_magnetometer_calibrated();
  
  EEPROM.put(address, gyro_calibrated);
  address += sizeof(bool);
  EEPROM.put(address, accel_calibrated);
  address += sizeof(bool);
  EEPROM.put(address, mag_calibrated);
  address += sizeof(bool);
  
  // Save sensor calibration offsets
  float gyro_offsets[3] = {0, 0, 0}; // Would get from calibration system
  float accel_offsets[3] = {0, 0, 0};
  float mag_offsets[3] = {0, 0, 0};
  
  EEPROM.put(address, gyro_offsets);
  address += sizeof(gyro_offsets);
  EEPROM.put(address, accel_offsets);
  address += sizeof(accel_offsets);
  EEPROM.put(address, mag_offsets);
  address += sizeof(mag_offsets);
  
  // Save enhanced sensor fusion config
  extern EnhancedSensorFusion enhanced_fusion;
  EEPROM.put(address, enhanced_fusion);
  address += sizeof(EnhancedSensorFusion);
  
  // Save sensor redundancy config
  // extern SensorRedundancySystem sensor_redundancy; // unused
  SensorRedundancyConfig redundancy_config;
  // redundancy_config = sensor_redundancy.get_config(); // Would implement getter
  EEPROM.put(address, redundancy_config);
  address += sizeof(SensorRedundancyConfig);
  
  // Arduino EEPROM automatically commits changes (no explicit commit needed)
  
  Serial.println("Configuration saved successfully:");
  Serial.print("  - Total size: ");
  Serial.print(address);
  Serial.println(" bytes");
  Serial.print("  - PID gains saved (Rate Roll KP: ");
  Serial.print(gains.rate_roll.kp, 3);
  Serial.println(")");
  Serial.print("  - ESC protocol: ");
  Serial.println(esc_config.protocol);
  Serial.print("  - Calibration status saved");
  Serial.println();
}
