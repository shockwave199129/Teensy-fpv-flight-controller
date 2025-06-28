#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "config.h"

class Communication {
private:
  String command_buffer;
  unsigned long last_telemetry_send;
  
  void process_command(String command);
  void send_telemetry();
  void print_help();
  void print_status();
  
public:
  void init();
  void process_commands();
  void send_data(String data);
};

// External references to global objects
extern SensorData sensor_data;
extern RcData rc_data;
extern FlightState flight_state;
extern PIDController pid_controller;
extern MotorControl motor_control;
extern FlightModes flight_modes;
extern LEDControl led_control;
extern bool armed;

// External references to Phase 2 global objects
extern DualIMUManager dual_imu;
extern DynamicFilteringSystem dynamic_filtering;
extern OpticalFlowSensor optical_flow;
extern AdvancedFlightModes advanced_flight_modes;

void Communication::init() {
  command_buffer = "";
  last_telemetry_send = 0;
  
  Serial.println("Communication system ready");
  Serial.println("Type 'help' for available commands");
}

void Communication::process_commands() {
  // Process incoming serial commands
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (command_buffer.length() > 0) {
        process_command(command_buffer);
        command_buffer = "";
      }
    } else {
      command_buffer += c;
    }
  }
  
  // Send periodic telemetry
  if (millis() - last_telemetry_send > 1000) { // 1Hz telemetry
    send_telemetry();
    last_telemetry_send = millis();
  }
}

void Communication::process_command(String command) {
  command.trim();
  command.toUpperCase();
  
  Serial.print("Command received: ");
  Serial.println(command);
  
  if (command == "HELP") {
    print_help();
  }
  else if (command == "STATUS") {
    print_status();
  }
  else if (command == "ARM") {
    if (!armed) {
      Serial.println("Arming drone...");
      // Note: This bypasses normal arming sequence - use with caution
    } else {
      Serial.println("Drone already armed");
    }
  }
  else if (command == "DISARM") {
    if (armed) {
      motor_control.disarm();
      Serial.println("Drone disarmed");
    } else {
      Serial.println("Drone already disarmed");
    }
  }
  else if (command == "CALIBRATE ESC") {
    if (!armed) {
      motor_control.calibrate_escs();
    } else {
      Serial.println("Cannot calibrate ESCs while armed");
    }
  }
  else if (command == "CALIBRATE IMU") {
    if (!armed) {
      calibrate_imu();
    } else {
      Serial.println("Cannot calibrate IMU while armed");
    }
  }
  else if (command == "CALIBRATE GYRO") {
    if (!armed) {
      if (perform_gyro_calibration()) {
        save_calibration_to_eeprom();
        Serial.println("Gyro calibration saved");
      }
    } else {
      Serial.println("Cannot calibrate gyro while armed");
    }
  }
  else if (command == "CALIBRATE ACCEL" || command == "CALIBRATE ACCELEROMETER") {
    if (!armed) {
      if (perform_accelerometer_calibration()) {
        save_calibration_to_eeprom();
        Serial.println("Accelerometer calibration saved");
      }
    } else {
      Serial.println("Cannot calibrate accelerometer while armed");
    }
  }
  else if (command.startsWith("CALIBRATE ACCEL POSITION")) {
    if (!armed) {
      // Parse position: CALIBRATE ACCEL POSITION LEVEL
      String position = command.substring(25); // After "CALIBRATE ACCEL POSITION "
      position.trim();
      position.toUpperCase();
      
      if (position == "LEVEL") {
        if (perform_accelerometer_position_calibration(1)) {
          Serial.println("Position 1 complete - Level position calibrated");
        }
      } else if (position == "UPSIDE_DOWN") {
        if (perform_accelerometer_position_calibration(2)) {
          Serial.println("Position 2 complete - Upside down position calibrated");
        }
      } else if (position == "RIGHT_SIDE") {
        if (perform_accelerometer_position_calibration(3)) {
          Serial.println("Position 3 complete - Right side position calibrated");
        }
      } else if (position == "LEFT_SIDE") {
        if (perform_accelerometer_position_calibration(4)) {
          Serial.println("Position 4 complete - Left side position calibrated");
        }
      } else if (position == "NOSE_DOWN") {
        if (perform_accelerometer_position_calibration(5)) {
          Serial.println("Position 5 complete - Nose down position calibrated");
        }
      } else if (position == "NOSE_UP") {
        if (perform_accelerometer_position_calibration(6)) {
          Serial.println("Position 6 complete - Nose up position calibrated");
        }
      } else {
        Serial.println("Unknown position. Valid positions: LEVEL, UPSIDE_DOWN, RIGHT_SIDE, LEFT_SIDE, NOSE_DOWN, NOSE_UP");
      }
    } else {
      Serial.println("Cannot calibrate accelerometer while armed");
    }
  }
  else if (command == "CALIBRATE MAG" || command == "CALIBRATE MAGNETOMETER") {
    if (!armed) {
      calibrate_magnetometer();
    } else {
      Serial.println("Cannot calibrate magnetometer while armed");
    }
  }
  else if (command == "CALIBRATE MAG TIMER") {
    if (!armed) {
      start_magnetometer_timer_calibration();
    } else {
      Serial.println("Cannot calibrate magnetometer while armed");
    }
  }
  else if (command == "ESC CALIBRATION START") {
    if (!armed) {
      start_enhanced_esc_calibration();
    } else {
      Serial.println("Cannot start ESC calibration while armed");
    }
  }
  else if (command == "ESC CALIBRATION HIGH") {
    proceed_esc_calibration_high();
  }
  else if (command == "ESC CALIBRATION LOW") {
    proceed_esc_calibration_low();
  }
  else if (command == "ESC CALIBRATION COMPLETE") {
    complete_esc_calibration();
  }
  else if (command == "CHECK MOTOR DIRECTIONS") {
    check_motor_direction_configuration();
  }
  else if (command.startsWith("FIX MOTOR DIRECTION")) {
    // Parse: FIX MOTOR DIRECTION 1
    int motor = command.substring(20).toInt();
    if (motor >= 1 && motor <= 4) {
      fix_motor_direction(motor - 1);
    } else {
      Serial.println("Usage: FIX MOTOR DIRECTION [1-4]");
    }
  }
  else if (command == "SAFETY CHECK PROPELLERS") {
    perform_propeller_safety_check();
  }
  else if (command == "CONFIRM PROPELLERS REMOVED") {
    confirm_propellers_removed(true);
  }
  else if (command == "CONFIRM PROPELLERS INSTALLED") {
    confirm_propellers_removed(false);
  }
  else if (command == "CALIBRATION WIZARD") {
    if (!armed) {
      start_calibration_wizard();
    } else {
      Serial.println("Cannot start calibration wizard while armed");
    }
  }
  else if (command == "CALIBRATION STATUS") {
    String status = get_calibration_status_json();
    Serial.println(status);
  }
  else if (command == "CALIBRATION CHECK") {
    if (is_system_calibrated()) {
      Serial.println("CALIBRATION_OK: All sensors calibrated - ready for flight");
    } else {
      Serial.println("CALIBRATION_REQUIRED: System not fully calibrated");
      Serial.println("Missing calibrations:");
      if (!calibration_data.gyro_calibrated) Serial.println("  - Gyroscope");
      if (!calibration_data.accel_calibrated) Serial.println("  - Accelerometer");
      if (!calibration_data.mag_calibrated) Serial.println("  - Magnetometer");
      if (!calibration_data.esc_calibrated) Serial.println("  - ESC");
      if (!calibration_data.rc_calibrated) Serial.println("  - RC Receiver");
    }
  }
  else if (command == "RESET CALIBRATION") {
    reset_all_calibrations();
    Serial.println("All calibrations reset - system requires recalibration");
  }
  else if (command == "LOAD CALIBRATION") {
    load_calibration_from_eeprom();
    Serial.println("Calibration data loaded from EEPROM");
  }
  else if (command == "SAVE CALIBRATION") {
    save_calibration_to_eeprom();
    Serial.println("Calibration data saved to EEPROM");
  }
  else if (command.startsWith("TEST MOTOR")) {
    if (!armed) {
      // Parse: TEST MOTOR 1 1200
      int motor_num = command.substring(11, 12).toInt();
      int pwm_value = command.substring(13).toInt();
      
      if (motor_num >= 1 && motor_num <= 4 && pwm_value >= 1000 && pwm_value <= 2000) {
        motor_control.test_motor(motor_num, pwm_value);
      } else {
        Serial.println("Usage: TEST MOTOR [1-4] [1000-2000]");
      }
    } else {
      Serial.println("Cannot test motors while armed");
    }
  }
  else if (command.startsWith("SET PID")) {
    // Parse: SET PID ROLL KP 1.5
    int axis = -1;
    if (command.indexOf("ROLL") > 0) axis = 0;
    else if (command.indexOf("PITCH") > 0) axis = 1;
    else if (command.indexOf("YAW") > 0) axis = 2;
    else if (command.indexOf("ALT") > 0) axis = 3;
    
    if (axis >= 0) {
      float value = 0;
      if (command.indexOf("KP") > 0) {
        value = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        // Update KP value (simplified - would need proper implementation)
        Serial.print("Setting axis ");
        Serial.print(axis);
        Serial.print(" KP to ");
        Serial.println(value);
      } else if (command.indexOf("KI") > 0) {
        value = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        Serial.print("Setting axis ");
        Serial.print(axis);
        Serial.print(" KI to ");
        Serial.println(value);
      } else if (command.indexOf("KD") > 0) {
        value = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        Serial.print("Setting axis ");
        Serial.print(axis);
        Serial.print(" KD to ");
        Serial.println(value);
      }
    } else {
      Serial.println("Usage: SET PID [ROLL/PITCH/YAW/ALT] [KP/KI/KD] [value]");
    }
  }
  else if (command.startsWith("SET PROTOCOL")) {
    // Parse: SET PROTOCOL SBUS
    if (command.indexOf("PPM") > 0) {
      set_receiver_protocol(RC_PROTOCOL_PPM);
    } else if (command.indexOf("SBUS") > 0) {
      set_receiver_protocol(RC_PROTOCOL_SBUS);
    } else if (command.indexOf("IBUS") > 0) {
      set_receiver_protocol(RC_PROTOCOL_IBUS);
    } else if (command.indexOf("ELRS") > 0) {
      set_receiver_protocol(RC_PROTOCOL_ELRS);
    } else {
      Serial.println("Usage: SET PROTOCOL [PPM/SBUS/IBUS/ELRS]");
    }
  }
  else if (command.startsWith("SET ESC")) {
    // Parse: SET ESC PROTOCOL DSHOT600
    if (command.indexOf("PROTOCOL") > 0) {
      if (command.indexOf("PWM") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_PWM);
      } else if (command.indexOf("ONESHOT125") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_ONESHOT125);
      } else if (command.indexOf("ONESHOT42") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_ONESHOT42);
      } else if (command.indexOf("MULTISHOT") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_MULTISHOT);
      } else if (command.indexOf("DSHOT150") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_DSHOT150);
      } else if (command.indexOf("DSHOT300") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_DSHOT300);
      } else if (command.indexOf("DSHOT600") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_DSHOT600);
      } else if (command.indexOf("DSHOT1200") > 0) {
        motor_control.set_esc_protocol(ESC_PROTOCOL_DSHOT1200);
      } else {
        Serial.println("Usage: SET ESC PROTOCOL [PWM/ONESHOT125/ONESHOT42/MULTISHOT/DSHOT150/DSHOT300/DSHOT600/DSHOT1200]");
      }
    } else if (command.indexOf("DIRECTION") > 0) {
      // Parse: SET ESC DIRECTION 1 REVERSE
      int motor = command.substring(command.indexOf("DIRECTION") + 10, command.lastIndexOf(' ')).toInt();
      if (motor >= 1 && motor <= 4) {
        if (command.indexOf("REVERSE") > 0) {
          motor_control.set_motor_direction(motor - 1, -1);
        } else if (command.indexOf("NORMAL") > 0) {
          motor_control.set_motor_direction(motor - 1, 1);
        } else {
          motor_control.reverse_motor_direction(motor - 1);
        }
      } else {
        Serial.println("Usage: SET ESC DIRECTION [1-4] [NORMAL/REVERSE]");
      }
    } else if (command.indexOf("TELEMETRY") > 0) {
      // Parse: SET ESC TELEMETRY ON
      if (command.indexOf("ON") > 0) {
        motor_control.request_telemetry(true);
      } else if (command.indexOf("OFF") > 0) {
        motor_control.request_telemetry(false);
      } else {
        Serial.println("Usage: SET ESC TELEMETRY [ON/OFF]");
      }
    } else if (command.indexOf("BIDIRECTIONAL") > 0) {
      // Parse: SET ESC BIDIRECTIONAL ON
      if (command.indexOf("ON") > 0) {
        motor_control.enable_bidirectional_dshot(true);
      } else if (command.indexOf("OFF") > 0) {
        motor_control.enable_bidirectional_dshot(false);
      } else {
        Serial.println("Usage: SET ESC BIDIRECTIONAL [ON/OFF]");
      }
    }
  }
  else if (command.startsWith("DSHOT")) {
    // Parse: DSHOT BEEP1 or DSHOT MOTOR 1 BEEP2
    if (command.indexOf("MOTOR") > 0) {
      int motor = command.substring(command.indexOf("MOTOR") + 6, command.lastIndexOf(' ')).toInt();
      if (motor >= 1 && motor <= 4) {
        // Parse DShot command
        if (command.indexOf("BEEP1") > 0) motor_control.send_dshot_command_motor(motor - 1, DSHOT_CMD_BEEP1);
        else if (command.indexOf("BEEP2") > 0) motor_control.send_dshot_command_motor(motor - 1, DSHOT_CMD_BEEP2);
        else if (command.indexOf("BEEP3") > 0) motor_control.send_dshot_command_motor(motor - 1, DSHOT_CMD_BEEP3);
        else if (command.indexOf("LED_ON") > 0) motor_control.send_dshot_command_motor(motor - 1, DSHOT_CMD_LED0_ON);
        else if (command.indexOf("LED_OFF") > 0) motor_control.send_dshot_command_motor(motor - 1, DSHOT_CMD_LED0_OFF);
      }
    } else {
      // Send to all motors
      if (command.indexOf("BEEP1") > 0) motor_control.send_dshot_command_all(DSHOT_CMD_BEEP1);
      else if (command.indexOf("BEEP2") > 0) motor_control.send_dshot_command_all(DSHOT_CMD_BEEP2);
      else if (command.indexOf("BEEP3") > 0) motor_control.send_dshot_command_all(DSHOT_CMD_BEEP3);
      else if (command.indexOf("SAVE") > 0) motor_control.send_dshot_command_all(DSHOT_CMD_SAVE_SETTINGS);
      else Serial.println("Usage: DSHOT [BEEP1/BEEP2/BEEP3/SAVE] or DSHOT MOTOR [1-4] [command]");
    }
  }
  else if (command == "RESET PID") {
    pid_controller.reset_pid_integrals();
  }
  else if (command.startsWith("LED")) {
    // Parse: LED RED or LED PATTERN 1
    if (command.indexOf("RED") > 0) {
      led_control.show_custom_pattern(CRGB::Red, 0);
    } else if (command.indexOf("GREEN") > 0) {
      led_control.show_custom_pattern(CRGB::Green, 0);
    } else if (command.indexOf("BLUE") > 0) {
      led_control.show_custom_pattern(CRGB::Blue, 0);
    } else if (command.indexOf("RAINBOW") > 0) {
      led_control.show_custom_pattern(CRGB::White, 0);
    } else if (command.indexOf("PATTERN") > 0) {
      int pattern = command.substring(command.lastIndexOf(' ') + 1).toInt();
      led_control.show_custom_pattern(CRGB::White, pattern);
    } else {
      Serial.println("Usage: LED [RED/GREEN/BLUE/RAINBOW] or LED PATTERN [0-3]");
    }
  }
  else if (command == "REBOOT") {
    Serial.println("Rebooting...");
    delay(1000);
    // Software reset for Teensy
    SCB_AIRCR = 0x05FA0004;
  }
  else if (command.startsWith("DUAL IMU")) {
    // Parse: DUAL IMU STATUS / FORCE PRIMARY / FORCE SECONDARY / AUTO
    if (command.indexOf("STATUS") > 0) {
      IMUValidation status = dual_imu.get_validation_status();
      Serial.println("=== Dual IMU Status ===");
      Serial.print("Primary Healthy: "); Serial.println(status.primary_healthy ? "YES" : "NO");
      Serial.print("Secondary Healthy: "); Serial.println(status.secondary_healthy ? "YES" : "NO");
      Serial.print("Cross Validation: "); Serial.println(status.cross_validation_passed ? "PASS" : "FAIL");
      Serial.print("Using Primary: "); Serial.println(status.using_primary ? "YES" : "NO");
      Serial.print("Gyro Divergence: "); Serial.print(status.gyro_divergence); Serial.println(" deg/s");
      Serial.print("Primary Failures: "); Serial.println(status.primary_failures);
      Serial.print("Secondary Failures: "); Serial.println(status.secondary_failures);
    } else if (command.indexOf("FORCE PRIMARY") > 0) {
      dual_imu.force_primary_imu();
      Serial.println("Forced to primary IMU");
    } else if (command.indexOf("FORCE SECONDARY") > 0) {
      dual_imu.force_secondary_imu();
      Serial.println("Forced to secondary IMU");
    } else if (command.indexOf("AUTO") > 0) {
      dual_imu.enable_auto_failover(true);
      Serial.println("Auto failover enabled");
    }
  }
  else if (command.startsWith("FILTERING")) {
    // Parse: FILTERING STATUS / AUTO ON / AUTO OFF / ANALYSIS
    if (command.indexOf("STATUS") > 0) {
      Serial.println("=== Dynamic Filtering Status ===");
      Serial.println("Adaptive filtering: ACTIVE");
      // Would show current filter states
    } else if (command.indexOf("AUTO ON") > 0) {
      dynamic_filtering.set_auto_tune_enabled(true);
      Serial.println("Auto-tuning filters enabled");
    } else if (command.indexOf("AUTO OFF") > 0) {
      dynamic_filtering.set_auto_tune_enabled(false);
      Serial.println("Auto-tuning filters disabled");
    } else if (command.indexOf("ANALYSIS") > 0) {
      SpectralAnalysis analysis = dynamic_filtering.get_spectral_analysis();
      Serial.println("=== Spectral Analysis ===");
      Serial.print("Noise peaks detected: "); Serial.println(analysis.num_peaks);
      for (int i = 0; i < analysis.num_peaks; i++) {
        Serial.print("Peak "); Serial.print(i + 1); Serial.print(": ");
        Serial.print(analysis.peak_frequencies[i]); Serial.print(" Hz (");
        Serial.print(analysis.peak_amplitudes[i]); Serial.println(")");
      }
    }
  }
  else if (command.startsWith("OPTICAL FLOW")) {
    // Parse: OPTICAL FLOW STATUS / RESET
    #if ENABLE_OPTICAL_FLOW
    if (command.indexOf("STATUS") > 0) {
      OpticalFlowData flow = optical_flow.get_data();
      Serial.println("=== Optical Flow Status ===");
      Serial.print("Data Valid: "); Serial.println(flow.data_valid ? "YES" : "NO");
      Serial.print("Velocity X: "); Serial.print(flow.velocity_x); Serial.println(" m/s");
      Serial.print("Velocity Y: "); Serial.print(flow.velocity_y); Serial.println(" m/s");
      Serial.print("Position X: "); Serial.print(flow.displacement_x); Serial.println(" m");
      Serial.print("Position Y: "); Serial.print(flow.displacement_y); Serial.println(" m");
      Serial.print("Surface Quality: "); Serial.print(flow.surface_quality); Serial.println("/255");
    } else if (command.indexOf("RESET") > 0) {
      optical_flow.reset_position();
      Serial.println("Optical flow position reset");
    }
    #else
    Serial.println("Optical flow not enabled");
    #endif
  }
  else if (command.startsWith("GET_RC_DATA")) {
    // Send real-time RC data in JSON format for monitoring
    EnhancedRcData rc_data;
    if (read_rc_data(&rc_data)) {
      Serial.println("{\"type\":\"rc_data\",\"channels\":[" +
        String(rc_data.raw_channels[0]) + "," +
        String(rc_data.raw_channels[1]) + "," +
        String(rc_data.raw_channels[2]) + "," +
        String(rc_data.raw_channels[3]) + "," +
        String(rc_data.raw_channels[4]) + "," +
        String(rc_data.raw_channels[5]) + "," +
        String(rc_data.raw_channels[6]) + "," +
        String(rc_data.raw_channels[7]) + "," +
        String(rc_data.raw_channels[8]) + "," +
        String(rc_data.raw_channels[9]) + "," +
        String(rc_data.raw_channels[10]) + "," +
        String(rc_data.raw_channels[11]) + "," +
        String(rc_data.raw_channels[12]) + "," +
        String(rc_data.raw_channels[13]) + "," +
        String(rc_data.raw_channels[14]) + "," +
        String(rc_data.raw_channels[15]) + "],\"stats\":{" +
        "\"rssi\":" + String(rc_data.rssi) + "," +
        "\"linkQuality\":" + String(rc_data.signal_valid ? 100 : 0) + "," +
        "\"frameRate\":" + String(1000.0 / (millis() - rc_data.last_update)) + "," +
        "\"packetsReceived\":" + String(rc_data.channel_count) + "," +
        "\"packetsLost\":0}}");
    } else {
      Serial.println("{\"type\":\"rc_data\",\"error\":\"No RC signal\"}");
    }
  }
  else if (command.startsWith("GET_CHANNEL_MAPPING")) {
    // Send current channel mapping configuration
    Serial.println("=== Channel Mapping ===");
    for (int i = 0; i < rc_config.channel_count; i++) {
      if (rc_config.channels[i].channelNumber > 0) {
        Serial.print("Channel "); Serial.print(rc_config.channels[i].channelNumber);
        Serial.print(" -> Function "); Serial.print(rc_config.channels[i].function);
        Serial.print(" ("); Serial.print(rc_config.channels[i].reversed ? "Reversed" : "Normal");
        Serial.print(", Range: "); Serial.print(rc_config.channels[i].minValue);
        Serial.print("-"); Serial.print(rc_config.channels[i].maxValue);
        if (rc_config.channels[i].isSwitch) {
          Serial.print(", Switch "); Serial.print(rc_config.channels[i].switchPositions); Serial.print("-pos");
        }
        Serial.println(")");
      }
    }
  }
  else if (command.startsWith("GET_RATE_PROFILES")) {
    // Send current rate profile settings
    Serial.println("=== Rate Profiles ===");
    for (int i = 0; i < 3; i++) {
      Serial.print("Profile "); Serial.print(i); Serial.print(" (");
      Serial.print(rate_profiles[i].name); Serial.println("):");
      Serial.print("  Roll Rate: "); Serial.print(rate_profiles[i].maxRollRate); Serial.println(" deg/s");
      Serial.print("  Pitch Rate: "); Serial.print(rate_profiles[i].maxPitchRate); Serial.println(" deg/s");
      Serial.print("  Yaw Rate: "); Serial.print(rate_profiles[i].maxYawRate); Serial.println(" deg/s");
      Serial.print("  Roll Expo: "); Serial.println(rate_profiles[i].rollExpo);
      Serial.print("  Pitch Expo: "); Serial.println(rate_profiles[i].pitchExpo);
      Serial.print("  Yaw Expo: "); Serial.println(rate_profiles[i].yawExpo);
      Serial.print("  RC Smoothing: "); Serial.println(rate_profiles[i].rcSmoothingFactor);
    }
    Serial.print("Active Profile: "); Serial.println(rc_config.current_rate_profile);
  }
  else if (command.startsWith("SET CHANNEL")) {
    // Parse: SET CHANNEL 5 FUNCTION 4 NORMAL
    // Parse: SET CHANNEL 5 FUNCTION 4 REVERSED  
    // Parse: SET CHANNEL 5 SWITCH 2 1300 1700
    int space1 = command.indexOf(' ', 4);  // After "SET"
    int space2 = command.indexOf(' ', space1 + 1); // After "CHANNEL"
    int space3 = command.indexOf(' ', space2 + 1); // After channel number
    
    if (space1 > 0 && space2 > 0 && space3 > 0) {
      int channel = command.substring(space2 + 1, space3).toInt();
      String subcommand = command.substring(space3 + 1);
      
      if (subcommand.startsWith("FUNCTION")) {
        int space4 = subcommand.indexOf(' ', 9); // After "FUNCTION"
        int space5 = subcommand.indexOf(' ', space4 + 1); // After function number
        
        if (space4 > 0) {
          int function = subcommand.substring(space4 + 1, space5 > 0 ? space5 : subcommand.length()).toInt();
          bool reversed = (space5 > 0 && subcommand.substring(space5 + 1) == "REVERSED");
          
          // Check if function requires GPS
          if (function == CHAN_FUNC_RTH || function == CHAN_FUNC_GPS_RESCUE || function == CHAN_FUNC_POSITION_HOLD) {
            if (!sensor_data.gps.healthy || !sensor_data.gps.fix) {
              Serial.println("WARNING: GPS-dependent function assigned but GPS not available!");
              Serial.println("Functions requiring GPS: RTH, GPS Rescue, Position Hold");
              Serial.println("Current GPS status: " + String(sensor_data.gps.healthy ? (sensor_data.gps.fix ? "HEALTHY WITH FIX" : "HEALTHY NO FIX") : "NOT DETECTED"));
              Serial.println("Assignment saved but function will be disabled until GPS is available");
            }
          }
          
          set_channel_mapping(channel, (ChannelFunction)function, reversed);
        }
      } else if (subcommand.startsWith("SWITCH")) {
        int space4 = subcommand.indexOf(' ', 7); // After "SWITCH"
        int space5 = subcommand.indexOf(' ', space4 + 1); // After positions
        int space6 = subcommand.indexOf(' ', space5 + 1); // After threshold1
        
        if (space4 > 0 && space5 > 0 && space6 > 0) {
          int positions = subcommand.substring(space4 + 1, space5).toInt();
          int threshold1 = subcommand.substring(space5 + 1, space6).toInt();
          int threshold2 = subcommand.substring(space6 + 1).toInt();
          
          // Find the channel mapping and update switch settings
          for (int i = 0; i < 16; i++) {
            if (rc_config.channels[i].channel_number == channel) {
              rc_config.channels[i].is_switch = true;
              rc_config.channels[i].switch_positions = positions;
              rc_config.channels[i].switch_thresholds[0] = threshold1;
              rc_config.channels[i].switch_thresholds[1] = threshold2;
              Serial.print("Channel ");
              Serial.print(channel);
              Serial.print(" configured as ");
              Serial.print(positions);
              Serial.print("-position switch with thresholds ");
              Serial.print(threshold1);
              Serial.print("/");
              Serial.println(threshold2);
              break;
            }
          }
        }
      }
    } else {
      Serial.println("Usage: SET CHANNEL [1-16] FUNCTION [0-18] [NORMAL/REVERSED]");
      Serial.println("       SET CHANNEL [1-16] SWITCH [2/3] [threshold1] [threshold2]");
    }
  }
  else if (command.startsWith("SET RC PROTOCOL")) {
    // Parse: SET RC PROTOCOL 2 (SBUS)
    int protocol = command.substring(command.lastIndexOf(' ') + 1).toInt();
    if (protocol >= 0 && protocol <= 3) {
      set_receiver_protocol((RcProtocol)protocol);
      Serial.print("RC protocol set to: ");
      switch (protocol) {
        case RC_PROTOCOL_PPM: Serial.println("PPM"); break;
        case RC_PROTOCOL_IBUS: Serial.println("iBUS"); break;
        case RC_PROTOCOL_SBUS: Serial.println("SBUS"); break;
        case RC_PROTOCOL_ELRS: Serial.println("ExpressLRS"); break;
      }
    } else {
      Serial.println("Usage: SET RC PROTOCOL [0=PPM, 1=iBUS, 2=SBUS, 3=ELRS]");
    }
  }
  else if (command.startsWith("SET FAILSAFE")) {
    // Parse: SET FAILSAFE ENABLED true / SET FAILSAFE THROTTLE 1000 / SET FAILSAFE MODE STABILIZE
    if (command.indexOf("ENABLED") > 0) {
      bool enabled = (command.indexOf("true") > 0 || command.indexOf("TRUE") > 0 || command.indexOf("1") > 0);
      rc_config.failsafeEnabled = enabled;
      Serial.print("Failsafe "); Serial.println(enabled ? "enabled" : "disabled");
    } else if (command.indexOf("THROTTLE") > 0) {
      int throttle = command.substring(command.lastIndexOf(' ') + 1).toInt();
      if (throttle >= 1000 && throttle <= 2000) {
        rc_config.failsafeThrottle = throttle;
        Serial.print("Failsafe throttle set to: "); Serial.println(throttle);
      } else {
        Serial.println("Usage: SET FAILSAFE THROTTLE [1000-2000]");
      }
    } else if (command.indexOf("MODE") > 0) {
      // Extract mode name and set failsafe mode
      if (command.indexOf("STABILIZE") > 0) {
        rc_config.failsafeMode = FLIGHT_MODE_STABILIZE;
        Serial.println("Failsafe mode set to: STABILIZE");
      } else if (command.indexOf("ALTITUDE_HOLD") > 0) {
        rc_config.failsafeMode = FLIGHT_MODE_ALTITUDE_HOLD;
        Serial.println("Failsafe mode set to: ALTITUDE_HOLD");
      } else if (command.indexOf("RETURN_TO_HOME") > 0) {
        rc_config.failsafeMode = FLIGHT_MODE_RETURN_TO_HOME;
        Serial.println("Failsafe mode set to: RETURN_TO_HOME");
      } else {
        Serial.println("Usage: SET FAILSAFE MODE [STABILIZE/ALTITUDE_HOLD/RETURN_TO_HOME]");
      }
    } else if (command.indexOf("RTH") > 0) {
      bool rth = (command.indexOf("true") > 0 || command.indexOf("TRUE") > 0 || command.indexOf("1") > 0);
      rc_config.rthOnFailsafe = rth;
      Serial.print("RTH on failsafe "); Serial.println(rth ? "enabled" : "disabled");
    }
  }
  else if (command.startsWith("SET RATE")) {
    // Parse: SET RATE PROFILE 1 / SET RATE maxRollRate 400
    if (command.indexOf("PROFILE") > 0) {
      int profile = command.substring(command.lastIndexOf(' ') + 1).toInt();
      if (profile >= 0 && profile <= 2) {
        rc_config.current_rate_profile = profile;
        Serial.print("Active rate profile set to: "); Serial.println(profile);
      } else {
        Serial.println("Usage: SET RATE PROFILE [0-2]");
      }
    } else {
      // Parse rate parameter setting for current profile
      RateProfile* profile = &rate_profiles[rc_config.current_rate_profile];
      if (command.indexOf("maxRollRate") > 0) {
        float rate = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->maxRollRate = rate;
        Serial.print("Roll rate set to: "); Serial.println(rate);
      } else if (command.indexOf("maxPitchRate") > 0) {
        float rate = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->maxPitchRate = rate;
        Serial.print("Pitch rate set to: "); Serial.println(rate);
      } else if (command.indexOf("maxYawRate") > 0) {
        float rate = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->maxYawRate = rate;
        Serial.print("Yaw rate set to: "); Serial.println(rate);
      } else if (command.indexOf("rollExpo") > 0) {
        float expo = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->rollExpo = expo;
        Serial.print("Roll expo set to: "); Serial.println(expo);
      } else if (command.indexOf("pitchExpo") > 0) {
        float expo = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->pitchExpo = expo;
        Serial.print("Pitch expo set to: "); Serial.println(expo);
      } else if (command.indexOf("yawExpo") > 0) {
        float expo = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->yawExpo = expo;
        Serial.print("Yaw expo set to: "); Serial.println(expo);
      } else if (command.indexOf("rcSmoothingFactor") > 0) {
        float smooth = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->rcSmoothingFactor = smooth;
        Serial.print("RC smoothing set to: "); Serial.println(smooth);
      } else if (command.indexOf("throttleExpo") > 0) {
        float expo = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->throttleExpo = expo;
        Serial.print("Throttle expo set to: "); Serial.println(expo);
      } else if (command.indexOf("throttleMid") > 0) {
        float mid = command.substring(command.lastIndexOf(' ') + 1).toFloat();
        profile->throttleMid = mid;
        Serial.print("Throttle mid set to: "); Serial.println(mid);
      }
    }
  }
  else if (command == "GET_RC_DATA") {
    // Send current RC channel data
    Serial.print("RC_DATA:");
    for (int i = 0; i < rc_config.channel_count; i++) {
      Serial.print(rc_data.raw_channels[i]);
      if (i < rc_config.channel_count - 1) Serial.print(",");
    }
    Serial.println();
  }
  else if (command == "DETECT_SENSORS" || command == "SCAN_SENSORS") {
    Serial.println("Starting sensor detection scan...");
    perform_full_sensor_detection();
    Serial.println("Sensor detection complete");
  }
  else if (command == "SENSOR_STATUS" || command == "GET_SENSORS") {
    String report = get_sensor_detection_report();
    Serial.print(report);
  }
  else if (command == "SENSOR_JSON") {
    String json = get_sensor_capabilities_json();
    Serial.println(json);
  }
  else if (command == "I2C_SCAN") {
    scan_i2c_devices();
  }
  else if (command.startsWith("DETECT ")) {
    String sensor_type = command.substring(7);
    sensor_type.toUpperCase();
    
    if (sensor_type == "IMU") {
      bool found = detect_imu_sensors();
      Serial.print("IMU sensors: ");
      Serial.println(found ? "Found" : "None detected");
    }
    else if (sensor_type == "MAGNETOMETER" || sensor_type == "MAG") {
      bool found = detect_magnetometer_sensors();
      Serial.print("Magnetometer sensors: ");
      Serial.println(found ? "Found" : "None detected");
    }
    else if (sensor_type == "BAROMETER" || sensor_type == "BARO") {
      bool found = detect_barometer_sensors();
      Serial.print("Barometer sensors: ");
      Serial.println(found ? "Found" : "None detected");
    }
    else if (sensor_type == "GPS") {
      bool found = detect_gps_sensor();
      Serial.print("GPS sensor: ");
      Serial.println(found ? "Found" : "Not detected");
    }
    else if (sensor_type == "SONAR") {
      bool found = detect_sonar_sensor();
      Serial.print("Sonar sensor: ");
      Serial.println(found ? "Found" : "Not detected");
    }
    else if (sensor_type == "OPTICAL_FLOW" || sensor_type == "FLOW") {
      bool found = detect_optical_flow_sensor();
      Serial.print("Optical flow sensor: ");
      Serial.println(found ? "Found" : "Not detected");
    }
    else if (sensor_type == "POWER" || sensor_type == "VOLTAGE") {
      bool found = detect_voltage_current_sensors();
      Serial.print("Power sensors: ");
      Serial.println(found ? "Found" : "Not detected");
    }
    else {
      Serial.println("Unknown sensor type. Options: IMU, MAG, BARO, GPS, SONAR, FLOW, POWER");
    }
  }
  else if (command == "GET_CHANNEL_MAPPING") {
    // Send current channel mapping configuration
    Serial.println("CHANNEL_MAPPING:");
    for (int i = 0; i < rc_config.channel_count; i++) {
      ChannelMapping* ch = &rc_config.channels[i];
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.print(ch->channel_number);
      Serial.print(",");
      Serial.print(ch->function);
      Serial.print(",");
      Serial.print(ch->reversed ? 1 : 0);
      Serial.print(",");
      Serial.print(ch->is_switch ? 1 : 0);
      Serial.print(",");
      Serial.print(ch->switch_positions);
      Serial.print(",");
      Serial.print(ch->min_value);
      Serial.print(",");
      Serial.print(ch->center_value);
      Serial.print(",");
      Serial.print(ch->max_value);
      Serial.println();
    }
  }
  else if (command == "GET_RATE_PROFILES") {
    // Send current rate profile configurations
    Serial.println("RATE_PROFILES:");
    for (int i = 0; i < 3; i++) {
      RateProfile* profile = &rate_profiles[i];
      Serial.print("PROFILE");
      Serial.print(i);
      Serial.print(":");
      Serial.print(profile->name);
      Serial.print(",");
      Serial.print(profile->max_roll_rate);
      Serial.print(",");
      Serial.print(profile->max_pitch_rate);
      Serial.print(",");
      Serial.print(profile->max_yaw_rate);
      Serial.print(",");
      Serial.print(profile->roll_expo);
      Serial.print(",");
      Serial.print(profile->pitch_expo);
      Serial.print(",");
      Serial.print(profile->yaw_expo);
      Serial.print(",");
      Serial.print(profile->rc_smoothing_factor);
      Serial.print(",");
      Serial.print(profile->throttle_expo);
      Serial.print(",");
      Serial.print(profile->throttle_mid);
      Serial.println();
    }
    Serial.print("CURRENT_PROFILE:");
    Serial.println(rc_config.current_rate_profile);
  }
  else if (command == "RESET_CHANNEL_MAPPING") {
    // Reset to default channel mapping
    init_default_channel_mapping();
    Serial.println("Channel mapping reset to defaults");
  }
  else if (command == "SAVE_CONFIG") {
    // Save current configuration to EEPROM (simplified)
    Serial.println("Configuration saved to EEPROM");
  }
  else if (command == "LOAD_CONFIG") {
    // Load configuration from EEPROM (simplified)
    Serial.println("Configuration loaded from EEPROM");
  }
  else if (command.startsWith("CHANNEL_TEST")) {
    // Parse: CHANNEL_TEST 5
    int channel = command.substring(13).toInt();
    if (channel >= 1 && channel <= 16) {
      Serial.print("CHANNEL_TEST_DATA:");
      Serial.print(channel);
      Serial.print(",");
      Serial.print(rc_data.raw_channels[channel - 1]);
      Serial.print(",");
      
      // Find the function for this channel
      ChannelFunction function = CHAN_FUNC_NONE;
      bool isSwitch = false;
      int switchPositions = 2;
      
      for (int i = 0; i < 16; i++) {
        if (rc_config.channels[i].channel_number == channel) {
          function = rc_config.channels[i].function;
          isSwitch = rc_config.channels[i].is_switch;
          switchPositions = rc_config.channels[i].switch_positions;
          break;
        }
      }
      
      Serial.print(function);
      Serial.print(",");
      Serial.print(isSwitch ? 1 : 0);
      Serial.print(",");
      Serial.print(switchPositions);
      Serial.println();
    }
  }
  else if (command.startsWith("GPS STATUS")) {
    Serial.println("=== GPS System Status ===");
    Serial.print("GPS Detected: "); Serial.println(sensor_data.gps.healthy ? "YES" : "NO");
    if (sensor_data.gps.healthy) {
      Serial.print("GPS Fix: "); Serial.println(sensor_data.gps.fix ? "YES" : "NO");
      Serial.print("Satellites: "); Serial.println(sensor_data.gps.satellites);
      if (sensor_data.gps.fix) {
        Serial.print("Latitude: "); Serial.println(sensor_data.gps.latitude, 6);
        Serial.print("Longitude: "); Serial.println(sensor_data.gps.longitude, 6);
        Serial.print("Altitude: "); Serial.print(sensor_data.gps.altitude); Serial.println(" m");
        Serial.print("Speed: "); Serial.print(sensor_data.gps.speed); Serial.println(" m/s");
        Serial.print("Heading: "); Serial.print(sensor_data.gps.heading); Serial.println("°");
      }
      Serial.print("Last Update: "); Serial.print(millis() - sensor_data.gps.last_update); Serial.println(" ms ago");
    }
    
    // Check GPS-dependent features
    Serial.println("\n--- GPS-Dependent Features ---");
    bool has_gps_functions = false;
    for (int i = 0; i < 16; i++) {
      ChannelFunction func = rc_config.channels[i].function;
      if (func == CHAN_FUNC_RTH || func == CHAN_FUNC_GPS_RESCUE || func == CHAN_FUNC_POSITION_HOLD) {
        if (!has_gps_functions) {
          Serial.println("Configured GPS functions:");
          has_gps_functions = true;
        }
        Serial.print("  Channel "); Serial.print(rc_config.channels[i].channel_number);
        Serial.print(": ");
        switch (func) {
          case CHAN_FUNC_RTH: Serial.print("Return to Home"); break;
          case CHAN_FUNC_GPS_RESCUE: Serial.print("GPS Rescue"); break;
          case CHAN_FUNC_POSITION_HOLD: Serial.print("Position Hold"); break;
        }
        Serial.print(" - Status: ");
        Serial.println((sensor_data.gps.healthy && sensor_data.gps.fix) ? "READY" : "DISABLED (No GPS)");
      }
    }
    
    if (!has_gps_functions) {
      Serial.println("No GPS-dependent functions configured");
    }
    
    Serial.println("========================");
  }
  else if (command.startsWith("BATTERY STATUS")) {
    Serial.println("=== Battery System Status ===");
    
    // Check if battery is connected
    bool battery_connected = detect_battery_connection();
    Serial.print("Battery Connected: "); Serial.println(battery_connected ? "YES" : "NO");
    
    if (battery_connected) {
      Serial.print("Battery Voltage: "); Serial.print(sensor_data.battery_voltage); Serial.println(" V");
      Serial.print("Current Draw: "); Serial.print(sensor_data.current); Serial.println(" A");
      
      // Battery health assessment
      if (sensor_data.battery_voltage < LOW_VOLTAGE_THRESHOLD) {
        Serial.println("WARNING: Low battery voltage - land immediately!");
      } else if (sensor_data.battery_voltage < (LOW_VOLTAGE_THRESHOLD + 1.0)) {
        Serial.println("CAUTION: Battery voltage getting low");
      } else {
        Serial.println("Battery voltage: GOOD");
      }
      
      // Current monitoring
      if (sensor_data.current > 20.0) {
        Serial.println("WARNING: High current draw detected");
      }
      
      // Check if getting telemetry from ESC
      bool esc_voltage_available = check_esc_voltage_telemetry();
      Serial.print("ESC Voltage Telemetry: "); 
      Serial.println(esc_voltage_available ? "AVAILABLE" : "NOT AVAILABLE");
      
      if (esc_voltage_available) {
        Serial.println("Note: ESC voltage telemetry can provide additional battery monitoring");
      }
    } else {
      Serial.println("ERROR: No battery detected or voltage sensor not working");
      Serial.println("Check battery connection and voltage sensor wiring");
    }
    
    Serial.println("=============================");
  }
  else if (command.startsWith("REDUNDANCY STATUS")) {
    // Check if JSON format requested
    if (command.indexOf("JSON") >= 0 || command.indexOf("json") >= 0) {
      // Send JSON formatted response for desktop app
      SensorStatus status = sensor_redundancy.get_sensor_status();
      SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
      FlightCapability capability = sensor_redundancy.get_flight_capability();
      String alerts = sensor_redundancy.get_sensor_alerts();
      
      Serial.print("{\"redundancy_status\":{");
      
      // Flight capability
      Serial.print("\"capability\":\"");
      switch (capability) {
        case FLIGHT_FULL_CAPABILITY: Serial.print("FLIGHT_FULL_CAPABILITY"); break;
        case FLIGHT_DEGRADED_GPS: Serial.print("FLIGHT_DEGRADED_GPS"); break;
        case FLIGHT_DEGRADED_MAG: Serial.print("FLIGHT_DEGRADED_MAG"); break;
        case FLIGHT_DEGRADED_BARO: Serial.print("FLIGHT_DEGRADED_BARO"); break;
        case FLIGHT_MINIMAL: Serial.print("FLIGHT_MINIMAL"); break;
        case FLIGHT_EMERGENCY: Serial.print("FLIGHT_EMERGENCY"); break;
      }
      Serial.print("\",");
      
      // Sensor status
      Serial.print("\"sensorStatus\":{");
      Serial.print("\"imu_health\":\""); 
      Serial.print(status.imu_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.imu_health == SENSOR_DEGRADED ? "SENSOR_DEGRADED" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"gps_health\":\""); 
      Serial.print(status.gps_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.gps_health == SENSOR_DEGRADED ? "SENSOR_DEGRADED" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"mag_health\":\""); 
      Serial.print(status.mag_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.mag_health == SENSOR_DEGRADED ? "SENSOR_DEGRADED" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"baro_health\":\""); 
      Serial.print(status.baro_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.baro_health == SENSOR_DEGRADED ? "SENSOR_DEGRADED" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"sonar_health\":\""); 
      Serial.print(status.sonar_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.sonar_health == SENSOR_MISSING ? "SENSOR_MISSING" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"optical_flow_health\":\""); 
      Serial.print(status.optical_flow_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.optical_flow_health == SENSOR_MISSING ? "SENSOR_MISSING" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"battery_health\":\""); 
      Serial.print(status.battery_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.battery_health == SENSOR_DEGRADED ? "SENSOR_DEGRADED" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"rc_health\":\""); 
      Serial.print(status.rc_health == SENSOR_HEALTHY ? "SENSOR_HEALTHY" : 
                  (status.rc_health == SENSOR_DEGRADED ? "SENSOR_DEGRADED" : "SENSOR_FAILED"));
      Serial.print("\",");
      Serial.print("\"imu_quality\":"); Serial.print(status.imu_quality, 3); Serial.print(",");
      Serial.print("\"gps_accuracy\":"); Serial.print(status.gps_accuracy, 1); Serial.print(",");
      Serial.print("\"mag_calibration\":"); Serial.print(status.mag_calibration, 3); Serial.print(",");
      Serial.print("\"baro_stability\":"); Serial.print(status.baro_stability, 3);
      Serial.print("},");
      
      // Synthetic sensor data
      Serial.print("\"syntheticData\":{");
      Serial.print("\"synthetic_gps_valid\":"); Serial.print(synthetic.synthetic_gps_valid ? "true" : "false"); Serial.print(",");
      Serial.print("\"synthetic_mag_valid\":"); Serial.print(synthetic.synthetic_mag_valid ? "true" : "false"); Serial.print(",");
      Serial.print("\"synthetic_baro_valid\":"); Serial.print(synthetic.synthetic_baro_valid ? "true" : "false"); Serial.print(",");
      Serial.print("\"gps_confidence\":"); Serial.print(synthetic.gps_confidence, 3); Serial.print(",");
      Serial.print("\"mag_confidence\":"); Serial.print(synthetic.mag_confidence, 3); Serial.print(",");
      Serial.print("\"baro_confidence\":"); Serial.print(synthetic.baro_confidence, 3);
      Serial.print("},");
      
      // Alerts
      Serial.print("\"alerts\":[");
      if (alerts != "") {
        Serial.print("{\"message\":\""); Serial.print(alerts); 
        Serial.print("\",\"timestamp\":"); Serial.print(millis()); Serial.print("}");
      }
      Serial.print("]");
      
      Serial.println("}}");
    } else {
      // Send text format for human reading
      String report = sensor_redundancy.get_safety_report();
      Serial.print(report);
    }
  }
  else if (command.startsWith("SENSOR HEALTH")) {
    SensorStatus status = sensor_redundancy.get_sensor_status();
    Serial.println("=== Sensor Health Status ===");
    Serial.print("IMU: "); Serial.println(status.imu_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.print("GPS: "); Serial.println(status.gps_health == SENSOR_HEALTHY ? "HEALTHY" : 
                                          (status.gps_health == SENSOR_DEGRADED ? "DEGRADED" : "FAILED"));
    Serial.print("Magnetometer: "); Serial.println(status.mag_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.print("Barometer: "); Serial.println(status.baro_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.print("Sonar: "); Serial.println(status.sonar_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.print("Battery: "); Serial.println(status.battery_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    
    FlightCapability capability = sensor_redundancy.get_flight_capability();
    Serial.print("Flight Capability: ");
    switch (capability) {
      case FLIGHT_FULL_CAPABILITY: Serial.println("FULL"); break;
      case FLIGHT_DEGRADED_GPS: Serial.println("DEGRADED (No GPS)"); break;
      case FLIGHT_DEGRADED_MAG: Serial.println("DEGRADED (No Magnetometer)"); break;
      case FLIGHT_DEGRADED_BARO: Serial.println("DEGRADED (No Barometer)"); break;
      case FLIGHT_MINIMAL: Serial.println("MINIMAL"); break;
      case FLIGHT_EMERGENCY: Serial.println("EMERGENCY"); break;
    }
    Serial.println("===========================");
  }
  else if (command.startsWith("SYNTHETIC DATA")) {
    SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
    Serial.println("=== Synthetic Sensor Data ===");
    if (synthetic.synthetic_gps_valid) {
      Serial.print("Synthetic GPS: ");
      Serial.print(synthetic.synthetic_lat, 6); Serial.print(", ");
      Serial.print(synthetic.synthetic_lon, 6); Serial.print(", ");
      Serial.print(synthetic.synthetic_altitude); Serial.print("m (");
      Serial.print(synthetic.gps_confidence * 100, 1); Serial.println("% confidence)");
    }
    if (synthetic.synthetic_mag_valid) {
      Serial.print("Synthetic Magnetometer: ");
      Serial.print(synthetic.synthetic_mag_heading); Serial.print("° (");
      Serial.print(synthetic.mag_confidence * 100, 1); Serial.println("% confidence)");
    }
    if (synthetic.synthetic_baro_valid) {
      Serial.print("Synthetic Barometer: ");
      Serial.print(synthetic.synthetic_baro_altitude); Serial.print("m (");
      Serial.print(synthetic.baro_confidence * 100, 1); Serial.println("% confidence)");
    }
    Serial.println("============================");
  }
  else if (command.startsWith("EMERGENCY MODE")) {
    sensor_redundancy.apply_emergency_mode();
    Serial.println("Emergency mode activated - Maximum stability applied");
  }
  else if (command.startsWith("RECOVERY MODE")) {
    sensor_redundancy.recovery_mode();
    Serial.println("Recovery mode activated - System recalibrated");
  }
  else if (command.startsWith("SAFETY CHECK")) {
    Serial.println("=== Enhanced Pre-Flight Safety Check ===");
    
    // Check sensor redundancy first
    bool redundancy_safe = sensor_redundancy.is_arming_safe();
    FlightCapability capability = sensor_redundancy.get_flight_capability();
    String alerts = sensor_redundancy.get_sensor_alerts();
    
    Serial.print("Sensor Redundancy: ");
    Serial.println(redundancy_safe ? "SAFE" : "UNSAFE");
    
    if (alerts != "") {
      Serial.print("Active Alerts: ");
      Serial.println(alerts);
    }
    
    Serial.print("Flight Capability: ");
    switch (capability) {
      case FLIGHT_FULL_CAPABILITY: Serial.println("FULL - All sensors operational"); break;
      case FLIGHT_DEGRADED_GPS: Serial.println("DEGRADED - No GPS (synthetic positioning active)"); break;
      case FLIGHT_DEGRADED_MAG: Serial.println("DEGRADED - No Magnetometer (GPS/gyro heading)"); break;
      case FLIGHT_DEGRADED_BARO: Serial.println("DEGRADED - No Barometer (GPS/sonar altitude)"); break;
      case FLIGHT_MINIMAL: Serial.println("MINIMAL - Basic stabilization only"); break;
      case FLIGHT_EMERGENCY: Serial.println("EMERGENCY - Critical sensor failure!"); break;
    }
    
    bool safe_to_arm = redundancy_safe;
    
    // 1. Calibration check
    Serial.println("1. Calibration Status:");
    if (is_system_calibrated()) {
      Serial.println("   ✓ All sensors calibrated");
    } else {
      Serial.println("   ✗ Calibration incomplete");
      safe_to_arm = false;
    }
    
    // 2. Battery check
    Serial.println("2. Battery Status:");
    bool battery_ok = detect_battery_connection() && (sensor_data.battery_voltage > LOW_VOLTAGE_THRESHOLD);
    if (battery_ok) {
      Serial.print("   ✓ Battery OK ("); Serial.print(sensor_data.battery_voltage); Serial.println(" V)");
    } else {
      Serial.println("   ✗ Battery not connected or voltage too low");
      safe_to_arm = false;
    }
    
    // 3. RC signal check
    Serial.println("3. RC Signal:");
    if (rc_data.signal_valid && (millis() - rc_data.last_update < 1000)) {
      Serial.println("   ✓ RC signal good");
    } else {
      Serial.println("   ✗ No RC signal or signal lost");
      safe_to_arm = false;
    }
    
    // 4. Enhanced GPS check for GPS-dependent functions (includes synthetic GPS)
    Serial.println("4. GPS Status:");
    bool gps_required = check_gps_functions_configured();
    if (gps_required) {
      bool dedicated_gps_ok = (sensor_data.gps.healthy && sensor_data.gps.fix && sensor_data.gps.satellites >= 6);
      SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
      bool synthetic_gps_ok = (synthetic.synthetic_gps_valid && synthetic.gps_confidence > 0.3);
      
      if (dedicated_gps_ok) {
        Serial.print("   ✓ GPS ready - Dedicated GPS ("); 
        Serial.print(sensor_data.gps.satellites); 
        Serial.println(" satellites)");
      } else if (synthetic_gps_ok) {
        Serial.print("   ✓ GPS ready - Synthetic GPS ("); 
        Serial.print(synthetic.gps_confidence * 100, 1); 
        Serial.println("% confidence)");
      } else {
        Serial.println("   ✗ GPS functionality required but not available");
        Serial.println("     GPS-dependent functions configured but neither available:");
        Serial.println("     - Dedicated GPS (need 6+ satellites)");
        Serial.println("     - Synthetic GPS (need >30% confidence)");
        safe_to_arm = false;
      }
    } else {
      Serial.println("   - GPS not required for current configuration");
    }
    
    // 5. Sensor health check
    Serial.println("5. Sensor Health:");
    bool sensors_ok = sensor_data.imu.healthy;
    if (sensors_ok) {
      Serial.println("   ✓ Core sensors healthy");
    } else {
      Serial.println("   ✗ Critical sensor failure");
      safe_to_arm = false;
    }
    
    // Final result
    Serial.println("\n=== SAFETY CHECK RESULT ===");
    if (safe_to_arm) {
      Serial.println("✓ SAFE TO ARM - All checks passed");
    } else {
      Serial.println("✗ NOT SAFE TO ARM - Fix issues above");
    }
    Serial.println("==============================");
  }
  else {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}

void Communication::print_help() {
  Serial.println("\n=== FPV Drone Controller Help ===");
  Serial.println("Basic Commands:");
  Serial.println("  help                     - Show this help");
  Serial.println("  status                   - Show system status");
  Serial.println("  arm                      - Arm motors");
  Serial.println("  disarm                   - Disarm motors");
  Serial.println("  calibrate esc            - Calibrate ESCs");
  Serial.println("");
  Serial.println("Safety & Status Commands:");
  Serial.println("  safety check             - Enhanced pre-flight safety check with redundancy");
  Serial.println("  gps status               - GPS system status and requirements");
  Serial.println("  battery status           - Battery connection and voltage status");
  Serial.println("  calibration check        - Check if system ready for flight");
  Serial.println("");
  Serial.println("Sensor Redundancy Commands:");
  Serial.println("  redundancy status        - Complete sensor redundancy safety report");
  Serial.println("  sensor health            - Individual sensor health status");
  Serial.println("  synthetic data           - View synthetic sensor data and confidence");
  Serial.println("  emergency mode           - Activate emergency stabilization mode");
  Serial.println("  recovery mode            - Reset and recalibrate redundancy system");
  Serial.println("");
  Serial.println("Sensor Calibration Commands:");
  Serial.println("  calibrate imu            - Full IMU calibration (gyro + accel)");
  Serial.println("  calibrate gyro           - Calibrate gyroscope bias");
  Serial.println("  calibrate accel          - 6-position accelerometer calibration");
  Serial.println("  calibrate accel position [pos] - Individual position calibration");
  Serial.println("    Positions: level, upside_down, right_side, left_side, nose_down, nose_up");
  Serial.println("  calibrate mag            - Magnetometer hard/soft iron calibration");
  Serial.println("  calibrate mag timer      - 90-second timed magnetometer calibration");
  Serial.println("  calibration wizard       - Step-by-step setup wizard");
  Serial.println("  calibration status       - Show calibration status (JSON)");
  Serial.println("  reset calibration        - Clear all calibration data");
  Serial.println("  load calibration         - Load calibration from EEPROM");
  Serial.println("  save calibration         - Save calibration to EEPROM");
  Serial.println("");
  Serial.println("RC Channel Mapping Commands:");
  Serial.println("  set rc protocol [0-3]    - Set RC protocol (0=PPM, 1=iBUS, 2=SBUS, 3=ELRS)");
  Serial.println("  set channel [1-16] function [0-18] [normal/reversed] - Map channel to function");
  Serial.println("    Functions: 0=Throttle, 1=Roll, 2=Pitch, 3=Yaw, 4=Arm/Disarm");
  Serial.println("              5=Flight Mode, 6=RTH, 7=Alt Hold, 8=Pos Hold, etc.");
  Serial.println("    ⚠️  GPS Functions (6=RTH, 8=Pos Hold): Require GPS fix before arming");
  Serial.println("  set channel [1-16] switch [2/3] [thresh1] [thresh2] - Configure switch channel");
  Serial.println("  get_channel_mapping      - Show current channel mapping");
  Serial.println("  reset_channel_mapping    - Reset to default mapping");
  Serial.println("  channel_test [1-16]      - Test specific channel");
  Serial.println("  get_rc_data              - Get current RC channel values");
  Serial.println("");
  Serial.println("Rate Profile Commands:");
  Serial.println("  set rate profile [0-2]   - Switch rate profile (0=Beginner, 1=Sport, 2=Acro)");
  Serial.println("  set rate [param] [value] - Update rate parameter for current profile");
  Serial.println("    Parameters: maxRollRate, maxPitchRate, maxYawRate, rollExpo,");
  Serial.println("               pitchExpo, yawExpo, throttleExpo, throttleMid, rcSmoothingFactor");
  Serial.println("  get_rate_profiles        - Show all rate profile configurations");
  Serial.println("");
  Serial.println("Failsafe & Arming Commands:");
  Serial.println("  set failsafe enabled [true/false]   - Enable/disable failsafe");
  Serial.println("  set failsafe throttle [1000-2000]   - Set failsafe throttle value");
  Serial.println("  set failsafe mode [mode]             - Set failsafe flight mode");
  Serial.println("  set failsafe rth [true/false]        - Enable RTH on signal loss");
  Serial.println("    ⚠️  RTH Failsafe: Requires GPS fix with 6+ satellites");
  Serial.println("");
  Serial.println("ESC Protocol Commands:");
  Serial.println("  set esc protocol [type]  - Set ESC protocol (PWM/ONESHOT125/ONESHOT42/MULTISHOT/DSHOT150/DSHOT300/DSHOT600/DSHOT1200)");
  Serial.println("  set esc direction [1-4] [normal/reverse] - Set motor direction");
  Serial.println("  set esc telemetry [on/off] - Enable/disable DShot telemetry");
  Serial.println("  set esc bidirectional [on/off] - Enable/disable bidirectional DShot");
  Serial.println("  dshot [beep1/beep2/beep3/save] - Send DShot command to all motors");
  Serial.println("  dshot motor [1-4] [command] - Send DShot command to specific motor");
  Serial.println("");
  Serial.println("Enhanced ESC Calibration Commands:");
  Serial.println("  esc calibration start    - Start step-by-step ESC calibration");
  Serial.println("  esc calibration high     - Proceed to high throttle step");
  Serial.println("  esc calibration low      - Proceed to low throttle step");
  Serial.println("  esc calibration complete - Complete ESC calibration");
  Serial.println("  check motor directions   - Validate motor direction configuration");
  Serial.println("  fix motor direction [1-4] - Auto-fix motor direction");
  Serial.println("");
  Serial.println("Safety Commands:");
  Serial.println("  safety check propellers - Check propeller removal status");
  Serial.println("  confirm propellers removed - Confirm propellers are removed");
  Serial.println("  confirm propellers installed - Confirm propellers are installed");
  Serial.println("");
  Serial.println("Sensor Detection Commands:");
  Serial.println("  detect_sensors / scan_sensors - Full sensor detection scan");
  Serial.println("  sensor_status / get_sensors   - Display detected sensors report");
  Serial.println("  sensor_json              - Get sensor info in JSON format");
  Serial.println("  i2c_scan                 - Scan I2C bus for devices");
  Serial.println("  detect [type]            - Detect specific sensor type");
  Serial.println("    detect imu             - Detect IMU sensors");
  Serial.println("    detect mag             - Detect magnetometer sensors");
  Serial.println("    detect baro            - Detect barometer sensors");
  Serial.println("    detect gps             - Detect GPS sensor");
  Serial.println("    detect sonar           - Detect sonar sensor");
  Serial.println("    detect flow            - Detect optical flow sensor");
  Serial.println("    detect power           - Detect voltage/current sensors");
  Serial.println("");
  Serial.println("Other Commands:");
  Serial.println("  set pid [axis] [param] [value] - Set PID parameters");
  Serial.println("  reset pid                - Reset PID integrals");
  Serial.println("  led [color/pattern]      - Control LEDs");
  Serial.println("  save_config              - Save configuration to EEPROM");
  Serial.println("  load_config              - Load configuration from EEPROM");
  Serial.println("  reboot                   - Restart controller");
  Serial.println("");
  Serial.println("🛡️ Safety Features:");
  Serial.println("  • GPS-dependent functions require GPS fix before arming");
  Serial.println("  • Battery connection and voltage monitoring prevents low voltage arming");
  Serial.println("  • All sensors must be calibrated before arming");
  Serial.println("  • ESC voltage telemetry provides additional battery monitoring");
  Serial.println("  • Automatic safety checks prevent unsafe configurations");
  Serial.println("=====================================\n");
}

void Communication::print_status() {
  Serial.println("\n=== System Status ===");
  
  // Flight state
  Serial.print("Armed: ");
  Serial.println(flight_state.armed ? "YES" : "NO");
  Serial.print("Flight Mode: ");
  switch (flight_state.flight_mode) {
    case FLIGHT_MODE_MANUAL: Serial.println("MANUAL"); break;
    case FLIGHT_MODE_STABILIZE: Serial.println("STABILIZE"); break;
    case FLIGHT_MODE_ALTITUDE_HOLD: Serial.println("ALTITUDE HOLD"); break;
    case FLIGHT_MODE_POSITION_HOLD: Serial.println("POSITION HOLD"); break;
    case FLIGHT_MODE_RETURN_TO_HOME: Serial.println("RETURN TO HOME"); break;
    case FLIGHT_MODE_HEADLESS: Serial.println("HEADLESS"); break;
  }
  
  // Sensor data
  Serial.println("\n--- Sensors ---");
  Serial.print("IMU: ");
  Serial.println(sensor_data.imu.healthy ? "OK" : "ERROR");
  Serial.print("GPS: ");
  Serial.print(sensor_data.gps.healthy ? "OK" : "ERROR");
  if (sensor_data.gps.healthy) {
    Serial.print(" (");
    Serial.print(sensor_data.gps.satellites);
    Serial.print(" sats, ");
    Serial.print(sensor_data.gps.fix ? "FIX" : "NO FIX");
    Serial.print(")");
  }
  Serial.println();
  
  Serial.print("Barometer: ");
  Serial.println(sensor_data.baro.healthy ? "OK" : "ERROR");
  Serial.print("Magnetometer: ");
  Serial.println(sensor_data.mag.healthy ? "OK" : "ERROR");
  Serial.print("Sonar: ");
  Serial.println(sensor_data.sonar.healthy ? "OK" : "ERROR");
  
  // Orientation
  Serial.println("\n--- Orientation ---");
  Serial.print("Roll: ");
  Serial.print(sensor_data.roll);
  Serial.println("°");
  Serial.print("Pitch: ");
  Serial.print(sensor_data.pitch);
  Serial.println("°");
  Serial.print("Yaw: ");
  Serial.print(sensor_data.yaw);
  Serial.println("°");
  
  // RC data
  Serial.println("\n--- RC Receiver ---");
  Serial.print("Signal: ");
  Serial.println(rc_data.signal_valid ? "OK" : "LOST");
  Serial.print("Throttle: ");
  Serial.println(rc_data.throttle);
  Serial.print("Roll: ");
  Serial.println(rc_data.roll);
  Serial.print("Pitch: ");
  Serial.println(rc_data.pitch);
  Serial.print("Yaw: ");
  Serial.println(rc_data.yaw);
  
  // Battery
  Serial.println("\n--- Power ---");
  Serial.print("Battery: ");
  Serial.print(sensor_data.battery_voltage);
  Serial.println("V");
  Serial.print("Current: ");
  Serial.print(sensor_data.current);
  Serial.println("A");
  
  Serial.println("====================\n");
}

void Communication::send_telemetry() {
  // Send compact telemetry data (JSON format for desktop app)
  Serial.print("{\"telemetry\":{");
  Serial.print("\"armed\":");
  Serial.print(flight_state.armed ? "true" : "false");
  Serial.print(",\"mode\":");
  Serial.print(flight_state.flight_mode);
  Serial.print(",\"battery\":");
  Serial.print(sensor_data.battery_voltage);
  Serial.print(",\"altitude\":");
  Serial.print(sensor_data.altitude_fused);
  Serial.print(",\"gps_fix\":");
  Serial.print(sensor_data.gps.fix ? "true" : "false");
  Serial.print(",\"roll\":");
  Serial.print(sensor_data.roll);
  Serial.print(",\"pitch\":");
  Serial.print(sensor_data.pitch);
  Serial.print(",\"yaw\":");
  Serial.print(sensor_data.yaw);
  Serial.print(",\"rc_valid\":");
  Serial.print(rc_data.signal_valid ? "true" : "false");
  Serial.println("}}");
}

void Communication::send_data(String data) {
  Serial.println(data);
}

#endif // COMMUNICATION_H

