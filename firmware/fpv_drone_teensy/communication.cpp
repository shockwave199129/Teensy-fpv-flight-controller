#include "communication.h"
#include "sensors.h"
#include "receivers.h"
#include "motor_control.h"
#include "flight_modes.h"
#include "led_control.h"
#include "safety_functions.h"
#include "sensor_redundancy.h"
#include "dual_imu_manager.h"
#include "dynamic_filtering.h"
#include "optical_flow.h"
#include "advanced_flight_modes.h"
#include "pid_controller.h"
#include <math.h>

// Global PID controller instance comes from main sketch
extern PIDController pid_controller;

enum WizardStep {WIZ_NONE, WIZ_START, WIZ_GYRO, WIZ_ACCEL, WIZ_MAG, WIZ_ESC, WIZ_MOTOR_DIR, WIZ_DONE};
static WizardStep wizard_step = WIZ_NONE;

// Forward declarations of wizard helpers
void start_calibration_wizard();
void wizard_next();
bool perform_motor_direction_test();

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
  else if (command == "SAFETY CHECK") {
    if (enhanced_safety_check_for_arming_with_feedback()) {
      Serial.println("✓ SAFE TO ARM - All enhanced safety checks passed");
    } else {
      Serial.println("✗ NOT SAFE TO ARM - Fix issues reported above");
    }
  }
  else if (command == "CALIBRATION CHECK") {
    if (is_system_calibrated()) {
      Serial.println("CALIBRATION_OK: All sensors calibrated - ready for flight");
    } else {
      Serial.println("CALIBRATION_REQUIRED: System not fully calibrated");
      Serial.println("Use 'calibration wizard' for step-by-step setup");
    }
  }
  else if (command == "ARM") {
    if (!armed) {
      if (enhanced_safety_check_for_arming_with_feedback()) {
        armed = true;
        motor_control.arm();
        led_control.set_status(LED_ARMED);
        Serial.println("Motors ARMED via CLI");
      } else {
        Serial.println("ARMING FAILED: Safety check did not pass");
      }
    } else {
      Serial.println("Already armed");
    }
  }
  else if (command == "DISARM") {
    if (armed) {
      armed = false;
      motor_control.disarm();
      led_control.set_status(LED_DISARMED);
      Serial.println("Motors disarmed via CLI");
    } else {
      Serial.println("Already disarmed");
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
  else if (command == "CALIBRATE MAG" || command == "CALIBRATE MAGNETOMETER") {
    if (!armed) {
      calibrate_magnetometer();
    } else {
      Serial.println("Cannot calibrate magnetometer while armed");
    }
  }
  else if (command == "REBOOT") {
    Serial.println("Rebooting...");
    delay(1000);
    // Software reset for Teensy
    SCB_AIRCR = 0x05FA0004;
  }
  else if (command == "GPS STATUS") {
    Serial.println("=== GPS System Status ===");
    Serial.print("GPS Detected: "); Serial.println(sensor_data.gps.healthy ? "YES" : "NO");
    if (sensor_data.gps.healthy) {
      Serial.print("GPS Fix: "); Serial.println(sensor_data.gps.fix ? "YES" : "NO");
      Serial.print("Satellites: "); Serial.println(sensor_data.gps.satellites);
      if (sensor_data.gps.fix) {
        Serial.print("Latitude: "); Serial.println(sensor_data.gps.latitude, 6);
        Serial.print("Longitude: "); Serial.println(sensor_data.gps.longitude, 6);
        Serial.print("Altitude: "); Serial.print(sensor_data.gps.altitude); Serial.println(" m");
      }
    }
    Serial.println("========================");
  }
  else if (command == "BATTERY STATUS") {
    Serial.println("=== Battery System Status ===");
    bool battery_connected = detect_battery_connection();
    Serial.print("Battery Connected: "); Serial.println(battery_connected ? "YES" : "NO");
    if (battery_connected) {
      Serial.print("Battery Voltage: "); Serial.print(sensor_data.battery_voltage); Serial.println(" V");
      Serial.print("Current Draw: "); Serial.print(sensor_data.current); Serial.println(" A");
    }
    Serial.println("=============================");
  }
  else if (command == "REDUNDANCY STATUS") {
    extern SensorRedundancySystem sensor_redundancy;
    String report = sensor_redundancy.get_safety_report();
    Serial.print(report);
  }
  else if (command == "SENSOR HEALTH") {
    extern SensorRedundancySystem sensor_redundancy;
    SensorStatus status = sensor_redundancy.get_sensor_status();
    Serial.println("=== Sensor Health Status ===");
    Serial.print("IMU: "); Serial.println(status.imu_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.print("GPS: "); Serial.println(status.gps_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.print("Magnetometer: "); Serial.println(status.mag_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.print("Barometer: "); Serial.println(status.baro_health == SENSOR_HEALTHY ? "HEALTHY" : "FAILED");
    Serial.println("===========================");
  }
  else if (command == "DETECT SENSORS") {
    Serial.println("Starting sensor detection scan...");
    perform_full_sensor_detection();
    Serial.println("Sensor detection complete");
  }
  else if (command == "SENSOR STATUS") {
    String report = get_sensor_detection_report();
    Serial.print(report);
  }
  else if (command == "I2C SCAN") {
    scan_i2c_devices();
  }
  else if (command.startsWith("DUAL IMU")) {
    if (command.indexOf("STATUS") > 0) {
      Serial.println("=== Dual IMU Status ===");
      Serial.print("System Healthy: "); Serial.println(dual_imu_manager.is_healthy() ? "YES" : "NO");
      Serial.print("Using Primary: "); Serial.println(dual_imu_manager.using_primary() ? "YES" : "NO");
      Serial.println("=======================");
    }
  }
  else if (command.startsWith("FILTERING")) {
    if (command.indexOf("STATUS") > 0) {
      Serial.println("=== Dynamic Filtering Status ===");
      Serial.print("Filtering Active: "); Serial.println(dynamic_filtering.is_filtering_active() ? "YES" : "NO");
      Serial.print("Noise Level: "); Serial.println(dynamic_filtering.get_noise_level());
      Serial.println("=================================");
    }
  }
  else if (command.startsWith("PID PERFORMANCE")) {
    PerformanceMetrics pm = pid_controller.get_performance_metrics();
    Serial.println("=== PID Performance Metrics ===");
    Serial.print("Control Effort (Roll/Pitch/Yaw): ");
    Serial.print(pm.control_effort[0]); Serial.print(" / ");
    Serial.print(pm.control_effort[1]); Serial.print(" / ");
    Serial.println(pm.control_effort[2]);
    Serial.print("Efficiency Score: "); Serial.print(pm.efficiency_score); Serial.println(" %");
    Serial.println("================================");
  }
  else if (command.startsWith("PID ADAPTIVE")) {
    if (command.indexOf("ENABLE") > 0 || command.indexOf("ON") > 0) {
      pid_controller.get_advanced_features().adaptive_gains_enabled = true;
      Serial.println("Adaptive PID gains ENABLED");
    } else if (command.indexOf("DISABLE") > 0 || command.indexOf("OFF") > 0) {
      pid_controller.get_advanced_features().adaptive_gains_enabled = false;
      Serial.println("Adaptive PID gains DISABLED");
    }
  }
  else if (command.startsWith("OPTICAL FLOW")) {
    #ifdef ENABLE_OPTICAL_FLOW
    if (command.indexOf("STATUS") > 0) {
      OpticalFlowData flow = optical_flow.get_data();
      Serial.println("=== Optical Flow Status ===");
      Serial.print("Data Valid: "); Serial.println(flow.data_valid ? "YES" : "NO");
      Serial.print("Velocity X: "); Serial.print(flow.velocity_x); Serial.println(" m/s");
      Serial.print("Velocity Y: "); Serial.print(flow.velocity_y); Serial.println(" m/s");
      Serial.println("===========================");
    }
    #else
    Serial.println("Optical flow not enabled");
    #endif
  }
  else if (command.startsWith("SET ESC PROTOCOL")) {
    // Example: SET ESC PROTOCOL DSHOT600
    int idx = command.lastIndexOf(' ');
    if (idx > 0) {
      String protoStr = command.substring(idx + 1);
      EscProtocol newProto = ESC_PROTOCOL_PWM;
      if (protoStr == "PWM") newProto = ESC_PROTOCOL_PWM;
      else if (protoStr == "ONESHOT125") newProto = ESC_PROTOCOL_ONESHOT125;
      else if (protoStr == "ONESHOT42") newProto = ESC_PROTOCOL_ONESHOT42;
      else if (protoStr == "MULTISHOT") newProto = ESC_PROTOCOL_MULTISHOT;
      else if (protoStr == "DSHOT150") newProto = ESC_PROTOCOL_DSHOT150;
      else if (protoStr == "DSHOT300") newProto = ESC_PROTOCOL_DSHOT300;
      else if (protoStr == "DSHOT600") newProto = ESC_PROTOCOL_DSHOT600;
      else if (protoStr == "DSHOT1200") newProto = ESC_PROTOCOL_DSHOT1200;
      motor_control.set_esc_protocol(newProto);
      Serial.print("ESC protocol set to "); Serial.println(protoStr);
    }
  }
  else if (command.startsWith("SET PID")) {
    // Syntax: SET PID ROLL P 1.2
    // tokens: SET PID <AXIS> <P|I|D> value
    int first = command.indexOf(' ', 7); // after "SET PID"
    if (first > 0) {
      String axisStr = command.substring(7, first);
      int second = command.indexOf(' ', first + 1);
      if (second > 0) {
        String termStr = command.substring(first + 1, second);
        float val = command.substring(second + 1).toFloat();
        int axis = 0; // roll default
        if (axisStr == "ROLL") axis = 0;
        else if (axisStr == "PITCH") axis = 1;
        else if (axisStr == "YAW") axis = 2;
        if (termStr == "P") pid_controller.set_rate_pid_gains(axis, val, pid_controller.get_cascaded_gains().rate_roll.ki, pid_controller.get_cascaded_gains().rate_roll.kd);
        else if (termStr == "I") pid_controller.set_rate_pid_gains(axis, pid_controller.get_cascaded_gains().rate_roll.kp, val, pid_controller.get_cascaded_gains().rate_roll.kd);
        else if (termStr == "D") pid_controller.set_rate_pid_gains(axis, pid_controller.get_cascaded_gains().rate_roll.kp, pid_controller.get_cascaded_gains().rate_roll.ki, val);
        Serial.println("PID term updated");
      }
    }
  }
  else if (command == "CALIBRATION WIZARD") {
    start_calibration_wizard();
  }
  else if (command.startsWith("BLACKBOX")) {
    if (command.indexOf("CLEAR")>0) {
      extern BlackboxLogger blackbox;
      blackbox.clearLogs();
      Serial.println("Blackbox logs cleared");
    } else if (command.indexOf("STATUS")>0) {
      File root=SD.open("/"); int count=0; if(root){File f=root.openNextFile(); while(f){if(!f.isDirectory() && f.name()[0]=='b' && f.name()[1]=='b') count++; f.close(); f=root.openNextFile();} root.close();}
      Serial.print("Blackbox log files: "); Serial.println(count);
    } else {
      Serial.println("BLACKBOX commands: STATUS | CLEAR");
    }
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
  Serial.println("  arm                      - Arm motors (via RC)");
  Serial.println("  disarm                   - Disarm motors");
  Serial.println("  calibrate esc            - Calibrate ESCs");
  Serial.println("");
  Serial.println("Safety & Status Commands:");
  Serial.println("  safety check             - Enhanced pre-flight safety check");
  Serial.println("  gps status               - GPS system status");
  Serial.println("  battery status           - Battery status");
  Serial.println("  calibration check        - Check calibration status");
  Serial.println("");
  Serial.println("Sensor Commands:");
  Serial.println("  calibrate imu            - Calibrate IMU");
  Serial.println("  calibrate gyro           - Calibrate gyroscope");
  Serial.println("  calibrate accel          - Calibrate accelerometer");
  Serial.println("  calibrate mag            - Calibrate magnetometer");
  Serial.println("");
  Serial.println("Sensor Detection:");
  Serial.println("  detect sensors           - Scan for all sensors");
  Serial.println("  sensor status            - Show detected sensors");
  Serial.println("  i2c scan                 - Scan I2C bus");
  Serial.println("");
  Serial.println("Advanced Systems:");
  Serial.println("  dual imu status          - Dual IMU system status");
  Serial.println("  filtering status         - Dynamic filtering status");
  Serial.println("  optical flow status      - Optical flow sensor status");
  Serial.println("  redundancy status        - Sensor redundancy status");
  Serial.println("  sensor health            - Individual sensor health");
  Serial.println("  blackbox status          - Show log count");
  Serial.println("  blackbox clear           - Delete all log files");
  Serial.println("");
  Serial.println("PID & Adaptive Control:");
  Serial.println("  pid performance           - Show PID performance metrics");
  Serial.println("  pid adaptive enable       - Enable adaptive gain scheduling");
  Serial.println("  pid adaptive disable      - Disable adaptive gain scheduling");
  Serial.println("");
  Serial.println("Other Commands:");
  Serial.println("  reboot                   - Restart controller");
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
  Serial.print(sensor_data.baro.altitude);
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

void start_calibration_wizard(){
  wizard_step = WIZ_START;
  Serial.println("\n=== Calibration Wizard ===\nType 'next' to progress, 'abort' to exit.");
  Serial.println("Step 1: Place drone motionless on level surface.");
}

void wizard_next(){
  switch(wizard_step){
    case WIZ_START:
      Serial.println("Calibrating Gyro...");
      if(perform_gyro_calibration()){ Serial.println("Gyro OK"); }
      wizard_step = WIZ_GYRO;
      Serial.println("Step 2: Accelerometer calibration – follow on-screen prompts.");
      break;
    case WIZ_GYRO:
      if(perform_accelerometer_calibration()){Serial.println("Accel OK");}
      wizard_step = WIZ_ACCEL;
      Serial.println("Step 3: Rotate drone slowly for Magnetometer calibration then type 'next'.");
      break;
    case WIZ_ACCEL:
      calibrate_magnetometer();
      wizard_step = WIZ_MAG;
      Serial.println("Step 4: ESC calibration will send max then min throttle, props removed? (yes/no)");
      break;
    case WIZ_MAG:
      Serial.println("Starting ESC calibration in 3s..."); delay(3000); motor_control.calibrate_escs();
      wizard_step = WIZ_ESC;
      Serial.println("Step 5: Motor direction test, ensure props removed. Type 'next' to continue");
      break;
    case WIZ_ESC:
      if(perform_motor_direction_test()) {
        Serial.println("Motor direction test passed ✅");
      } else {
        Serial.println("Motor directions corrected where needed ❗️");
      }
      // Give ESCs time to persist settings
      delay(500);
      wizard_step = WIZ_MOTOR_DIR;
      Serial.println("Saving calibration to EEPROM...");
      save_calibration_to_eeprom();
      wizard_step = WIZ_DONE;
      Serial.println("Wizard finished. Reboot or enjoy flight!");
      break;
    default: break;
  }
}

// Motor direction test using gyro Z-rate during DShot beep
bool perform_motor_direction_test() {
  Serial.println("Starting automatic motor direction test (gyro-based)…");
  const int expected_pattern[4] = { 1, -1, 1, -1 }; // 1=CW group, -1=CCW group relative pattern
  int measured_sign[4] = {0,0,0,0};
  int baseline_sign = 0;
  const float MIN_GZ_THRESHOLD = 30.0f; // deg/s

  for (int m = 0; m < 4; m++) {
    Serial.print("  > Testing motor "); Serial.println(m+1);
    // Send 3 short beep commands to excite motor
    for (int k = 0; k < 3; k++) {
      motor_control.send_dshot_command_motor(m, DSHOT_CMD_BEEP1);
      delay(60);
    }

    // Collect gyro samples for 250 ms
    unsigned long tStart = millis();
    float sum = 0; int cnt = 0;
    while (millis() - tStart < 250) {
      IMUData d; if (read_imu_data(&d)) { sum += d.gyro_z; cnt++; }
      delay(5);
    }
    if (cnt == 0) { Serial.println("     Unable to read gyro – skipping"); continue; }
    float avg = sum / cnt;
    if (fabs(avg) < MIN_GZ_THRESHOLD) {
      Serial.println("     Rotation too small to detect – check props removed?");
      continue;
    }
    int sign = (avg > 0) ? 1 : -1;
    measured_sign[m] = sign;
    Serial.print("     Gyro Z avg: "); Serial.print(avg,1); Serial.print(" deg/s → sign "); Serial.println(sign);
    if (m == 0) baseline_sign = sign; // use first motor as baseline for CW group
  }

  bool all_ok = true;
  for (int m = 0; m < 4; m++) {
    if (measured_sign[m] == 0) { all_ok = false; continue; }
    int expected_sign = expected_pattern[m] * baseline_sign;
    if (measured_sign[m] != expected_sign) {
      Serial.print("Motor "); Serial.print(m+1); Serial.println(" spin direction WRONG – auto-reversing");
      motor_control.reverse_motor_direction(m);
      all_ok = false;
    } else {
      Serial.print("Motor "); Serial.print(m+1); Serial.println(" direction OK");
    }
  }

  if (all_ok) {
    Serial.println("All motor directions correct ✔");
  } else {
    Serial.println("Motor direction corrections applied – saving to ESCs");
    motor_control.send_dshot_command_all(DSHOT_CMD_SAVE_SETTINGS);
    delay(100);
  }
  return all_ok;
} 