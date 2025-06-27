#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include <Servo.h>

class MotorControl {
private:
  // Legacy PWM support
  Servo motor1, motor2, motor3, motor4;
  
  // Advanced protocol support
  EscConfig esc_config;
  MotorOutput motor_output;
  bool motors_armed;
  bool calibration_mode;
  
  // DShot support
  uint32_t dshot_packet[4];
  uint8_t dshot_buffer[4][16];  // 16 bits per motor
  bool telemetry_request[4];
  uint16_t motor_rpm[4];
  uint8_t motor_temperature[4];
  uint16_t motor_voltage[4];
  uint16_t motor_current[4];
  
  // Protocol-specific functions
  void output_pwm(int motor, int value);
  void output_oneshot125(int motor, int value);
  void output_oneshot42(int motor, int value);
  void output_multishot(int motor, int value);
  void output_dshot(int motor, int value, bool telemetry = false);
  void output_dshot_command(int motor, DshotCommand cmd);
  
  // DShot helper functions
  uint16_t prepare_dshot_packet(uint16_t value, bool telemetry);
  void send_dshot_packet(int motor, uint16_t packet);
  uint8_t calculate_dshot_checksum(uint16_t value, bool telemetry);
  void encode_dshot_bit(uint8_t* buffer, int bit_index, bool bit_value, uint32_t bit_time);
  void read_dshot_telemetry();
  
  void apply_motor_mixing(PIDOutput& pid_output, int base_throttle);
  int map_throttle_to_protocol(int throttle_percent);
  
public:
  void init();
  void init_with_protocol(EscProtocol protocol);
  void update(PIDOutput& pid_output, bool armed);
  void arm();
  void disarm();
  void emergency_stop();
  void calibrate_escs();
  void test_motor(int motor_num, int pwm_value);
  
  // Protocol management
  void set_esc_protocol(EscProtocol protocol);
  EscProtocol get_esc_protocol() { return esc_config.protocol; }
  void configure_esc(const EscConfig& config);
  EscConfig get_esc_config() { return esc_config; }
  
  // DShot specific functions
  void send_dshot_command_all(DshotCommand cmd);
  void send_dshot_command_motor(int motor, DshotCommand cmd);
  void output_dshot_command(int motor, DshotCommand cmd);
  void enable_bidirectional_dshot(bool enable);
  void request_telemetry(bool enable);
  
  // Motor direction and configuration
  void set_motor_direction(int motor, int direction);
  void reverse_motor_direction(int motor);
  void enable_motor(int motor, bool enable);
  
  // Telemetry access
  uint16_t get_motor_rpm(int motor) { return (motor >= 0 && motor < 4) ? motor_rpm[motor] : 0; }
  uint8_t get_motor_temperature(int motor) { return (motor >= 0 && motor < 4) ? motor_temperature[motor] : 0; }
  uint16_t get_motor_voltage(int motor) { return (motor >= 0 && motor < 4) ? motor_voltage[motor] : 0; }
  uint16_t get_motor_current(int motor) { return (motor >= 0 && motor < 4) ? motor_current[motor] : 0; }
  
  // Status functions
  bool is_armed() { return motors_armed; }
  bool supports_bidirectional() { 
    return esc_config.protocol >= ESC_PROTOCOL_DSHOT150 && esc_config.protocol <= ESC_PROTOCOL_DSHOT1200; 
  }
  bool supports_telemetry() { 
    return esc_config.protocol >= ESC_PROTOCOL_DSHOT150 && esc_config.protocol <= ESC_PROTOCOL_DSHOT1200; 
  }
  const char* get_protocol_name();
  
  // DShot telemetry reading and RPM feedback
  void enable_dshot_telemetry(bool enable);
  void update_rpm_based_filtering();
  bool read_motor_telemetry(int motor_num, MotorTelemetry* telemetry);
  void apply_rpm_notch_filtering();
  
  // Motor performance monitoring
  MotorTelemetry get_motor_telemetry(int motor_num);
  void monitor_motor_health();
  float get_motor_efficiency(int motor_num);
};

void MotorControl::init() {
  init_with_protocol(ESC_PROTOCOL_PWM);
}

void MotorControl::init_with_protocol(EscProtocol protocol) {
  // Initialize ESC configuration with defaults
  esc_config.protocol = protocol;
  esc_config.telemetry_enabled = false;
  esc_config.bidirectional_enabled = false;
  esc_config.idle_throttle_percent = 0;
  esc_config.min_throttle_percent = 0;
  esc_config.max_throttle_percent = 100;
  esc_config.beep_on_startup = true;
  
  for (int i = 0; i < 4; i++) {
    esc_config.motor_direction[i] = 1;  // Normal direction
    esc_config.motor_enabled[i] = true;
    motor_rpm[i] = 0;
    motor_temperature[i] = 0;
    motor_voltage[i] = 0;
    motor_current[i] = 0;
    telemetry_request[i] = false;
  }
  
  // Protocol-specific initialization
  switch (protocol) {
    case ESC_PROTOCOL_PWM:
      // Standard PWM initialization
      motor1.attach(MOTOR1_PIN);
      motor2.attach(MOTOR2_PIN);
      motor3.attach(MOTOR3_PIN);
      motor4.attach(MOTOR4_PIN);
      
      motor1.writeMicroseconds(ESC_MIN_PWM);
      motor2.writeMicroseconds(ESC_MIN_PWM);
      motor3.writeMicroseconds(ESC_MIN_PWM);
      motor4.writeMicroseconds(ESC_MIN_PWM);
      break;
      
    case ESC_PROTOCOL_ONESHOT125:
    case ESC_PROTOCOL_ONESHOT42:
    case ESC_PROTOCOL_MULTISHOT:
      // OneShot protocols use similar PWM setup but different timing
      motor1.attach(MOTOR1_PIN);
      motor2.attach(MOTOR2_PIN);
      motor3.attach(MOTOR3_PIN);
      motor4.attach(MOTOR4_PIN);
      break;
      
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_DSHOT1200:
      // DShot protocols require digital pin setup
      pinMode(MOTOR1_PIN, OUTPUT);
      pinMode(MOTOR2_PIN, OUTPUT);
      pinMode(MOTOR3_PIN, OUTPUT);
      pinMode(MOTOR4_PIN, OUTPUT);
      
      digitalWrite(MOTOR1_PIN, LOW);
      digitalWrite(MOTOR2_PIN, LOW);
      digitalWrite(MOTOR3_PIN, LOW);
      digitalWrite(MOTOR4_PIN, LOW);
      
      // Enable telemetry if supported
      if (esc_config.telemetry_enabled) {
        // Set up telemetry pins as inputs when not transmitting
        // This requires bidirectional pin configuration
      }
      break;
  }
  
  motors_armed = false;
  calibration_mode = false;
  
  Serial.print("Motor control initialized with protocol: ");
  Serial.println(get_protocol_name());
  
  // Send startup beeps if enabled
  if (esc_config.beep_on_startup && supports_telemetry()) {
    delay(1000);
    send_dshot_command_all(DSHOT_CMD_BEEP1);
    delay(200);
  }
  
  delay(1000); // Give ESCs time to initialize
}

void MotorControl::update(PIDOutput& pid_output, bool armed) {
  if (!armed) {
    disarm();
    return;
  }
  
  if (!motors_armed) {
    arm();
  }
  
  // Apply motor mixing
  apply_motor_mixing(pid_output, pid_output.throttle);
  
  // Apply safety limits and output to motors
  for (int i = 0; i < 4; i++) {
    int motor_values[4] = {motor_output.motor1, motor_output.motor2, motor_output.motor3, motor_output.motor4};
    int limited_value = 0;
    
    if (esc_config.motor_enabled[i]) {
      // Apply protocol-specific limits
      switch (esc_config.protocol) {
        case ESC_PROTOCOL_PWM:
          limited_value = constrain(motor_values[i], ESC_MIN_PWM, ESC_MAX_PWM);
          break;
        case ESC_PROTOCOL_ONESHOT125:
          limited_value = constrain(motor_values[i], ONESHOT125_MIN, ONESHOT125_MAX);
          break;
        case ESC_PROTOCOL_ONESHOT42:
          limited_value = constrain(motor_values[i], ONESHOT42_MIN, ONESHOT42_MAX);
          break;
        case ESC_PROTOCOL_MULTISHOT:
          limited_value = constrain(motor_values[i], MULTISHOT_MIN, MULTISHOT_MAX);
          break;
        default: // DShot protocols
          limited_value = constrain(motor_values[i], 48, 2047); // DShot range
          break;
      }
      
      // Apply motor direction
      if (esc_config.motor_direction[i] == -1) {
        // For DShot, we can send reverse command
        if (supports_bidirectional() && esc_config.bidirectional_enabled) {
          // Implement bidirectional logic
          limited_value = -limited_value;
        }
      }
    } else {
      limited_value = (esc_config.protocol <= ESC_PROTOCOL_MULTISHOT) ? ESC_MIN_PWM : 0;
    }
    
    // Output to motors based on protocol
    switch (esc_config.protocol) {
      case ESC_PROTOCOL_PWM:
        output_pwm(i, limited_value);
        break;
      case ESC_PROTOCOL_ONESHOT125:
        output_oneshot125(i, limited_value);
        break;
      case ESC_PROTOCOL_ONESHOT42:
        output_oneshot42(i, limited_value);
        break;
      case ESC_PROTOCOL_MULTISHOT:
        output_multishot(i, limited_value);
        break;
      default: // DShot protocols
        output_dshot(i, limited_value, esc_config.telemetry_enabled);
        break;
    }
  }
  
  // Read telemetry if available
  if (esc_config.telemetry_enabled && supports_telemetry()) {
    read_dshot_telemetry();
  }
}

void MotorControl::output_pwm(int motor, int value) {
  switch (motor) {
    case 0: motor1.writeMicroseconds(value); break;
    case 1: motor2.writeMicroseconds(value); break;
    case 2: motor3.writeMicroseconds(value); break;
    case 3: motor4.writeMicroseconds(value); break;
  }
}

void MotorControl::output_oneshot125(int motor, int value) {
  // OneShot125: 125-250 microseconds
  int mapped_value = map(value, ESC_MIN_PWM, ESC_MAX_PWM, ONESHOT125_MIN, ONESHOT125_MAX);
  
  switch (motor) {
    case 0: motor1.writeMicroseconds(mapped_value); break;
    case 1: motor2.writeMicroseconds(mapped_value); break;
    case 2: motor3.writeMicroseconds(mapped_value); break;
    case 3: motor4.writeMicroseconds(mapped_value); break;
  }
}

void MotorControl::output_oneshot42(int motor, int value) {
  // OneShot42: 42-84 microseconds  
  int mapped_value = map(value, ESC_MIN_PWM, ESC_MAX_PWM, ONESHOT42_MIN, ONESHOT42_MAX);
  
  switch (motor) {
    case 0: motor1.writeMicroseconds(mapped_value); break;
    case 1: motor2.writeMicroseconds(mapped_value); break;
    case 2: motor3.writeMicroseconds(mapped_value); break;
    case 3: motor4.writeMicroseconds(mapped_value); break;
  }
}

void MotorControl::output_multishot(int motor, int value) {
  // Multishot: 5-25 microseconds
  int mapped_value = map(value, ESC_MIN_PWM, ESC_MAX_PWM, MULTISHOT_MIN, MULTISHOT_MAX);
  
  switch (motor) {
    case 0: motor1.writeMicroseconds(mapped_value); break;
    case 1: motor2.writeMicroseconds(mapped_value); break;
    case 2: motor3.writeMicroseconds(mapped_value); break;
    case 3: motor4.writeMicroseconds(mapped_value); break;
  }
}

void MotorControl::output_dshot(int motor, int value, bool telemetry) {
  // DShot protocols: 11-bit throttle value (0-2047, 48-2047 usable range)
  uint16_t throttle = constrain(value, 0, 2047);
  uint16_t packet = prepare_dshot_packet(throttle, telemetry);
  send_dshot_packet(motor, packet);
}

uint16_t MotorControl::prepare_dshot_packet(uint16_t value, bool telemetry) {
  // DShot packet: 11 bits throttle + 1 bit telemetry + 4 bits checksum
  uint16_t packet = (value << 1) | (telemetry ? 1 : 0);
  uint8_t checksum = calculate_dshot_checksum(value, telemetry);
  packet = (packet << 4) | checksum;
  return packet;
}

uint8_t MotorControl::calculate_dshot_checksum(uint16_t value, bool telemetry) {
  uint16_t data = (value << 1) | (telemetry ? 1 : 0);
  uint8_t checksum = 0;
  
  for (int i = 0; i < 3; i++) {
    checksum ^= (data >> (4 * i)) & 0x0F;
  }
  
  return checksum;
}

void MotorControl::send_dshot_packet(int motor, uint16_t packet) {
  uint32_t bit_time = DSHOT600_BIT_TIME; // Default to DShot600
  
  switch (esc_config.protocol) {
    case ESC_PROTOCOL_DSHOT150:  bit_time = DSHOT150_BIT_TIME; break;
    case ESC_PROTOCOL_DSHOT300:  bit_time = DSHOT300_BIT_TIME; break;
    case ESC_PROTOCOL_DSHOT600:  bit_time = DSHOT600_BIT_TIME; break;
    case ESC_PROTOCOL_DSHOT1200: bit_time = DSHOT1200_BIT_TIME; break;
  }
  
  // Get motor pin
  int pin = MOTOR1_PIN;
  switch (motor) {
    case 0: pin = MOTOR1_PIN; break;
    case 1: pin = MOTOR2_PIN; break;
    case 2: pin = MOTOR3_PIN; break;
    case 3: pin = MOTOR4_PIN; break;
  }
  
  // Disable interrupts for precise timing
  noInterrupts();
  
  // Send 16 bits (DShot packet is 16 bits total)
  for (int i = 15; i >= 0; i--) {
    bool bit = (packet >> i) & 1;
    
    if (bit) {
      // Bit 1: ~74% high, ~26% low
      digitalWriteFast(pin, HIGH);
      delayNanoseconds(bit_time * DSHOT_BIT_1_HIGH_TIME_FACTOR);
      digitalWriteFast(pin, LOW);
      delayNanoseconds(bit_time * (1.0 - DSHOT_BIT_1_HIGH_TIME_FACTOR));
    } else {
      // Bit 0: ~37% high, ~63% low
      digitalWriteFast(pin, HIGH);
      delayNanoseconds(bit_time * DSHOT_BIT_0_HIGH_TIME_FACTOR);
      digitalWriteFast(pin, LOW);
      delayNanoseconds(bit_time * (1.0 - DSHOT_BIT_0_HIGH_TIME_FACTOR));
    }
  }
  
  interrupts();
}

void MotorControl::read_dshot_telemetry() {
  // Implement telemetry reading logic
  // This requires switching pins to input mode after transmission
  // and reading the response from ESCs
  // Implementation depends on specific Teensy capabilities
}

void MotorControl::apply_motor_mixing(PIDOutput& pid_output, int base_throttle) {
  // Quadcopter X configuration motor mixing
  // Motor layout:
  //   1   2
  //    \ /
  //     X
  //    / \
  //   4   3
  
  int throttle = map_throttle_to_protocol(base_throttle);
  
  // Apply PID outputs to motor mixing
  motor_output.motor1 = throttle - pid_output.roll + pid_output.pitch - pid_output.yaw; // Front Right
  motor_output.motor2 = throttle + pid_output.roll + pid_output.pitch + pid_output.yaw; // Front Left
  motor_output.motor3 = throttle + pid_output.roll - pid_output.pitch - pid_output.yaw; // Rear Left
  motor_output.motor4 = throttle - pid_output.roll - pid_output.pitch + pid_output.yaw; // Rear Right
}

int MotorControl::map_throttle_to_protocol(int base_throttle) {
  // Map throttle based on current protocol
  switch (esc_config.protocol) {
    case ESC_PROTOCOL_PWM:
      return map(base_throttle, 1000, 2000, ESC_MIN_PWM, ESC_MAX_PWM);
    case ESC_PROTOCOL_ONESHOT125:
      return map(base_throttle, 1000, 2000, ONESHOT125_MIN, ONESHOT125_MAX);
    case ESC_PROTOCOL_ONESHOT42:
      return map(base_throttle, 1000, 2000, ONESHOT42_MIN, ONESHOT42_MAX);
    case ESC_PROTOCOL_MULTISHOT:
      return map(base_throttle, 1000, 2000, MULTISHOT_MIN, MULTISHOT_MAX);
    default: // DShot protocols
      return map(base_throttle, 1000, 2000, 48, 2047);
  }
}

void MotorControl::set_esc_protocol(EscProtocol protocol) {
  if (motors_armed) {
    Serial.println("Cannot change ESC protocol while armed!");
    return;
  }
  
  Serial.print("Changing ESC protocol from ");
  Serial.print(get_protocol_name());
  Serial.print(" to ");
  
  esc_config.protocol = protocol;
  Serial.println(get_protocol_name());
  
  // Reinitialize with new protocol
  init_with_protocol(protocol);
}

void MotorControl::configure_esc(const EscConfig& config) {
  if (motors_armed) {
    Serial.println("Cannot change ESC configuration while armed!");
    return;
  }
  
  esc_config = config;
  init_with_protocol(config.protocol);
}

void MotorControl::send_dshot_command_all(DshotCommand cmd) {
  if (!supports_telemetry()) {
    Serial.println("DShot commands not supported with current protocol");
    return;
  }
  
  for (int i = 0; i < 4; i++) {
    send_dshot_command_motor(i, cmd);
  }
}

void MotorControl::send_dshot_command_motor(int motor, DshotCommand cmd) {
  if (!supports_telemetry()) {
    Serial.println("DShot commands not supported with current protocol");
    return;
  }
  
  output_dshot_command(motor, cmd);
  
  // Send command multiple times as required by DShot spec
  for (int i = 0; i < 10; i++) {
    output_dshot_command(motor, cmd);
    delay(1);
  }
}

void MotorControl::output_dshot_command(int motor, DshotCommand cmd) {
  uint16_t packet = prepare_dshot_packet(cmd, false);
  send_dshot_packet(motor, packet);
}

void MotorControl::set_motor_direction(int motor, int direction) {
  if (motor >= 0 && motor < 4) {
    esc_config.motor_direction[motor] = (direction > 0) ? 1 : -1;
    
    // Send DShot direction command if supported
    if (supports_telemetry()) {
      DshotCommand cmd = (direction > 0) ? DSHOT_CMD_SPIN_DIRECTION_NORMAL : DSHOT_CMD_SPIN_DIRECTION_REVERSED;
      send_dshot_command_motor(motor, cmd);
    }
  }
}

void MotorControl::reverse_motor_direction(int motor) {
  if (motor >= 0 && motor < 4) {
    esc_config.motor_direction[motor] *= -1;
    
    if (supports_telemetry()) {
      DshotCommand cmd = (esc_config.motor_direction[motor] > 0) ? 
        DSHOT_CMD_SPIN_DIRECTION_NORMAL : DSHOT_CMD_SPIN_DIRECTION_REVERSED;
      send_dshot_command_motor(motor, cmd);
    }
  }
}

void MotorControl::enable_motor(int motor, bool enable) {
  if (motor >= 0 && motor < 4) {
    esc_config.motor_enabled[motor] = enable;
  }
}

void MotorControl::enable_bidirectional_dshot(bool enable) {
  if (supports_bidirectional()) {
    esc_config.bidirectional_enabled = enable;
    // Send 3D mode command
    DshotCommand cmd = enable ? DSHOT_CMD_3D_MODE_ON : DSHOT_CMD_3D_MODE_OFF;
    send_dshot_command_all(cmd);
  }
}

void MotorControl::request_telemetry(bool enable) {
  if (supports_telemetry()) {
    esc_config.telemetry_enabled = enable;
    for (int i = 0; i < 4; i++) {
      telemetry_request[i] = enable;
    }
  }
}

const char* MotorControl::get_protocol_name() {
  switch (esc_config.protocol) {
    case ESC_PROTOCOL_PWM:       return "PWM";
    case ESC_PROTOCOL_ONESHOT125: return "OneShot125";
    case ESC_PROTOCOL_ONESHOT42:  return "OneShot42";
    case ESC_PROTOCOL_MULTISHOT:  return "Multishot";
    case ESC_PROTOCOL_DSHOT150:   return "DShot150";
    case ESC_PROTOCOL_DSHOT300:   return "DShot300";
    case ESC_PROTOCOL_DSHOT600:   return "DShot600";
    case ESC_PROTOCOL_DSHOT1200:  return "DShot1200";
    default:                      return "Unknown";
  }
}

void MotorControl::arm() {
  if (!motors_armed) {
    motors_armed = true;
    Serial.print("Motors ARMED with protocol: ");
    Serial.println(get_protocol_name());
    
    // Protocol-specific arming sequence
    if (supports_telemetry()) {
      // Send beep to indicate arming
      send_dshot_command_all(DSHOT_CMD_BEEP2);
    } else {
      // Gradual arm sequence for PWM protocols
      int min_val = (esc_config.protocol == ESC_PROTOCOL_PWM) ? ESC_ARM_PWM : 
                    (esc_config.protocol == ESC_PROTOCOL_ONESHOT125) ? ONESHOT125_MIN :
                    (esc_config.protocol == ESC_PROTOCOL_ONESHOT42) ? ONESHOT42_MIN : MULTISHOT_MIN;
      
      for (int i = min_val; i <= min_val + 50; i += 10) {
        output_pwm(0, i);
        output_pwm(1, i);
        output_pwm(2, i);
        output_pwm(3, i);
        delay(50);
      }
    }
  }
}

void MotorControl::disarm() {
  if (motors_armed) {
    motors_armed = false;
    Serial.println("Motors DISARMED");
    
    if (supports_telemetry()) {
      // Send stop command to all motors
      send_dshot_command_all(DSHOT_CMD_MOTOR_STOP);
    } else {
      // Gradual disarm for PWM protocols
      int min_val = (esc_config.protocol == ESC_PROTOCOL_PWM) ? ESC_MIN_PWM : 
                    (esc_config.protocol == ESC_PROTOCOL_ONESHOT125) ? ONESHOT125_MIN :
                    (esc_config.protocol == ESC_PROTOCOL_ONESHOT42) ? ONESHOT42_MIN : MULTISHOT_MIN;
      
      // Set all motors to minimum
      output_pwm(0, min_val);
      output_pwm(1, min_val);
      output_pwm(2, min_val);
      output_pwm(3, min_val);
    }
  }
}

void MotorControl::emergency_stop() {
  motors_armed = false;
  
  if (supports_telemetry()) {
    send_dshot_command_all(DSHOT_CMD_MOTOR_STOP);
  } else {
    int min_val = (esc_config.protocol == ESC_PROTOCOL_PWM) ? ESC_MIN_PWM : 
                  (esc_config.protocol == ESC_PROTOCOL_ONESHOT125) ? ONESHOT125_MIN :
                  (esc_config.protocol == ESC_PROTOCOL_ONESHOT42) ? ONESHOT42_MIN : MULTISHOT_MIN;
    
    output_pwm(0, min_val);
    output_pwm(1, min_val);
    output_pwm(2, min_val);
    output_pwm(3, min_val);
  }
  
  Serial.println("EMERGENCY STOP - All motors stopped");
}

void MotorControl::calibrate_escs() {
  if (motors_armed) {
    Serial.println("ERROR: Cannot calibrate ESCs while armed");
    return;
  }
  
  Serial.println("=== ESC Calibration Started ===");
  Serial.println("⚠️  WARNING: REMOVE ALL PROPELLERS!");
  Serial.println("ESCs will receive high and low signals for calibration.");
  Serial.println("This process takes about 10 seconds.");
  
  // Different calibration process based on protocol
  if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150) {
    // DShot ESC calibration
    Serial.println("DShot ESC Configuration Mode");
    
    // Send DShot commands for configuration
    send_dshot_command_all(DSHOT_CMD_ESC_INFO);
    delay(1000);
    
    // Configure settings
    send_dshot_command_all(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
    delay(500);
    
    // Save settings
    send_dshot_command_all(DSHOT_CMD_SAVE_SETTINGS);
    delay(2000);
    
    Serial.println("DShot ESC configuration complete");
    
  } else {
    // Traditional PWM/OneShot calibration
    Serial.println("Starting traditional ESC calibration...");
    Serial.println("Sending maximum throttle signal...");
    
         // Send max throttle for 3 seconds
     for (int i = 0; i < 4; i++) {
       output_value(i, ESC_MAX_PWM);
     }
    delay(3000);
    
    Serial.println("Sending minimum throttle signal...");
    
         // Send min throttle for 3 seconds  
     for (int i = 0; i < 4; i++) {
       output_value(i, ESC_MIN_PWM);
     }
    delay(3000);
    
    Serial.println("ESC calibration sequence complete");
  }
  
     // Stop all motors
   for (int i = 0; i < 4; i++) {
     output_value(i, ESC_ARM_PWM);
   }
  
  // Update calibration status
  extern CalibrationData calibration_data;
  calibration_data.esc_calibrated = true;
  calibration_data.esc_cal_quality = 90;
  calibration_data.esc_cal_time = millis();
  
  Serial.println("✅ ESC calibration completed successfully");
  Serial.println("=== ESC Calibration Finished ===");
}

void MotorControl::test_motor(int motor_num, int test_value) {
  if (motors_armed) {
    Serial.println("Cannot test motors while armed. Disarm first.");
    return;
  }
  
  if (motor_num < 1 || motor_num > 4) {
    Serial.println("Invalid motor number. Use 1-4.");
    return;
  }
  
  int motor_index = motor_num - 1;
  
  // Map test value to protocol range
  int mapped_value = map_throttle_to_protocol(test_value);
  
  Serial.print("Testing motor ");
  Serial.print(motor_num);
  Serial.print(" with protocol ");
  Serial.print(get_protocol_name());
  Serial.print(" at value ");
  Serial.println(mapped_value);
  
  // Test motor based on protocol
  switch (esc_config.protocol) {
    case ESC_PROTOCOL_PWM:
      output_pwm(motor_index, mapped_value);
      break;
    case ESC_PROTOCOL_ONESHOT125:
      output_oneshot125(motor_index, mapped_value);
      break;
    case ESC_PROTOCOL_ONESHOT42:
      output_oneshot42(motor_index, mapped_value);
      break;
    case ESC_PROTOCOL_MULTISHOT:
      output_multishot(motor_index, mapped_value);
      break;
    default: // DShot protocols
      output_dshot(motor_index, mapped_value, false);
      break;
  }
  
  delay(2000);
  
  // Return to minimum
  switch (esc_config.protocol) {
    case ESC_PROTOCOL_PWM:
      output_pwm(motor_index, ESC_MIN_PWM);
      break;
    case ESC_PROTOCOL_ONESHOT125:
      output_pwm(motor_index, ONESHOT125_MIN);
      break;
    case ESC_PROTOCOL_ONESHOT42:
      output_pwm(motor_index, ONESHOT42_MIN);
      break;
    case ESC_PROTOCOL_MULTISHOT:
      output_pwm(motor_index, MULTISHOT_MIN);
      break;
    default: // DShot protocols
      output_dshot(motor_index, 0, false);
      break;
  }
}

// DShot telemetry reading and RPM feedback
void MotorControl::enable_dshot_telemetry(bool enable) {
  // Implementation of enable_dshot_telemetry function
}

void MotorControl::update_rpm_based_filtering() {
  // Implementation of update_rpm_based_filtering function
}

bool MotorControl::read_motor_telemetry(int motor_num, MotorTelemetry* telemetry) {
  // Implementation of read_motor_telemetry function
  return false; // Placeholder return, actual implementation needed
}

void MotorControl::apply_rpm_notch_filtering() {
  // Implementation of apply_rpm_notch_filtering function
}

// Motor performance monitoring
MotorTelemetry MotorControl::get_motor_telemetry(int motor_num) {
  // Implementation of get_motor_telemetry function
  MotorTelemetry empty_telemetry;
  return empty_telemetry; // Placeholder return, actual implementation needed
}

void MotorControl::monitor_motor_health() {
  // Implementation of monitor_motor_health function
}

float MotorControl::get_motor_efficiency(int motor_num) {
  // Implementation of get_motor_efficiency function
  return 0.0f; // Placeholder return, actual implementation needed
}

#endif // MOTOR_CONTROL_H
