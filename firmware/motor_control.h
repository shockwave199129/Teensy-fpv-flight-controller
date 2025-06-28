#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include <Servo.h>

struct AdvancedMotorFeatures {
  // RPM-based dynamic filtering
  bool rpm_based_filtering_enabled;
  float rpm_notch_frequencies[4][3];    // 3 harmonics per motor
  float rpm_filter_q_factor;
  bool auto_rpm_filter_tuning;
  
  // Motor health monitoring
  bool motor_health_monitoring_enabled;
  float motor_vibration_levels[4];
  float motor_efficiency_scores[4];
  float motor_temperature_limits[4];
  bool motor_warning_flags[4];
  
  // Enhanced motor mixing
  bool advanced_mixing_enabled;
  float motor_thrust_scaling[4];        // Individual motor thrust scaling
  float dynamic_motor_saturation_comp;  // Dynamic saturation compensation
  bool torque_based_mixing;             // Torque-based vs thrust-based mixing
  
  // Predictive motor control
  bool predictive_control_enabled;
  float motor_response_prediction[4];   // Predicted motor response times
  float thrust_lag_compensation;        // Compensate for motor response lag
  
  // Adaptive motor characteristics
  float motor_kv_values[4];            // Actual KV ratings per motor
  float motor_resistance[4];           // Motor resistance for efficiency calc
  bool battery_compensation_enabled;    // Compensate for battery voltage sag
};

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
  
  // Advanced motor features
  AdvancedMotorFeatures advanced_motor;
  
  // Enhanced motor control methods
  void update_rpm_based_filtering();
  void monitor_motor_health_advanced();
  void apply_enhanced_motor_mixing(PIDOutput& pid_output, int base_throttle);
  void calculate_motor_efficiency_scores();
  void apply_predictive_motor_control();
  void compensate_battery_voltage_sag();
  void detect_motor_vibration_issues();
  void optimize_motor_timing();
  
  // Motor characterization methods
  void characterize_motor_response(int motor_num);
  void calibrate_motor_kv_values();
  void measure_motor_resistance();
  
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
  
  // Advanced motor features
  AdvancedMotorFeatures advanced_motor;
  
  // Enhanced motor control methods
  void update_rpm_based_filtering();
  void monitor_motor_health_advanced();
  void apply_enhanced_motor_mixing(PIDOutput& pid_output, int base_throttle);
  void calculate_motor_efficiency_scores();
  void apply_predictive_motor_control();
  void compensate_battery_voltage_sag();
  void detect_motor_vibration_issues();
  void optimize_motor_timing();
  
  // Motor characterization methods
  void characterize_motor_response(int motor_num);
  void calibrate_motor_kv_values();
  void measure_motor_resistance();
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

// =================== PHASE 3: ADVANCED MOTOR CONTROL IMPLEMENTATIONS ===================

void MotorControl::update_rpm_based_filtering() {
  if (!advanced_motor.rpm_based_filtering_enabled) return;
  
  extern SensorData sensor_data;
  extern NotchFilter motor_notch_filters[4];
  
  // Update RPM-based notch frequencies for each motor
  for (int i = 0; i < 4; i++) {
    uint16_t motor_rpm = get_motor_rpm(i);
    if (motor_rpm > 500) { // Only filter if motor is spinning
      // Calculate fundamental frequency and harmonics
      float fundamental_freq = (motor_rpm * 14) / 60.0f; // 14-pole motors
      
      // Store harmonic frequencies
      advanced_motor.rpm_notch_frequencies[i][0] = fundamental_freq;        // 1st harmonic
      advanced_motor.rpm_notch_frequencies[i][1] = fundamental_freq * 2;    // 2nd harmonic  
      advanced_motor.rpm_notch_frequencies[i][2] = fundamental_freq * 3;    // 3rd harmonic
      
      // Update notch filter with dominant frequency
      motor_notch_filters[i].center_freq_hz = fundamental_freq;
      motor_notch_filters[i].q_factor = advanced_motor.rpm_filter_q_factor;
      motor_notch_filters[i].enabled = true;
      
      // Recalculate filter coefficients
      calculate_notch_coefficients(&motor_notch_filters[i], 1000.0f);
    } else {
      motor_notch_filters[i].enabled = false;
    }
  }
}

void MotorControl::monitor_motor_health_advanced() {
  if (!advanced_motor.motor_health_monitoring_enabled) return;
  
  static unsigned long last_health_check = 0;
  if (millis() - last_health_check < 500) return; // 2Hz update
  
  for (int i = 0; i < 4; i++) {
    // Monitor motor temperature
    uint8_t temp = get_motor_temperature(i);
    if (temp > advanced_motor.motor_temperature_limits[i]) {
      advanced_motor.motor_warning_flags[i] = true;
      Serial.print("WARNING: Motor ");
      Serial.print(i + 1);
      Serial.print(" temperature high: ");
      Serial.println(temp);
    }
    
    // Calculate vibration level from RPM variance
    static uint16_t rpm_history[4][10] = {{0}};
    static int rpm_index = 0;
    
    rpm_history[i][rpm_index] = get_motor_rpm(i);
    
    if (rpm_index == 9) {
      float rpm_variance = 0;
      float mean_rpm = 0;
      for (int j = 0; j < 10; j++) {
        mean_rpm += rpm_history[i][j];
      }
      mean_rpm /= 10.0f;
      
      for (int j = 0; j < 10; j++) {
        rpm_variance += pow(rpm_history[i][j] - mean_rpm, 2);
      }
      rpm_variance /= 10.0f;
      
      advanced_motor.motor_vibration_levels[i] = sqrt(rpm_variance) / mean_rpm * 100.0f;
      
      // Detect excessive vibration
      if (advanced_motor.motor_vibration_levels[i] > 15.0f) {
        advanced_motor.motor_warning_flags[i] = true;
      } else {
        advanced_motor.motor_warning_flags[i] = false;
      }
    }
  }
  
  rpm_index = (rpm_index + 1) % 10;
  last_health_check = millis();
}

void MotorControl::apply_enhanced_motor_mixing(PIDOutput& pid_output, int base_throttle) {
  if (!advanced_motor.advanced_mixing_enabled) {
    apply_motor_mixing(pid_output, base_throttle); // Use standard mixing
    return;
  }
  
  // Enhanced quadcopter motor mixing with individual motor scaling
  float scaled_throttle = base_throttle;
  
  // Apply battery compensation
  if (advanced_motor.battery_compensation_enabled) {
    extern SensorData sensor_data;
    float battery_factor = 12.6f / sensor_data.battery_voltage; // Normalize to 3S full
    scaled_throttle *= constrain(battery_factor, 0.8f, 1.3f);
  }
  
  // Calculate motor outputs with thrust scaling
  float motor1_output = scaled_throttle + pid_output.pitch - pid_output.roll - pid_output.yaw;
  float motor2_output = scaled_throttle + pid_output.pitch + pid_output.roll + pid_output.yaw;
  float motor3_output = scaled_throttle - pid_output.pitch + pid_output.roll - pid_output.yaw;
  float motor4_output = scaled_throttle - pid_output.pitch - pid_output.roll + pid_output.yaw;
  
  // Apply individual motor thrust scaling
  motor1_output *= advanced_motor.motor_thrust_scaling[0];
  motor2_output *= advanced_motor.motor_thrust_scaling[1];
  motor3_output *= advanced_motor.motor_thrust_scaling[2];
  motor4_output *= advanced_motor.motor_thrust_scaling[3];
  
  // Dynamic saturation compensation
  float max_output = max(max(motor1_output, motor2_output), max(motor3_output, motor4_output));
  if (max_output > 2000) {
    float scale_factor = 2000.0f / max_output * advanced_motor.dynamic_motor_saturation_comp;
    motor1_output *= scale_factor;
    motor2_output *= scale_factor;
    motor3_output *= scale_factor;
    motor4_output *= scale_factor;
  }
  
  motor_output.motor1 = constrain(motor1_output, 1000, 2000);
  motor_output.motor2 = constrain(motor2_output, 1000, 2000);
  motor_output.motor3 = constrain(motor3_output, 1000, 2000);
  motor_output.motor4 = constrain(motor4_output, 1000, 2000);
}

void MotorControl::calculate_motor_efficiency_scores() {
  static unsigned long last_efficiency_calc = 0;
  if (millis() - last_efficiency_calc < 1000) return; // 1Hz update
  
  for (int i = 0; i < 4; i++) {
    uint16_t rpm = get_motor_rpm(i);
    uint16_t current = get_motor_current(i);
    uint16_t voltage = get_motor_voltage(i);
    
    if (rpm > 0 && current > 0 && voltage > 0) {
      // Calculate mechanical power (approximation)
      float mechanical_power = (rpm * 2 * PI / 60.0f) * 0.01f; // Simplified torque estimation
      
      // Calculate electrical power
      float electrical_power = (voltage / 1000.0f) * (current / 1000.0f);
      
      // Efficiency = mechanical power / electrical power
      if (electrical_power > 0) {
        advanced_motor.motor_efficiency_scores[i] = (mechanical_power / electrical_power) * 100.0f;
        advanced_motor.motor_efficiency_scores[i] = constrain(advanced_motor.motor_efficiency_scores[i], 0, 100);
      }
    } else {
      advanced_motor.motor_efficiency_scores[i] = 0;
    }
  }
  
  last_efficiency_calc = millis();
}

void MotorControl::apply_predictive_motor_control() {
  if (!advanced_motor.predictive_control_enabled) return;
  
  static float last_motor_outputs[4] = {0};
  static unsigned long last_prediction_time = 0;
  
  unsigned long current_time = micros();
  float dt = (current_time - last_prediction_time) / 1000000.0f;
  
  if (dt > 0.0001f) {
    for (int i = 0; i < 4; i++) {
      float current_output = 0;
      switch (i) {
        case 0: current_output = motor_output.motor1; break;
        case 1: current_output = motor_output.motor2; break;
        case 2: current_output = motor_output.motor3; break;
        case 3: current_output = motor_output.motor4; break;
      }
      
      // Calculate rate of change
      float output_rate = (current_output - last_motor_outputs[i]) / dt;
      
      // Predict future output based on current rate
      float predicted_output = current_output + output_rate * advanced_motor.thrust_lag_compensation;
      
      // Apply lag compensation
      predicted_output = constrain(predicted_output, 1000, 2000);
      
      // Store prediction for feedback
      advanced_motor.motor_response_prediction[i] = predicted_output;
      
      last_motor_outputs[i] = current_output;
    }
    
    last_prediction_time = current_time;
  }
}

void MotorControl::compensate_battery_voltage_sag() {
  if (!advanced_motor.battery_compensation_enabled) return;
  
  extern SensorData sensor_data;
  static float baseline_voltage = 12.6f; // 3S battery full charge
  static bool baseline_set = false;
  
  if (!baseline_set && sensor_data.battery_voltage > 12.0f) {
    baseline_voltage = sensor_data.battery_voltage;
    baseline_set = true;
  }
  
  // Calculate compensation factor
  float voltage_ratio = baseline_voltage / sensor_data.battery_voltage;
  float compensation_factor = constrain(voltage_ratio, 0.8f, 1.3f);
  
  // Apply compensation to motor thrust scaling
  for (int i = 0; i < 4; i++) {
    advanced_motor.motor_thrust_scaling[i] = compensation_factor;
  }
}

void MotorControl::detect_motor_vibration_issues() {
  static unsigned long last_vibration_check = 0;
  if (millis() - last_vibration_check < 200) return; // 5Hz update
  
  extern SensorData sensor_data;
  
  // Detect vibrations from accelerometer data
  static float accel_history[3][20] = {{0}};
  static int accel_index = 0;
  
  accel_history[0][accel_index] = sensor_data.imu.accel_x;
  accel_history[1][accel_index] = sensor_data.imu.accel_y;
  accel_history[2][accel_index] = sensor_data.imu.accel_z;
  
  accel_index = (accel_index + 1) % 20;
  
  // Calculate vibration magnitude
  float vibration_magnitude = 0;
  for (int axis = 0; axis < 3; axis++) {
    float variance = 0;
    float mean = 0;
    
    for (int i = 0; i < 20; i++) {
      mean += accel_history[axis][i];
    }
    mean /= 20.0f;
    
    for (int i = 0; i < 20; i++) {
      variance += pow(accel_history[axis][i] - mean, 2);
    }
    variance /= 20.0f;
    
    vibration_magnitude += sqrt(variance);
  }
  
  // If high vibration detected, enable more aggressive filtering
  if (vibration_magnitude > 2.0f) {
    advanced_motor.rpm_based_filtering_enabled = true;
    advanced_motor.auto_rpm_filter_tuning = true;
    
    // Increase filter Q factor for stronger notching
    advanced_motor.rpm_filter_q_factor = 30.0f;
  } else {
    advanced_motor.rpm_filter_q_factor = 15.0f;
  }
  
  last_vibration_check = millis();
}

void MotorControl::characterize_motor_response(int motor_num) {
  if (motor_num < 0 || motor_num >= 4) return;
  
  Serial.print("Characterizing motor ");
  Serial.print(motor_num + 1);
  Serial.println(" response...");
  
  // Perform step response test
  unsigned long start_time = millis();
  uint16_t baseline_rpm = get_motor_rpm(motor_num);
  
  // Apply step input
  int step_value = 1300; // 30% throttle step
  test_motor(motor_num, step_value);
  
  // Measure response time
  while (millis() - start_time < 2000) {
    uint16_t current_rpm = get_motor_rpm(motor_num);
    if (current_rpm > baseline_rpm + 500) {
      advanced_motor.motor_response_prediction[motor_num] = millis() - start_time;
      break;
    }
    delay(10);
  }
  
  // Return to idle
  test_motor(motor_num, 1000);
  
  Serial.print("Motor ");
  Serial.print(motor_num + 1);
  Serial.print(" response time: ");
  Serial.print(advanced_motor.motor_response_prediction[motor_num]);
  Serial.println(" ms");
}

#endif // MOTOR_CONTROL_H
