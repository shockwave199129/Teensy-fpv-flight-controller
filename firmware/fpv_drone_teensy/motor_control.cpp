#include "motor_control.h"
#include "Arduino.h"
#include "constants.h"
#include "esc_dma.h"
#include "dynamic_filtering.h"
#include "hal_timer_pwm.h"
#include "blackbox_logger.h"
#include "pid_controller.h"
#include <string.h>

// Delay helper now provided by HAL
using HalTimerPWM::delayNS;

bool MotorControl::check_esc_voltage_telemetry() {
  // Check if any ESC is providing voltage telemetry
  if (!supports_telemetry()) return false;
  
  for (int i = 0; i < 4; i++) {
    if (motor_voltage[i] > 0 && (millis() - motor_telemetry_last_update[i] < 1000)) {
      return true;
    }
  }
  return false;
}

void MotorControl::init() {
  init_with_protocol(ESC_PROTOCOL_PWM);
}

void MotorControl::init_with_protocol(EscProtocol protocol) {
  // This implementation is based on the partial file view and may be incomplete.
  esc_config.protocol = protocol;
  esc_config.telemetry_enabled = false;
  esc_config.bidirectional_enabled = false;
  esc_config.idle_throttle_percent = 0;
  esc_config.min_throttle_percent = 0;
  esc_config.max_throttle_percent = 100;
  esc_config.beep_on_startup = true;
  esc_config.auto_detect_firmware = true;  // enable auto-detection by default
  esc_config.firmware_type = ESC_FIRMWARE_UNKNOWN;
  memset(&esc_config.capabilities, 0, sizeof(esc_config.capabilities));
  
  // Default predictive motor control settings
  advanced_motor.predictive_control_enabled = false; // disabled until telemetry validated
  advanced_motor.thrust_lag_compensation = 0.0005f;
  for(int i=0;i<4;i++){
    advanced_motor.motor_response_prediction[i] = 0.0f;
    prev_rpm[i] = 0;
    esc_config.motor_direction[i] = 1;
    esc_config.motor_enabled[i] = true;
    motor_rpm[i] = 0;
    motor_temperature[i] = 0;
    motor_voltage[i] = 0;
    motor_current[i] = 0;
    motor_telemetry_last_update[i] = 0;
    telemetry_request[i] = false;
  }
  
  // Protocol-specific initialization
  // NOTE: This switch statement is likely incomplete due to partial file visibility.
  switch (protocol) {
    case ESC_PROTOCOL_PWM:
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
      motor1.attach(MOTOR1_PIN);
      motor2.attach(MOTOR2_PIN);
      motor3.attach(MOTOR3_PIN);
      motor4.attach(MOTOR4_PIN);
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_DSHOT1200:
      pinMode(MOTOR1_PIN, OUTPUT);
      pinMode(MOTOR2_PIN, OUTPUT);
      pinMode(MOTOR3_PIN, OUTPUT);
      pinMode(MOTOR4_PIN, OUTPUT);
      {
        const uint8_t pins[4] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};
        ESC_DMA::init(protocol, pins, 4);
      }
      break;
  }
  
  // Perform ESC firmware auto-detection for DShot protocols
  if (esc_config.auto_detect_firmware && esc_config.protocol >= ESC_PROTOCOL_DSHOT150) {
    EscFirmwareType detected = detect_esc_firmware();
    esc_config.firmware_type = detected;
    Serial.print("ESC firmware detected: ");
    switch (detected) {
      case ESC_FIRMWARE_BLHELI_S: Serial.println("BLHeli-S"); break;
      case ESC_FIRMWARE_BLHELI_32: Serial.println("BLHeli-32"); break;
      case ESC_FIRMWARE_BLUEJAY: Serial.println("BlueJay"); break;
      case ESC_FIRMWARE_AM32: Serial.println("AM32"); break;
      case ESC_FIRMWARE_ESCAPE32: Serial.println("ESCAPE32"); break;
      case ESC_FIRMWARE_GENERIC: Serial.println("Generic"); break;
      default: Serial.println("Unknown"); break;
    }

    // Populate capability hints based on firmware
    EscFirmwareCapabilities caps = {};
    switch (detected) {
      case ESC_FIRMWARE_BLHELI_S:
        caps.supports_rpm_telemetry = false; // BLHeli_S original does not
        caps.supports_bidirectional_dshot = false;
        caps.max_rpm = 100000; break;
      case ESC_FIRMWARE_BLUEJAY:
        caps.supports_rpm_telemetry = true;
        caps.supports_bidirectional_dshot = true;
        caps.supports_temperature_telemetry = false;
        caps.max_rpm = 200000;
        break;
      case ESC_FIRMWARE_BLHELI_32:
        caps.supports_rpm_telemetry = true;
        caps.supports_temperature_telemetry = true;
        caps.supports_voltage_telemetry = true;
        caps.supports_current_telemetry = true;
        caps.supports_bidirectional_dshot = true;
        caps.supports_variable_pwm_frequency = true;
        caps.supports_beacon = true;
        caps.supports_settings_via_dshot = true;
        caps.supports_firmware_update = true;
        caps.max_rpm = 250000;
        break;
      case ESC_FIRMWARE_AM32:
        caps.supports_rpm_telemetry = true;
        caps.supports_temperature_telemetry = true;
        caps.supports_bidirectional_dshot = true;
        caps.max_rpm = 300000; break;
      default:
        break;
    }
    esc_config.capabilities = caps;

    if (caps.supports_voltage_telemetry || caps.supports_current_telemetry || caps.supports_temperature_telemetry) {
      // Command number 13 per spec (may overlap) – send raw value 13
      for (int m = 0; m < 4; m++) {
        send_dshot_command_motor(m, (DshotCommand)13); // Extended telemetry enable
        delay(2);
      }
    }
  }
  
  motors_armed = false;
  calibration_mode = false;
  
  Serial.print("Motor control initialized with protocol: ");
  Serial.println(get_protocol_name());
  
  if (esc_config.beep_on_startup && supports_telemetry()) {
    delay(1000);
  }
}

// ============================ NEW IMPLEMENTATIONS ============================

static inline void safe_digital_write(uint8_t pin, bool val) {
  if (val)
    GPIO6_DR_SET = (1UL << pin);
  else
    GPIO6_DR_CLEAR = (1UL << pin);
}

void MotorControl::apply_motor_mixing(PIDOutput& pid, int base_throttle) {
  // Simple quad-X mixing
  motor_output.motor1 = base_throttle + pid.roll - pid.pitch + pid.yaw; // Front Right CW
  motor_output.motor2 = base_throttle - pid.roll - pid.pitch - pid.yaw; // Front Left CCW
  motor_output.motor3 = base_throttle - pid.roll + pid.pitch + pid.yaw; // Rear Left CW
  motor_output.motor4 = base_throttle + pid.roll + pid.pitch - pid.yaw; // Rear Right CCW

  // Constrain
  motor_output.motor1 = constrain(motor_output.motor1, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
  motor_output.motor2 = constrain(motor_output.motor2, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
  motor_output.motor3 = constrain(motor_output.motor3, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
  motor_output.motor4 = constrain(motor_output.motor4, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
}

int MotorControl::map_throttle_to_protocol(int pwm) {
  switch (esc_config.protocol) {
    case ESC_PROTOCOL_PWM:
    case ESC_PROTOCOL_ONESHOT125:
    case ESC_PROTOCOL_ONESHOT42:
    case ESC_PROTOCOL_MULTISHOT:
      return pwm; // direct µs
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_DSHOT1200:
      // map 1000-2000µs to 48-2047 range used by dshot
      return map(pwm, 1000, 2000, 48, 2047);
  }
  return 0;
}

void MotorControl::output_pwm(int motor, int value) {
  switch (motor) {
    case 0: motor1.writeMicroseconds(value); break;
    case 1: motor2.writeMicroseconds(value); break;
    case 2: motor3.writeMicroseconds(value); break;
    case 3: motor4.writeMicroseconds(value); break;
  }
}

void MotorControl::output_dshot(int motor, int value, bool telemetry) {
  uint16_t packet = prepare_dshot_packet(value, telemetry);
  send_dshot_packet(motor, packet);
}

uint16_t MotorControl::prepare_dshot_packet(uint16_t value, bool telemetry) {
  value = value << 1; // make room for telemetry bit
  if (telemetry)
    value |= 1;
  // checksum
  uint16_t csum = 0;
  uint16_t csum_data = value;
  for (int i = 0; i < 3; i++) {
    csum ^= csum_data;              // xor data by nibbles
    csum_data >>= 4;
  }
  csum &= 0x0F;
  return (value << 4) | csum;
}

void MotorControl::send_dshot_packet(int motor, uint16_t packet) {
  // Very simplistic blocking bit-bang implementation, sufficient for 150/300/600k.
  uint8_t pin = MOTOR1_PIN + motor; // contiguous pins 2-5
  uint32_t t0 = DSHOT600_BIT_TIME; // nanoseconds per bit at 600k
  if (esc_config.protocol == ESC_PROTOCOL_DSHOT300) t0 = DSHOT300_BIT_TIME;
  else if (esc_config.protocol == ESC_PROTOCOL_DSHOT150) t0 = DSHOT150_BIT_TIME;
  else if (esc_config.protocol == ESC_PROTOCOL_DSHOT1200) t0 = DSHOT1200_BIT_TIME;

  noInterrupts();
  for (int i = 15; i >= 0; i--) {
    bool bit = packet & (1 << i);
    digitalWrite(pin, HIGH);
    delayNS(bit ? t0 * DSHOT_BIT_1_HIGH_TIME_FACTOR : t0 * DSHOT_BIT_0_HIGH_TIME_FACTOR);
    digitalWrite(pin, LOW);
    delayNS(bit ? t0 * (1 - DSHOT_BIT_1_HIGH_TIME_FACTOR) : t0 * (1 - DSHOT_BIT_0_HIGH_TIME_FACTOR));
  }
  interrupts();
}

void MotorControl::update(PIDOutput& pid_output, bool armed_flag) {
  if (!motors_armed || !armed_flag) {
    // Keep motors at idle/stop
    if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150)
      send_dshot_command_all(DSHOT_CMD_MOTOR_STOP);
    else {
      output_pwm(0, ESC_MIN_PWM);
      output_pwm(1, ESC_MIN_PWM);
      output_pwm(2, ESC_MIN_PWM);
      output_pwm(3, ESC_MIN_PWM);
    }
    return;
  }

  // Compute throttle base
  int base_throttle = map_throttle_to_protocol(pid_output.throttle + MOTOR_IDLE_THROTTLE);
  apply_motor_mixing(pid_output, base_throttle);

  // Optional predictive control optimisation
  // Send to ESCs
  if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150) {
    uint16_t vals[4] = { (uint16_t)motor_output.motor1, (uint16_t)motor_output.motor2,
                         (uint16_t)motor_output.motor3, (uint16_t)motor_output.motor4 };
    ESC_DMA::sendThrottle(vals,false);
  } else {
    output_pwm(0, motor_output.motor1);
    output_pwm(1, motor_output.motor2);
    output_pwm(2, motor_output.motor3);
    output_pwm(3, motor_output.motor4);
  }

  // --- Blackbox logging ---
  extern BlackboxLogger blackbox;
  if(blackbox.isEnabled()){
    extern PIDController pid_controller; // assumed global
    extern RcData rc_data_global;        // assumed global
    extern SensorData sensor_data_global;
    blackbox.logFrame(sensor_data_global, rc_data_global, pid_controller.get_outputs(), motor_output);
  }

  // Request telemetry for next frame if enabled
  ESC_DMA::requestTelemetry(esc_config.telemetry_enabled);

  if (esc_config.telemetry_enabled) {
    extern DynamicFilteringSystem dynamic_filtering; // declared in main .ino
    for (uint8_t m = 0; m < 4; m++) {
      uint16_t erpm=0, volt=0, curr=0;
      uint8_t temp=0;
      if (ESC_DMA::getTelemetry(m, erpm, volt, temp, curr)) {
        if(erpm) motor_rpm[m] = erpm;
        if(volt) motor_voltage[m] = volt;
        if(temp) motor_temperature[m] = temp;
        if(curr) motor_current[m] = curr;
        motor_telemetry_last_update[m] = millis();
        // nothing else; rpm array will be fed later
      }
    }

    // Provide RPM data to dynamic filter for feed-forward notch tuning
    dynamic_filtering.feed_motor_rpm(motor_rpm);
  }
}

void MotorControl::arm() {
  motors_armed = true;
  Serial.println("MotorControl: Motors armed");
}

void MotorControl::disarm() {
  motors_armed = false;
  // Ensure motors stop
  emergency_stop();
  Serial.println("MotorControl: Motors disarmed");
}

void MotorControl::emergency_stop() {
  if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150) {
    send_dshot_command_all(DSHOT_CMD_MOTOR_STOP);
  } else {
    output_pwm(0, ESC_MIN_PWM);
    output_pwm(1, ESC_MIN_PWM);
    output_pwm(2, ESC_MIN_PWM);
    output_pwm(3, ESC_MIN_PWM);
  }
}

void MotorControl::send_dshot_command_all(DshotCommand cmd) {
  for (int m = 0; m < 4; m++) send_dshot_command_motor(m, cmd);
}

void MotorControl::send_dshot_command_motor(int motor, DshotCommand cmd) {
  output_dshot(motor, (uint16_t)cmd | 0x700, false); // commands are 11-bit with 0x7 prefix
}

void MotorControl::set_esc_protocol(EscProtocol protocol) {
  init_with_protocol(protocol);
}

void MotorControl::configure_esc(const EscConfig& cfg) {
  esc_config = cfg;
  init_with_protocol(cfg.protocol);
}

const char* MotorControl::get_protocol_name() {
  switch (esc_config.protocol) {
    case ESC_PROTOCOL_PWM: return "PWM";
    case ESC_PROTOCOL_ONESHOT125: return "ONESHOT125";
    case ESC_PROTOCOL_ONESHOT42: return "ONESHOT42";
    case ESC_PROTOCOL_MULTISHOT: return "MULTISHOT";
    case ESC_PROTOCOL_DSHOT150: return "DSHOT150";
    case ESC_PROTOCOL_DSHOT300: return "DSHOT300";
    case ESC_PROTOCOL_DSHOT600: return "DSHOT600";
    case ESC_PROTOCOL_DSHOT1200: return "DSHOT1200";
  }
  return "UNKNOWN";
}

// --- Additional helper stubs to satisfy linker ---
void MotorControl::calibrate_escs() {
  // Simple calibration routine: send max then min throttle for analog protocols.
  if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150) {
    // DShot ESCs typically auto-calibrate; send max then min command sequence
    send_dshot_command_all(DSHOT_CMD_ESC_CALIBRATION);
  } else {
    Serial.println("ESC Calibration: Sending max throttle");
    output_pwm(0, ESC_MAX_PWM);
    output_pwm(1, ESC_MAX_PWM);
    output_pwm(2, ESC_MAX_PWM);
    output_pwm(3, ESC_MAX_PWM);
    delay(2000);
    Serial.println("ESC Calibration: Sending min throttle");
    output_pwm(0, ESC_MIN_PWM);
    output_pwm(1, ESC_MIN_PWM);
    output_pwm(2, ESC_MIN_PWM);
    output_pwm(3, ESC_MIN_PWM);
    delay(2000);
    Serial.println("ESC Calibration complete");
  }
}

void MotorControl::set_motor_direction(int motor, int direction) {
  if (motor < 0 || motor > 3) return;
  direction = (direction >= 0) ? 1 : -1;
  esc_config.motor_direction[motor] = direction;

  if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150) {
    DshotCommand cmd = (direction == 1) ? DSHOT_CMD_SPIN_DIRECTION_NORMAL : DSHOT_CMD_SPIN_DIRECTION_REVERSED;
    send_dshot_command_motor(motor, cmd);
    delay(50);
    // Optionally save settings after all motors configured elsewhere
  } else {
    // For analog protocols direction is hardware-set; we just update state
  }
}

void MotorControl::reverse_motor_direction(int motor) {
  if (motor < 0 || motor > 3) return;
  int new_dir = -esc_config.motor_direction[motor];
  set_motor_direction(motor, new_dir);
}

// ============================================================================
// ESC firmware detection logic
// ============================================================================

EscFirmwareType MotorControl::detect_esc_firmware() {
  // Attempts to detect firmware type on connected ESCs by issuing the ESC_INFO
  // command (DShot command 6) and parsing the first two telemetry words.
  const uint8_t MAX_ATTEMPTS = 20;  // attempts per motor
  uint8_t typeCounts[ESC_FIRMWARE_GENERIC + 1] = {0};

  for (uint8_t motor = 0; motor < 4; motor++) {
    // Enable telemetry during detection
    ESC_DMA::requestTelemetry(true);
    // Send ESC_INFO command
    send_dshot_command_motor(motor, DSHOT_CMD_ESC_INFO);
    delay(5);

    uint16_t throttleVals[4] = {0, 0, 0, 0};
    bool gotFirst = false, gotSecond = false;
    uint16_t word1 = 0, word2 = 0;

    // Request telemetry for subsequent frames
    for (uint8_t attempt = 0; attempt < MAX_ATTEMPTS && !(gotFirst && gotSecond); attempt++) {
      ESC_DMA::sendThrottle(throttleVals, true);
      delay(2);
      uint16_t raw;
      if (ESC_DMA::getRawPacket(motor, raw)) {
        if (!gotFirst) {
          word1 = raw;
          gotFirst = true;
        } else if (!gotSecond) {
          word2 = raw;
          gotSecond = true;
        }
      }
    }
    // Stop requesting telemetry
    ESC_DMA::requestTelemetry(false);

    if (!(gotFirst && gotSecond)) continue; // Unable to read

    uint32_t info24 = ((uint32_t)(word2 & 0x0FFF) << 12) | (word1 & 0x0FFF);
    char c1 = (char)(info24 & 0xFF);
    char c2 = (char)((info24 >> 8) & 0xFF);

    EscFirmwareType type = ESC_FIRMWARE_UNKNOWN;
    if (c1 == 'B' && c2 == 'J') {
      type = ESC_FIRMWARE_BLUEJAY;
    } else if (c1 == '3' && c2 == '2') {
      type = ESC_FIRMWARE_BLHELI_32;
    } else if (c1 == 'B' && c2 == 'L') {
      type = ESC_FIRMWARE_BLHELI_S;
    } else if (c1 == 'A' && c2 == 'M') {
      type = ESC_FIRMWARE_AM32;
    } else {
      type = ESC_FIRMWARE_GENERIC;
    }

    if (type <= ESC_FIRMWARE_GENERIC) {
      typeCounts[type]++;
    }

    Serial.print("Motor "); Serial.print(motor); Serial.print(" ESC_INFO bytes: 0x");
    Serial.print((uint32_t)info24, HEX);
    Serial.print(" => firmware type: ");
    switch (type) {
      case ESC_FIRMWARE_BLHELI_S: Serial.println("BLHeli-S"); break;
      case ESC_FIRMWARE_BLHELI_32: Serial.println("BLHeli-32"); break;
      case ESC_FIRMWARE_BLUEJAY: Serial.println("BlueJay"); break;
      case ESC_FIRMWARE_AM32: Serial.println("AM32"); break;
      default: Serial.println("Generic"); break;
    }
  }

  // Select firmware type with highest count (majority voting)
  EscFirmwareType best = ESC_FIRMWARE_UNKNOWN;
  uint8_t bestCount = 0;
  for (uint8_t t = 0; t <= ESC_FIRMWARE_GENERIC; t++) {
    if (typeCounts[t] > bestCount) {
      bestCount = typeCounts[t];
      best = (EscFirmwareType)t;
    }
  }
  return best;
}

void MotorControl::apply_predictive_motor_control() {
  if(!advanced_motor.predictive_control_enabled) return;
  // Determine max RPM capability
  uint32_t max_rpm = esc_config.capabilities.max_rpm ? esc_config.capabilities.max_rpm : 200000UL; // fallback
  float gain = advanced_motor.thrust_lag_compensation;
  if(gain <= 0.0f) gain = 0.001f;

  auto adjust_motor = [&](int idx, int &cmd){
    // Only adjust if we have fresh telemetry (<100ms)
    if(millis() - motor_telemetry_last_update[idx] > 100) return;
    float desired_rpm = ((float)cmd / 2047.0f) * (float)max_rpm;
    float error = desired_rpm - motor_rpm[idx];
    int correction = (int)(error * gain);
    cmd += correction;
    // constrain cmd depending on protocol
    if(cmd < 48) cmd = 48;
    if(cmd > 2047) cmd = 2047;
  };

  adjust_motor(0, motor_output.motor1);
  adjust_motor(1, motor_output.motor2);
  adjust_motor(2, motor_output.motor3);
  adjust_motor(3, motor_output.motor4);
}

// ============================================================================ 