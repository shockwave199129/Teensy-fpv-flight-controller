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

// Map logical motor index to hardware pin. Extend as needed for larger frames.
static const uint8_t MOTOR_PINS[MAX_MOTORS] = {
  MOTOR1_PIN,
  MOTOR2_PIN,
  MOTOR3_PIN,
  MOTOR4_PIN,
#if MAX_MOTORS > 4
  /* default extra assignments – adjust for your wiring */
  6, 7, 8, 9, 10, 11, 12, 13
#endif
};

bool MotorControl::check_esc_voltage_telemetry() {
  // Check if any ESC is providing voltage telemetry
  if (!supports_telemetry()) return false;
  
  for (int i = 0; i < MOTOR_COUNT; i++) {
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
  for(int i=0;i<MOTOR_COUNT;i++){
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
      for(int m=0;m<MOTOR_COUNT;m++){
          motors[m].attach(MOTOR_PINS[m]);
          motors[m].writeMicroseconds(ESC_MIN_PWM);
      }
      break;
    case ESC_PROTOCOL_ONESHOT125:
    case ESC_PROTOCOL_ONESHOT42:
    case ESC_PROTOCOL_MULTISHOT:
      for(int m=0;m<MOTOR_COUNT;m++){
          motors[m].attach(MOTOR_PINS[m]);
      }
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_DSHOT1200:
      for(int m=0;m<MOTOR_COUNT;m++){
          pinMode(MOTOR_PINS[m], OUTPUT);
      }
      ESC_DMA::init(protocol, MOTOR_PINS, MOTOR_COUNT);
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
      for (int m = 0; m < MOTOR_COUNT; m++) {
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
  // Mixing matrices per frame type.
  // Coefficients: [ rollCoef, pitchCoef, yawCoef ] per motor; collective term added later.

  static const float MIX_X4[4][3] = {
      {  1, -1, -1 },   // Front Right CW
      { -1, -1,  1 },   // Front Left  CCW
      { -1,  1, -1 },   // Rear  Left  CW
      {  1,  1,  1 }    // Rear  Right CCW
  };

  // Y-4 (twin front motors + rear)
  static const float MIX_Y4[4][3] = {
      {  0, -1, -1 },   // Front Left  (CCW front)
      {  0, -1,  1 },   // Front Right (CW  front)
      { -1,  1, -1 },   // Rear Left   (CW)
      {  1,  1,  1 }    // Rear Right  (CCW)
  };

  // X6 (flat hex, starting front and going CW)
  static const float MIX_X6[6][3] = {
      {  0, -1,  1 },   // 0  front
      { -1, -0.5, -1 }, // 60° left front
      { -1,  0.5,  1 }, // 120° left rear
      {  0,  1, -1 },   // 180° rear
      {  1,  0.5,  1 }, // 240° right rear
      {  1, -0.5, -1 }  // 300° right front
  };

  // Coaxial X8: use X4 pattern duplicated (top first 4, bottom reversed yaw)
  static float MIX_X8[8][3];
  static bool mixX8Init=false;
  if(!mixX8Init){
      for(int i=0;i<4;i++){
          MIX_X8[i][0]=MIX_X4[i][0];
          MIX_X8[i][1]=MIX_X4[i][1];
          MIX_X8[i][2]=MIX_X4[i][2];
          // bottom motors (i+4) roll & pitch same, yaw reversed
          MIX_X8[i+4][0]=MIX_X4[i][0];
          MIX_X8[i+4][1]=MIX_X4[i][1];
          MIX_X8[i+4][2]=-MIX_X4[i][2];
      }
      mixX8Init=true;
  }

  // X12: duplicate X6 for bottom layer with yaw reversed.
  static float MIX_X12[12][3];
  static bool mixX12Init=false;
  if(!mixX12Init){
      for(int i=0;i<6;i++){
          for(int j=0;j<3;j++) MIX_X12[i][j]=MIX_X6[i][j];
          MIX_X12[i+6][0]=MIX_X6[i][0];
          MIX_X12[i+6][1]=MIX_X6[i][1];
          MIX_X12[i+6][2]=-MIX_X6[i][2];
      }
      mixX12Init=true;
  }

  const float (*mix)[3] = nullptr;
  switch(FRAME_TYPE){
      case FRAME_X4: mix = MIX_X4; break;
      case FRAME_Y4: mix = MIX_Y4; break;
      case FRAME_X6: mix = MIX_X6; break;
      case FRAME_X8: mix = MIX_X8; break;
      case FRAME_X12: mix = MIX_X12; break;
      default: mix = MIX_X4; break;
  }

  // Compute outputs into local array then populate legacy struct for first 4 motors
  int pwm[MAX_MOTORS];
  for(int m=0;m<MOTOR_COUNT;m++){
      float out = base_throttle
                  + mix[m][0]*pid.roll
                  + mix[m][1]*pid.pitch
                  + mix[m][2]*pid.yaw;
      pwm[m] = constrain((int)out, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
  }

  // Map to struct (first 4) for backward compatibility
  if(MOTOR_COUNT>=4){
      motor_output.motor1 = pwm[0];
      motor_output.motor2 = pwm[1];
      motor_output.motor3 = pwm[2];
      motor_output.motor4 = pwm[3];
  }

  // Send to hardware (handled later in update function using pwm array). For now store into prev arrays
  for(int m=0;m<MOTOR_COUNT;m++){
      motor_rpm[m] = pwm[m]; // placeholder meaning of rpm array misuse; real rpm updated elsewhere
  }
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
  if(motor<0 || motor>=MOTOR_COUNT) return;
  motors[motor].writeMicroseconds(value);
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
  if(motor<0 || motor>=MOTOR_COUNT) return;
  uint8_t pin = MOTOR_PINS[motor];
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
      for(int m=0;m<MOTOR_COUNT;m++){
          output_pwm(m, ESC_MIN_PWM);
      }
    }
    return;
  }

  // Compute throttle base
  int base_throttle = map_throttle_to_protocol(pid_output.throttle + MOTOR_IDLE_THROTTLE);
  apply_motor_mixing(pid_output, base_throttle);

  // Optional predictive control optimisation
  // Send to ESCs
  if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150) {
    uint16_t vals[MAX_MOTORS] = {0};
    // Map outputs – legacy struct holds first 4; beyond that use motor_rpm placeholder we stored earlier
    if(MOTOR_COUNT>=1) vals[0] = motor_output.motor1;
    if(MOTOR_COUNT>=2) vals[1] = motor_output.motor2;
    if(MOTOR_COUNT>=3) vals[2] = motor_output.motor3;
    if(MOTOR_COUNT>=4) vals[3] = motor_output.motor4;
    for(int m=4;m<MOTOR_COUNT;m++){
        vals[m] = motor_rpm[m]; // we stored PWM values in motor_rpm earlier
    }
    ESC_DMA::sendThrottle(vals,false);
  } else {
    for(int m=0;m<MOTOR_COUNT && m<4;m++){
        int val = (m==0)?motor_output.motor1:(m==1)?motor_output.motor2:(m==2)?motor_output.motor3:motor_output.motor4;
        output_pwm(m,val);
    }
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
    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
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
    for(int m=0;m<MOTOR_COUNT;m++){
        output_pwm(m, ESC_MIN_PWM);
    }
  }
}

void MotorControl::send_dshot_command_all(DshotCommand cmd) {
  for (int m = 0; m < MOTOR_COUNT; m++) send_dshot_command_motor(m, cmd);
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
    for(int m=0;m<MOTOR_COUNT;m++){
        output_pwm(m, ESC_MAX_PWM);
    }
    delay(2000);
    Serial.println("ESC Calibration: Sending min throttle");
    for(int m=0;m<MOTOR_COUNT;m++){
        output_pwm(m, ESC_MIN_PWM);
    }
    delay(2000);
    Serial.println("ESC Calibration complete");
  }
}

void MotorControl::set_motor_direction(int motor, int direction) {
  if (motor < 0 || motor > MOTOR_COUNT) return;
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
  if (motor < 0 || motor > MOTOR_COUNT) return;
  int new_dir = -esc_config.motor_direction[motor];
  set_motor_direction(motor, new_dir);
}

// ===================== Direction Utilities =====================
void MotorControl::set_default_motor_directions() {
    // Expected direction tables: 1 = NORMAL/CW, -1 = REVERSE/CCW
    static const int8_t X4_DIR[4]  = { 1, -1, 1, -1 };  // FR, FL, RL, RR (CW/CCW alternated)
    static const int8_t Y4_DIR[4]  = { -1, 1, 1, -1 };  // Example Y4 (twin front counter)
    static const int8_t X6_DIR[6]  = { -1, 1, -1, 1, -1, 1 }; // alternating around hex
    static const int8_t X8_DIR[8]  = { 1,-1,1,-1, -1,1,-1,1 }; // top layer same as X4, bottom reversed
    static const int8_t X12_DIR[12]= { -1,1,-1,1,-1,1, 1,-1,1,-1,1,-1 }; // similar pattern

    const int8_t *tbl=nullptr;
    switch(FRAME_TYPE){
        case FRAME_X4:  tbl=X4_DIR; break;
        case FRAME_Y4:  tbl=Y4_DIR; break;
        case FRAME_X6:  tbl=X6_DIR; break;
        case FRAME_X8:  tbl=X8_DIR; break;
        case FRAME_X12: tbl=X12_DIR; break;
        default: tbl=X4_DIR; break;
    }
    for(int m=0;m<MOTOR_COUNT;m++){
        esc_config.motor_direction[m] = (tbl)? tbl[m]:1;
    }
}

bool MotorControl::validate_motor_directions(bool verbose){
    const int8_t *expected=nullptr;
    static const int8_t X4_DIR[4]  = { 1, -1, 1, -1 };
    static const int8_t Y4_DIR[4]  = { -1, 1, 1, -1 };
    static const int8_t X6_DIR[6]  = { -1, 1, -1, 1, -1, 1 };
    static const int8_t X8_DIR[8]  = { 1,-1,1,-1, -1,1,-1,1 };
    static const int8_t X12_DIR[12]= { -1,1,-1,1,-1,1, 1,-1,1,-1,1,-1 };
    switch(FRAME_TYPE){
        case FRAME_X4: expected=X4_DIR; break;
        case FRAME_Y4: expected=Y4_DIR; break;
        case FRAME_X6: expected=X6_DIR; break;
        case FRAME_X8: expected=X8_DIR; break;
        case FRAME_X12: expected=X12_DIR; break;
        default: expected=X4_DIR; break;
    }
    bool ok=true;
    for(int m=0;m<MOTOR_COUNT;m++){
        if(esc_config.motor_direction[m] != expected[m]){
            ok=false;
            if(verbose){
               Serial.print("Motor "); Serial.print(m+1);
               Serial.print(" direction mismatch. Expected ");
               Serial.print(expected[m]==1?"NORMAL(CW)":"REVERSE(CCW)");
               Serial.print(" but is ");
               Serial.println(esc_config.motor_direction[m]==1?"NORMAL(CW)":"REVERSE(CCW)");
            }
        }
    }
    if(verbose){
        Serial.println(ok?"All motor directions valid ✅":"Motor direction errors detected ❌");
    }
    return ok;
}

// ============================================================================
// ESC firmware detection logic
// ============================================================================

EscFirmwareType MotorControl::detect_esc_firmware() {
  // Attempts to detect firmware type on connected ESCs by issuing the ESC_INFO
  // command (DShot command 6) and parsing the first two telemetry words.
  const uint8_t MAX_ATTEMPTS = 20;  // attempts per motor
  uint8_t typeCounts[ESC_FIRMWARE_GENERIC + 1] = {0};

  for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
    // Enable telemetry during detection
    ESC_DMA::requestTelemetry(true);
    // Send ESC_INFO command
    send_dshot_command_motor(motor, DSHOT_CMD_ESC_INFO);
    delay(5);

    uint16_t throttleVals[MOTOR_COUNT] = {0};
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