#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include "led_control.h"
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
  unsigned long motor_telemetry_last_update[4];
  
  // For predictive control
  uint16_t prev_rpm[4];
  
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
  
  // Firmware detection helper
  EscFirmwareType detect_esc_firmware();
  
  // Motor characterization methods
  void characterize_motor_response(int motor_num);
  void calibrate_motor_kv_values();
  void measure_motor_resistance();
  
  // Missing helper function
  void output_value(int motor, int value);
  
  // Private member variables
  int rpm_index = 0;  // For RPM filtering
  
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
  bool check_esc_voltage_telemetry();
  const char* get_protocol_name();
  
  // DShot telemetry reading and RPM feedback
  void enable_dshot_telemetry(bool enable);
  bool read_motor_telemetry(int motor_num, MotorTelemetry* telemetry);
  void apply_rpm_notch_filtering();
  
  // Motor performance monitoring
  MotorTelemetry get_motor_telemetry(int motor_num);
  void monitor_motor_health();
  float get_motor_efficiency(int motor_num);
};

#endif // MOTOR_CONTROL_H
