#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"

// Enhanced cascaded PID structure
struct CascadedPIDGains {
  // Rate loop (inner loop) - fast response
  PIDGains rate_roll;
  PIDGains rate_pitch; 
  PIDGains rate_yaw;
  
  // Angle loop (outer loop) - smooth response
  PIDGains angle_roll;
  PIDGains angle_pitch;
  
  // Altitude control
  PIDGains altitude_rate;  // Vertical velocity
  PIDGains altitude_pos;   // Altitude position
};

// Enhanced PID gains structure
struct EnhancedPIDGains {
  float kp, ki, kd;
  float integral;
  float last_error;
  float last_derivative;
  float last_input;  // For derivative on measurement
  float output_min, output_max;
  
  // Advanced features
  bool derivative_on_measurement;  // Prevents derivative kick
  bool proportional_on_measurement; // PonM for smoother response
  float integral_limit;            // Anti-windup limit
  float derivative_filter_alpha;   // Low-pass filter for D-term
  unsigned long last_update_time;
};

class PIDController {
private:
  CascadedPIDGains cascaded_gains;
  PIDOutput rate_output;    // Rate loop output
  PIDOutput angle_output;   // Angle loop output
  PIDOutput final_output;   // Final mixed output
  
  // Enhanced PID calculation with advanced features
  float enhanced_pid_calculate(EnhancedPIDGains* gains, float setpoint, float measured_value, float dt);
  
  // Cascaded control functions
  void update_angle_loop(SensorData& sensor_data, RcData& rc_data, float dt);
  void update_rate_loop(SensorData& sensor_data, float dt);
  void apply_motor_saturation_compensation();
  
  // Filtering and smoothing
  float apply_derivative_filter(float derivative, float alpha);
  void reset_pid_state(EnhancedPIDGains* gains);
  
public:
  void init();
  void update(SensorData& sensor_data, RcData& rc_data, FlightState& flight_state);
  PIDOutput get_outputs();
  
  void set_rate_pid_gains(int axis, float kp, float ki, float kd);
  void set_angle_pid_gains(int axis, float kp, float ki, float kd);
  void set_advanced_pid_features(int axis, bool derivative_on_measurement, bool proportional_on_measurement);
  void reset_pid_integrals();
  
  // Auto-tuning helpers
  void enable_auto_tune(int axis);
  void process_auto_tune_step();
};

void PIDController::init() {
  // Initialize rate loop PIDs (inner loop - fast response)
  cascaded_gains.rate_roll = {
    .kp = DEFAULT_RATE_ROLL_KP,
    .ki = DEFAULT_RATE_ROLL_KI, 
    .kd = DEFAULT_RATE_ROLL_KD,
    .integral = 0,
    .last_error = 0,
    .last_derivative = 0,
    .last_input = 0,
    .output_min = -500,
    .output_max = 500,
    .derivative_on_measurement = true,
    .proportional_on_measurement = false,
    .integral_limit = 100,
    .derivative_filter_alpha = 0.1,
    .last_update_time = 0
  };
  
  cascaded_gains.rate_pitch = cascaded_gains.rate_roll;
  cascaded_gains.rate_pitch.kp = DEFAULT_RATE_PITCH_KP;
  cascaded_gains.rate_pitch.ki = DEFAULT_RATE_PITCH_KI;
  cascaded_gains.rate_pitch.kd = DEFAULT_RATE_PITCH_KD;
  
  cascaded_gains.rate_yaw = {
    .kp = DEFAULT_RATE_YAW_KP,
    .ki = DEFAULT_RATE_YAW_KI,
    .kd = DEFAULT_RATE_YAW_KD,
    .integral = 0,
    .last_error = 0,
    .last_derivative = 0,
    .last_input = 0,
    .output_min = -500,
    .output_max = 500,
    .derivative_on_measurement = true,
    .proportional_on_measurement = false,
    .integral_limit = 50,  // Lower limit for yaw
    .derivative_filter_alpha = 0.15,
    .last_update_time = 0
  };
  
  // Initialize angle loop PIDs (outer loop - smooth response)
  cascaded_gains.angle_roll = {
    .kp = DEFAULT_ANGLE_ROLL_KP,
    .ki = DEFAULT_ANGLE_ROLL_KI,
    .kd = DEFAULT_ANGLE_ROLL_KD,
    .integral = 0,
    .last_error = 0,
    .last_derivative = 0,
    .last_input = 0,
    .output_min = -200,  // Rate setpoint limits (deg/s)
    .output_max = 200,
    .derivative_on_measurement = true,
    .proportional_on_measurement = true,  // PonM for angle control
    .integral_limit = 50,
    .derivative_filter_alpha = 0.2,
    .last_update_time = 0
  };
  
  cascaded_gains.angle_pitch = cascaded_gains.angle_roll;
  cascaded_gains.angle_pitch.kp = DEFAULT_ANGLE_PITCH_KP;
  cascaded_gains.angle_pitch.ki = DEFAULT_ANGLE_PITCH_KI;
  cascaded_gains.angle_pitch.kd = DEFAULT_ANGLE_PITCH_KD;
  
  // Initialize altitude PIDs
  cascaded_gains.altitude_rate = {
    .kp = DEFAULT_ALT_RATE_KP,
    .ki = DEFAULT_ALT_RATE_KI,
    .kd = DEFAULT_ALT_RATE_KD,
    .integral = 0,
    .last_error = 0,
    .last_derivative = 0,
    .last_input = 0,
    .output_min = -500,
    .output_max = 500,
    .derivative_on_measurement = true,
    .proportional_on_measurement = false,
    .integral_limit = 200,
    .derivative_filter_alpha = 0.1,
    .last_update_time = 0
  };
  
  cascaded_gains.altitude_pos = {
    .kp = DEFAULT_ALT_POS_KP,
    .ki = DEFAULT_ALT_POS_KI,
    .kd = DEFAULT_ALT_POS_KD,
    .integral = 0,
    .last_error = 0,
    .last_derivative = 0,
    .last_input = 0,
    .output_min = -5.0,  // Vertical velocity limits (m/s)
    .output_max = 5.0,
    .derivative_on_measurement = true,
    .proportional_on_measurement = true,
    .integral_limit = 2.0,
    .derivative_filter_alpha = 0.3,
    .last_update_time = 0
  };
  
  Serial.println("Enhanced Cascaded PID Controller initialized");
}

void PIDController::update(SensorData& sensor_data, RcData& rc_data, FlightState& flight_state) {
  static unsigned long last_update = 0;
  unsigned long current_time = micros();
  float dt = (current_time - last_update) / 1000000.0;
  
  if (dt < 0.0001) return; // Minimum update rate (10kHz max)
  if (dt > 0.01) dt = 0.01; // Maximum dt to prevent jumps
  
  last_update = current_time;
  
  switch (flight_state.flight_mode) {
    case FLIGHT_MODE_MANUAL:
      // Direct rate control - bypass angle loop
      update_rate_loop_direct(sensor_data, rc_data, dt);
      break;
      
    case FLIGHT_MODE_STABILIZE:
    case FLIGHT_MODE_ALTITUDE_HOLD:
    case FLIGHT_MODE_POSITION_HOLD:
      // Cascaded control - angle loop then rate loop
      update_angle_loop(sensor_data, rc_data, dt);
      update_rate_loop(sensor_data, dt);
      break;
      
    default:
      update_angle_loop(sensor_data, rc_data, dt);
      update_rate_loop(sensor_data, dt);
      break;
  }
  
  // Apply motor saturation compensation
  apply_motor_saturation_compensation();
}

void PIDController::update_angle_loop(SensorData& sensor_data, RcData& rc_data, float dt) {
  // Calculate angle setpoints from RC input
  float roll_setpoint = map(rc_data.roll, 1000, 2000, -45, 45);    // degrees
  float pitch_setpoint = map(rc_data.pitch, 1000, 2000, -45, 45);  // degrees
  
  // Get adaptive tuning from sensor redundancy system
  extern SensorRedundancySystem sensor_redundancy;
  FlightTuning adaptive_tuning = sensor_redundancy.get_adaptive_tuning();
  
  // Apply adaptive gains
  float roll_gain_modifier = adaptive_tuning.stability_gain_multiplier;
  float pitch_gain_modifier = adaptive_tuning.stability_gain_multiplier;
  
  // Angle loop outputs rate setpoints with adaptive tuning
  angle_output.roll = enhanced_pid_calculate(&cascaded_gains.angle_roll, 
                                           roll_setpoint, sensor_data.roll, dt) * roll_gain_modifier;
  angle_output.pitch = enhanced_pid_calculate(&cascaded_gains.angle_pitch, 
                                            pitch_setpoint, sensor_data.pitch, dt) * pitch_gain_modifier;
  
  // Yaw is always rate control
  angle_output.yaw = map(rc_data.yaw, 1000, 2000, -200, 200);  // deg/s
}

void PIDController::update_rate_loop(SensorData& sensor_data, float dt) {
  // Rate loop uses angle loop outputs as setpoints
  final_output.roll = enhanced_pid_calculate(&cascaded_gains.rate_roll,
                                           angle_output.roll, sensor_data.imu.gyro_x, dt);
  final_output.pitch = enhanced_pid_calculate(&cascaded_gains.rate_pitch,
                                            angle_output.pitch, sensor_data.imu.gyro_y, dt);
  final_output.yaw = enhanced_pid_calculate(&cascaded_gains.rate_yaw,
                                          angle_output.yaw, sensor_data.imu.gyro_z, dt);
}

void PIDController::update_rate_loop_direct(SensorData& sensor_data, RcData& rc_data, float dt) {
  // Direct rate control for manual mode
  float roll_rate_setpoint = map(rc_data.roll, 1000, 2000, -360, 360);   // deg/s
  float pitch_rate_setpoint = map(rc_data.pitch, 1000, 2000, -360, 360); // deg/s
  float yaw_rate_setpoint = map(rc_data.yaw, 1000, 2000, -200, 200);     // deg/s
  
  final_output.roll = enhanced_pid_calculate(&cascaded_gains.rate_roll,
                                           roll_rate_setpoint, sensor_data.imu.gyro_x, dt);
  final_output.pitch = enhanced_pid_calculate(&cascaded_gains.rate_pitch,
                                            pitch_rate_setpoint, sensor_data.imu.gyro_y, dt);
  final_output.yaw = enhanced_pid_calculate(&cascaded_gains.rate_yaw,
                                          yaw_rate_setpoint, sensor_data.imu.gyro_z, dt);
}

float PIDController::enhanced_pid_calculate(EnhancedPIDGains* gains, float setpoint, float measured_value, float dt) {
  float error = setpoint - measured_value;
  float proportional_input = gains->proportional_on_measurement ? measured_value : error;
  
  // Proportional term (with PonM option)
  float p_term = gains->kp * (gains->proportional_on_measurement ? -proportional_input : error);
  
  // Integral term with advanced anti-windup
  gains->integral += error * dt;
  
  // Conditional integration (anti-windup)
  if (gains->integral > gains->integral_limit) gains->integral = gains->integral_limit;
  if (gains->integral < -gains->integral_limit) gains->integral = -gains->integral_limit;
  
  float i_term = gains->ki * gains->integral;
  
  // Derivative term (with derivative on measurement option)
  float derivative;
  if (gains->derivative_on_measurement) {
    derivative = -(measured_value - gains->last_input) / dt;  // Derivative on measurement
    gains->last_input = measured_value;
  } else {
    derivative = (error - gains->last_error) / dt;  // Derivative on error
    gains->last_error = error;
  }
  
  // Apply derivative filter to reduce noise
  derivative = apply_derivative_filter(derivative, gains->derivative_filter_alpha);
  gains->last_derivative = derivative;
  
  float d_term = gains->kd * derivative;
  
  // Calculate final output
  float output = p_term + i_term + d_term;
  
  // Apply output limits
  output = constrain(output, gains->output_min, gains->output_max);
  
  return output;
}

float PIDController::apply_derivative_filter(float derivative, float alpha) {
  // Simple low-pass filter for derivative term
  static float filtered_derivative[4] = {0, 0, 0, 0}; // For roll, pitch, yaw, alt
  static int filter_index = 0;
  
  filtered_derivative[filter_index] = alpha * derivative + (1.0 - alpha) * filtered_derivative[filter_index];
  return filtered_derivative[filter_index];
}

void PIDController::apply_motor_saturation_compensation() {
  // Check if any motor would saturate and compensate
  float max_output = max(max(abs(final_output.roll), abs(final_output.pitch)), 
                        max(abs(final_output.yaw), abs(final_output.throttle)));
  
  if (max_output > 500) {
    float scale_factor = 500.0 / max_output;
    final_output.roll *= scale_factor;
    final_output.pitch *= scale_factor;
    final_output.yaw *= scale_factor;
  }
}

PIDOutput PIDController::get_outputs() {
  return final_output;
}

void PIDController::set_rate_pid_gains(int axis, float kp, float ki, float kd) {
  EnhancedPIDGains* gains = nullptr;
  
  switch (axis) {
    case 0: gains = &cascaded_gains.rate_roll; break;
    case 1: gains = &cascaded_gains.rate_pitch; break;
    case 2: gains = &cascaded_gains.rate_yaw; break;
    default: return;
  }
  
  gains->kp = kp;
  gains->ki = ki;
  gains->kd = kd;
  
  Serial.print("Rate PID gains updated for axis ");
  Serial.print(axis);
  Serial.print(": KP=");
  Serial.print(kp, 3);
  Serial.print(", KI=");
  Serial.print(ki, 3);
  Serial.print(", KD=");
  Serial.println(kd, 3);
}

void PIDController::set_angle_pid_gains(int axis, float kp, float ki, float kd) {
  EnhancedPIDGains* gains = nullptr;
  
  switch (axis) {
    case 0: gains = &cascaded_gains.angle_roll; break;
    case 1: gains = &cascaded_gains.angle_pitch; break;
    default: return;
  }
  
  gains->kp = kp;
  gains->ki = ki;
  gains->kd = kd;
  
  Serial.print("Angle PID gains updated for axis ");
  Serial.print(axis);
  Serial.print(": KP=");
  Serial.print(kp, 3);
  Serial.print(", KI=");
  Serial.print(ki, 3);
  Serial.print(", KD=");
  Serial.println(kd, 3);
}

void PIDController::reset_pid_integrals() {
  cascaded_gains.rate_roll.integral = 0;
  cascaded_gains.rate_pitch.integral = 0;
  cascaded_gains.rate_yaw.integral = 0;
  cascaded_gains.angle_roll.integral = 0;
  cascaded_gains.angle_pitch.integral = 0;
  cascaded_gains.altitude_rate.integral = 0;
  cascaded_gains.altitude_pos.integral = 0;
  
  Serial.println("All PID integrals reset");
}

#endif // PID_CONTROLLER_H
