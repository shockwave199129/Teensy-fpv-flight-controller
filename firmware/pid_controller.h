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

struct AdvancedPIDFeatures {
  // Adaptive PID gains based on flight conditions
  bool adaptive_gains_enabled;
  float gain_scheduler_factor;      // 0.5-2.0 gain multiplier
  float load_factor_compensation;   // Compensate for varying load
  
  // Setpoint weighting for smoother response
  bool setpoint_weighting_enabled;
  float setpoint_weight_p;          // 0.0-1.0 (0=no setpoint weighting)
  float setpoint_weight_d;          // 0.0-1.0
  
  // Feedforward control
  bool feedforward_enabled;
  float feedforward_gain;           // Feedforward gain
  float velocity_feedforward;       // Velocity-based feedforward
  
  // Dynamic response adjustment
  bool dynamic_response_enabled;
  float aggressiveness_level;       // 0.1-2.0 (flight characteristic)
  float smoothness_factor;          // 0.1-2.0 (cinematic vs sport)
  
  // Harmonic compensation
  bool harmonic_compensation_enabled;
  float motor_harmonic_filters[4];  // Per-motor harmonic filtering
  
  // Advanced anti-windup
  bool conditional_integration;     // Smart integral freeze
  bool dynamic_integral_limit;      // Variable integral limits
  float integral_decay_rate;        // Integral term decay
};

struct FlightPhaseDetection {
  enum FlightPhase {
    PHASE_GROUND = 0,
    PHASE_TAKEOFF,
    PHASE_HOVER,
    PHASE_FORWARD_FLIGHT,
    PHASE_AGGRESSIVE_MANEUVERS,
    PHASE_LANDING
  } current_phase;
  
  float hover_detection_threshold;
  float aggressive_detection_threshold;
  float forward_flight_speed_threshold;
  unsigned long phase_transition_time;
  bool phase_stable;
};

struct PerformanceMetrics {
  float control_effort[3];          // Roll, pitch, yaw control effort
  float setpoint_tracking_error[3]; // RMS error tracking
  float oscillation_frequency[3];   // Detected oscillation frequencies
  float vibration_level;            // Overall vibration metric
  float efficiency_score;           // Control efficiency (0-100)
  unsigned long last_metrics_update;
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
  void update_rate_loop_direct(SensorData& sensor_data, RcData& rc_data, float dt);
  void apply_motor_saturation_compensation();
  
  // Filtering and smoothing
  float apply_derivative_filter(float derivative, float alpha);
  void reset_pid_state(EnhancedPIDGains* gains);
  
  // Advanced PID features
  AdvancedPIDFeatures advanced_features;
  FlightPhaseDetection phase_detection;
  PerformanceMetrics performance_metrics;
  
  // Adaptive control methods
  void update_adaptive_gains(SensorData& sensor_data, RcData& rc_data);
  void detect_flight_phase(SensorData& sensor_data, RcData& rc_data);
  void calculate_performance_metrics();
  float calculate_setpoint_weighted_error(float setpoint, float measured, float weight);
  float calculate_feedforward_term(float setpoint_rate, float gains);
  void apply_harmonic_compensation();
  
  // Flight characteristic adjustments
  void set_flight_characteristics(float aggressiveness, float smoothness);
  void optimize_for_flight_phase(FlightPhaseDetection::FlightPhase phase);
  
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
  // Reset integral terms for all PID loops
  cascaded_gains.rate_roll.integral = 0;
  cascaded_gains.rate_pitch.integral = 0;
  cascaded_gains.rate_yaw.integral = 0;
  cascaded_gains.angle_roll.integral = 0;
  cascaded_gains.angle_pitch.integral = 0;
  cascaded_gains.altitude_rate.integral = 0;
  cascaded_gains.altitude_pos.integral = 0;
  
  Serial.println("PID integral terms reset");
}

// =================== PHASE 3: ADVANCED PID IMPLEMENTATIONS ===================

void PIDController::update_adaptive_gains(SensorData& sensor_data, RcData& rc_data) {
  if (!advanced_features.adaptive_gains_enabled) return;
  
  // Calculate load factor based on motor saturation and control effort
  float total_control_effort = abs(final_output.roll) + abs(final_output.pitch) + abs(final_output.yaw);
  float load_factor = constrain(total_control_effort / 1000.0f, 0.1f, 2.0f);
  
  // Detect wind disturbance from control effort variance
  static float control_effort_history[10] = {0};
  static int history_index = 0;
  control_effort_history[history_index++] = total_control_effort;
  if (history_index >= 10) history_index = 0;
  
  float control_variance = 0;
  float mean_effort = 0;
  for (int i = 0; i < 10; i++) {
    mean_effort += control_effort_history[i];
  }
  mean_effort /= 10.0f;
  
  for (int i = 0; i < 10; i++) {
    control_variance += pow(control_effort_history[i] - mean_effort, 2);
  }
  control_variance /= 10.0f;
  
  // Adaptive gain scheduling
  float disturbance_factor = constrain(sqrt(control_variance) / 100.0f, 0.5f, 2.0f);
  advanced_features.gain_scheduler_factor = disturbance_factor * load_factor;
  
  // Apply adaptive gains to rate loops (more responsive to disturbances)
  float adaptive_multiplier = advanced_features.gain_scheduler_factor;
  
  // Increase P gain for better disturbance rejection
  cascaded_gains.rate_roll.kp *= (0.8f + 0.4f * adaptive_multiplier);
  cascaded_gains.rate_pitch.kp *= (0.8f + 0.4f * adaptive_multiplier);
  cascaded_gains.rate_yaw.kp *= (0.9f + 0.2f * adaptive_multiplier);
  
  // Moderate I gain increase for steady-state accuracy
  cascaded_gains.rate_roll.ki *= (0.9f + 0.2f * adaptive_multiplier);
  cascaded_gains.rate_pitch.ki *= (0.9f + 0.2f * adaptive_multiplier);
  cascaded_gains.rate_yaw.ki *= (0.95f + 0.1f * adaptive_multiplier);
}

void PIDController::detect_flight_phase(SensorData& sensor_data, RcData& rc_data) {
  static unsigned long last_phase_update = 0;
  if (millis() - last_phase_update < 100) return; // 10Hz update
  
  float throttle_percent = (rc_data.throttle - 1000) / 10.0f;
  float stick_movement = abs(rc_data.roll - 1500) + abs(rc_data.pitch - 1500) + abs(rc_data.yaw - 1500);
  float angular_velocity = sqrt(pow(sensor_data.imu.gyro_x, 2) + pow(sensor_data.imu.gyro_y, 2) + pow(sensor_data.imu.gyro_z, 2));
  
  FlightPhaseDetection::FlightPhase new_phase = phase_detection.current_phase;
  
  if (throttle_percent < 20) {
    new_phase = FlightPhaseDetection::PHASE_GROUND;
  } else if (throttle_percent > 20 && throttle_percent < 40 && stick_movement < 300) {
    new_phase = FlightPhaseDetection::PHASE_TAKEOFF;
  } else if (stick_movement < 200 && angular_velocity < 50) {
    new_phase = FlightPhaseDetection::PHASE_HOVER;
  } else if (stick_movement > 500 || angular_velocity > 200) {
    new_phase = FlightPhaseDetection::PHASE_AGGRESSIVE_MANEUVERS;
  } else if (throttle_percent > 40) {
    new_phase = FlightPhaseDetection::PHASE_FORWARD_FLIGHT;
  }
  
  if (new_phase != phase_detection.current_phase) {
    phase_detection.current_phase = new_phase;
    phase_detection.phase_transition_time = millis();
    phase_detection.phase_stable = false;
    optimize_for_flight_phase(new_phase);
  } else if (millis() - phase_detection.phase_transition_time > 2000) {
    phase_detection.phase_stable = true;
  }
  
  last_phase_update = millis();
}

void PIDController::optimize_for_flight_phase(FlightPhaseDetection::FlightPhase phase) {
  switch (phase) {
    case FlightPhaseDetection::PHASE_GROUND:
      set_flight_characteristics(0.3f, 2.0f);  // Minimal gains
      break;
    case FlightPhaseDetection::PHASE_TAKEOFF:
      set_flight_characteristics(0.7f, 1.5f);  // Stable takeoff
      break;
    case FlightPhaseDetection::PHASE_HOVER:
      set_flight_characteristics(1.0f, 1.8f);  // Maximum stability
      advanced_features.setpoint_weighting_enabled = true;
      break;
    case FlightPhaseDetection::PHASE_FORWARD_FLIGHT:
      set_flight_characteristics(1.2f, 1.2f);  // Balanced response
      advanced_features.feedforward_enabled = true;
      break;
    case FlightPhaseDetection::PHASE_AGGRESSIVE_MANEUVERS:
      set_flight_characteristics(2.0f, 0.8f);  // Maximum responsiveness
      advanced_features.setpoint_weighting_enabled = false;
      break;
    case FlightPhaseDetection::PHASE_LANDING:
      set_flight_characteristics(0.6f, 2.0f);  // Gentle landing
      break;
  }
}

void PIDController::set_flight_characteristics(float aggressiveness, float smoothness) {
  advanced_features.aggressiveness_level = constrain(aggressiveness, 0.1f, 2.0f);
  advanced_features.smoothness_factor = constrain(smoothness, 0.1f, 2.0f);
  
  // Apply aggressiveness to rate loop gains
  float rate_multiplier = advanced_features.aggressiveness_level;
  cascaded_gains.rate_roll.kp *= rate_multiplier;
  cascaded_gains.rate_pitch.kp *= rate_multiplier;
  cascaded_gains.rate_yaw.kp *= (rate_multiplier * 0.8f);
  
  // Apply smoothness to derivative filtering
  float smooth_factor = advanced_features.smoothness_factor;
  cascaded_gains.rate_roll.derivative_filter_alpha = 0.1f * smooth_factor;
  cascaded_gains.rate_pitch.derivative_filter_alpha = 0.1f * smooth_factor;
  cascaded_gains.rate_yaw.derivative_filter_alpha = 0.15f * smooth_factor;
  
  // Adjust angle loop responsiveness
  cascaded_gains.angle_roll.kp *= (2.0f / smooth_factor);
  cascaded_gains.angle_pitch.kp *= (2.0f / smooth_factor);
}

void PIDController::calculate_performance_metrics() {
  static unsigned long last_metrics_calc = 0;
  if (millis() - last_metrics_calc < 1000) return;
  
  // Calculate control effort
  performance_metrics.control_effort[0] = abs(final_output.roll) / 500.0f * 100.0f;
  performance_metrics.control_effort[1] = abs(final_output.pitch) / 500.0f * 100.0f;
  performance_metrics.control_effort[2] = abs(final_output.yaw) / 500.0f * 100.0f;
  
  // Calculate efficiency score
  float total_effort = (performance_metrics.control_effort[0] + 
                       performance_metrics.control_effort[1] + 
                       performance_metrics.control_effort[2]) / 3.0f;
  performance_metrics.efficiency_score = 100.0f - constrain(total_effort, 0, 100);
  
  performance_metrics.last_metrics_update = millis();
  last_metrics_calc = millis();
}

float PIDController::calculate_setpoint_weighted_error(float setpoint, float measured, float weight) {
  if (!advanced_features.setpoint_weighting_enabled) return setpoint - measured;
  float weighted_setpoint = weight * setpoint + (1.0f - weight) * measured;
  return weighted_setpoint - measured;
}

float PIDController::calculate_feedforward_term(float setpoint_rate, float gain) {
  if (!advanced_features.feedforward_enabled) return 0;
  return setpoint_rate * gain * advanced_features.velocity_feedforward;
}

void PIDController::apply_harmonic_compensation() {
  if (!advanced_features.harmonic_compensation_enabled) return;
  
  extern MotorControl motor_control;
  for (int i = 0; i < 4; i++) {
    uint16_t motor_rpm = motor_control.get_motor_rpm(i);
    if (motor_rpm > 0) {
      float fundamental_freq = (motor_rpm * 14) / 60.0f; // 14-pole motors
      advanced_features.motor_harmonic_filters[i] = fundamental_freq;
    }
  }
}

// Advanced PID Methods Implementation

void PIDController::update_adaptive_gains(SensorData& sensor_data, RcData& rc_data) {
  if (!advanced_features.adaptive_gains_enabled) return;
  
  // Calculate load factor based on motor saturation and control effort
  float total_control_effort = abs(final_output.roll) + abs(final_output.pitch) + abs(final_output.yaw);
  float load_factor = constrain(total_control_effort / 1000.0f, 0.1f, 2.0f);
  
  // Detect wind disturbance from control effort variance
  static float control_effort_history[10] = {0};
  static int history_index = 0;
  control_effort_history[history_index++] = total_control_effort;
  if (history_index >= 10) history_index = 0;
  
  float control_variance = 0;
  float mean_effort = 0;
  for (int i = 0; i < 10; i++) {
    mean_effort += control_effort_history[i];
  }
  mean_effort /= 10.0f;
  
  for (int i = 0; i < 10; i++) {
    control_variance += pow(control_effort_history[i] - mean_effort, 2);
  }
  control_variance /= 10.0f;
  
  // Adaptive gain scheduling
  float disturbance_factor = constrain(sqrt(control_variance) / 100.0f, 0.5f, 2.0f);
  advanced_features.gain_scheduler_factor = disturbance_factor * load_factor;
  
  // Apply adaptive gains to rate loops (more responsive to disturbances)
  float adaptive_multiplier = advanced_features.gain_scheduler_factor;
  
  // Increase P gain for better disturbance rejection
  cascaded_gains.rate_roll.kp *= (0.8f + 0.4f * adaptive_multiplier);
  cascaded_gains.rate_pitch.kp *= (0.8f + 0.4f * adaptive_multiplier);
  cascaded_gains.rate_yaw.kp *= (0.9f + 0.2f * adaptive_multiplier);
  
  // Moderate I gain increase for steady-state accuracy
  cascaded_gains.rate_roll.ki *= (0.9f + 0.2f * adaptive_multiplier);
  cascaded_gains.rate_pitch.ki *= (0.9f + 0.2f * adaptive_multiplier);
  cascaded_gains.rate_yaw.ki *= (0.95f + 0.1f * adaptive_multiplier);
}

void PIDController::detect_flight_phase(SensorData& sensor_data, RcData& rc_data) {
  static unsigned long last_phase_update = 0;
  if (millis() - last_phase_update < 100) return; // 10Hz update
  
  float throttle_percent = (rc_data.throttle - 1000) / 10.0f;
  float stick_movement = abs(rc_data.roll - 1500) + abs(rc_data.pitch - 1500) + abs(rc_data.yaw - 1500);
  float angular_velocity = sqrt(pow(sensor_data.imu.gyro_x, 2) + pow(sensor_data.imu.gyro_y, 2) + pow(sensor_data.imu.gyro_z, 2));
  
  FlightPhaseDetection::FlightPhase new_phase = phase_detection.current_phase;
  
  if (throttle_percent < 20) {
    new_phase = FlightPhaseDetection::PHASE_GROUND;
  } else if (throttle_percent > 20 && throttle_percent < 40 && stick_movement < 300) {
    new_phase = FlightPhaseDetection::PHASE_TAKEOFF;
  } else if (stick_movement < 200 && angular_velocity < 50) {
    new_phase = FlightPhaseDetection::PHASE_HOVER;
  } else if (stick_movement > 500 || angular_velocity > 200) {
    new_phase = FlightPhaseDetection::PHASE_AGGRESSIVE_MANEUVERS;
  } else if (throttle_percent > 40) {
    new_phase = FlightPhaseDetection::PHASE_FORWARD_FLIGHT;
  }
  
  if (new_phase != phase_detection.current_phase) {
    phase_detection.current_phase = new_phase;
    phase_detection.phase_transition_time = millis();
    phase_detection.phase_stable = false;
    optimize_for_flight_phase(new_phase);
  } else if (millis() - phase_detection.phase_transition_time > 2000) {
    phase_detection.phase_stable = true;
  }
  
  last_phase_update = millis();
}

void PIDController::optimize_for_flight_phase(FlightPhaseDetection::FlightPhase phase) {
  switch (phase) {
    case FlightPhaseDetection::PHASE_GROUND:
      // Minimal gains for ground operations
      set_flight_characteristics(0.3f, 2.0f);
      break;
      
    case FlightPhaseDetection::PHASE_TAKEOFF:
      // Stable, responsive takeoff
      set_flight_characteristics(0.7f, 1.5f);
      break;
      
    case FlightPhaseDetection::PHASE_HOVER:
      // Maximum stability for hover
      set_flight_characteristics(1.0f, 1.8f);
      advanced_features.setpoint_weighting_enabled = true;
      advanced_features.setpoint_weight_p = 0.7f;
      break;
      
    case FlightPhaseDetection::PHASE_FORWARD_FLIGHT:
      // Balanced response for forward flight
      set_flight_characteristics(1.2f, 1.2f);
      advanced_features.feedforward_enabled = true;
      break;
      
    case FlightPhaseDetection::PHASE_AGGRESSIVE_MANEUVERS:
      // Maximum responsiveness for acro
      set_flight_characteristics(2.0f, 0.8f);
      advanced_features.setpoint_weighting_enabled = false;
      advanced_features.feedforward_enabled = true;
      break;
      
    case FlightPhaseDetection::PHASE_LANDING:
      // Gentle, stable landing
      set_flight_characteristics(0.6f, 2.0f);
      break;
  }
}

void PIDController::set_flight_characteristics(float aggressiveness, float smoothness) {
  advanced_features.aggressiveness_level = constrain(aggressiveness, 0.1f, 2.0f);
  advanced_features.smoothness_factor = constrain(smoothness, 0.1f, 2.0f);
  
  // Apply aggressiveness to rate loop gains
  float rate_multiplier = advanced_features.aggressiveness_level;
  cascaded_gains.rate_roll.kp *= rate_multiplier;
  cascaded_gains.rate_pitch.kp *= rate_multiplier;
  cascaded_gains.rate_yaw.kp *= (rate_multiplier * 0.8f); // Yaw less aggressive
  
  // Apply smoothness to derivative filtering and angle loop
  float smooth_factor = advanced_features.smoothness_factor;
  cascaded_gains.rate_roll.derivative_filter_alpha = 0.1f * smooth_factor;
  cascaded_gains.rate_pitch.derivative_filter_alpha = 0.1f * smooth_factor;
  cascaded_gains.rate_yaw.derivative_filter_alpha = 0.15f * smooth_factor;
  
  // Adjust angle loop responsiveness
  cascaded_gains.angle_roll.kp *= (2.0f / smooth_factor);
  cascaded_gains.angle_pitch.kp *= (2.0f / smooth_factor);
}

float PIDController::calculate_setpoint_weighted_error(float setpoint, float measured, float weight) {
  if (!advanced_features.setpoint_weighting_enabled) {
    return setpoint - measured;
  }
  
  // Setpoint weighting reduces proportional kick on setpoint changes
  float weighted_setpoint = weight * setpoint + (1.0f - weight) * measured;
  return weighted_setpoint - measured;
}

float PIDController::calculate_feedforward_term(float setpoint_rate, float gain) {
  if (!advanced_features.feedforward_enabled) return 0;
  
  // Simple velocity feedforward
  return setpoint_rate * gain * advanced_features.velocity_feedforward;
}

void PIDController::calculate_performance_metrics() {
  static unsigned long last_metrics_calc = 0;
  if (millis() - last_metrics_calc < 1000) return; // 1Hz update
  
  // Calculate control effort (how hard the system is working)
  performance_metrics.control_effort[0] = abs(final_output.roll) / 500.0f * 100.0f;
  performance_metrics.control_effort[1] = abs(final_output.pitch) / 500.0f * 100.0f;
  performance_metrics.control_effort[2] = abs(final_output.yaw) / 500.0f * 100.0f;
  
  // Calculate efficiency score (lower control effort = higher efficiency)
  float total_effort = performance_metrics.control_effort[0] + 
                       performance_metrics.control_effort[1] + 
                       performance_metrics.control_effort[2];
  performance_metrics.efficiency_score = 100.0f - constrain(total_effort / 3.0f, 0, 100);
  
  // Detect oscillations in rate loops
  static float rate_history[3][20] = {{0}};
  static int history_idx = 0;
  
  extern SensorData sensor_data;
  rate_history[0][history_idx] = sensor_data.imu.gyro_x;
  rate_history[1][history_idx] = sensor_data.imu.gyro_y;
  rate_history[2][history_idx] = sensor_data.imu.gyro_z;
  
  history_idx = (history_idx + 1) % 20;
  
  // Simple oscillation detection using zero crossings
  for (int axis = 0; axis < 3; axis++) {
    int zero_crossings = 0;
    for (int i = 1; i < 20; i++) {
      if ((rate_history[axis][i] > 0) != (rate_history[axis][i-1] > 0)) {
        zero_crossings++;
      }
    }
    performance_metrics.oscillation_frequency[axis] = zero_crossings * 2.5f; // Approximate frequency
  }
  
  performance_metrics.last_metrics_update = millis();
  last_metrics_calc = millis();
}

void PIDController::apply_harmonic_compensation() {
  if (!advanced_features.harmonic_compensation_enabled) return;
  
  extern MotorControl motor_control;
  
  // Get motor RPM data for harmonic filtering
  for (int i = 0; i < 4; i++) {
    uint16_t motor_rpm = motor_control.get_motor_rpm(i);
    if (motor_rpm > 0) {
      // Calculate fundamental frequency and harmonics
      float fundamental_freq = (motor_rpm * 14) / 60.0f; // 14-pole motors
      float second_harmonic = fundamental_freq * 2;
      float third_harmonic = fundamental_freq * 3;
      
      // Apply notch filtering at harmonic frequencies
      // This would integrate with the existing notch filter system
      advanced_features.motor_harmonic_filters[i] = fundamental_freq;
    }
  }
}

#endif // PID_CONTROLLER_H
