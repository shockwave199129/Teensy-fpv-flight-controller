#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "config.h"
#include "constants.h"

// Pin definitions are now in config.h
// Voltage and current scaling factors are now in config.h/constants.h

// SensorHealthStatus is now defined in config.h to avoid circular dependencies

// Sensor types (forward declarations)
struct SensorData;
struct IMUData;
struct MagnetometerData;
struct BarometerData;
struct GPSData;
struct SonarData;
struct NotchFilter;

// Filter structures
struct MahonyFilter {
  float q0, q1, q2, q3;
  float integralFBx, integralFBy, integralFBz;
  bool initialized;
  unsigned long last_update;
};

struct VibratingFilter {
  float cutoff_hz;
  float alpha;
  float filtered_value;
  float raw_value;
  bool initialized;
};

// Function declarations
void init_mahony_filter();
void init_vibration_filters();
void init_notch_filters();
void mahony_ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float dt);
float apply_vibration_filter(VibratingFilter* filter, float input, float dt);
float apply_notch_filter(NotchFilter* filter, float input);
void calculate_notch_coefficients(NotchFilter* filter, float sample_rate);
void detect_vibration_issues(SensorData* data);

bool init_imu();
bool init_magnetometer();
bool init_barometer();
void init_gps();
void init_sonar();

bool read_imu_data(IMUData* data);
bool read_magnetometer_data(MagnetometerData* data);
bool read_barometer_data(BarometerData* data);
bool read_gps_data(GPSData* data);
bool read_sonar_data(SonarData* data);

void calibrate_imu();
void calibrate_magnetometer();

void update_sensor_fusion();
void complementary_filter(float dt);

// Added declarations for functions called from .ino
void init_sensors();
void update_sensors();
void update_magnetometer_timer();
bool perform_safety_checks();
void update_flight_state();
void init_motors();
void init_leds();
void init_communication();
void load_config();
void monitor_system_health();

// Sensor detection structure
struct SensorDetectionStatus {
  // IMU sensors
  bool mpu6050_present;
  bool mpu9250_present;
  bool icm20948_present;
  bool icm42688p_present;
  bool bmi270_present;
  bool lsm6dso32_present;
  bool bmi323_present;
  bool icm20602_present;
  bool lsm6ds33_present;
  
  // Magnetometer sensors
  bool hmc5883l_present;
  bool qmc5883l_present;
  bool rm3100_present;
  bool mmc5883ma_present;
  bool ist8310_present;
  bool lis3mdl_present;
  bool ak8963_present;
  
  // Barometer sensors
  bool bmp280_present;
  bool bme280_present;
  bool bmp388_present;
  bool ms5611_present;
  bool lps22hb_present;
  
  // Other sensors
  bool gps_present;
  bool sonar_present;
  bool optical_flow_present;
  bool voltage_sensor_present;
  bool current_sensor_present;
  
  // I2C device scan results
  uint8_t i2c_devices[16];
  uint8_t i2c_device_count;
  
  unsigned long last_detection_scan;
  bool detection_complete;
};

// Enhanced sensor fusion structures
struct EnhancedSensorFusion {
  bool multi_rate_fusion_enabled;
  unsigned long imu_update_interval;
  unsigned long mag_update_interval;
  unsigned long baro_update_interval;
  unsigned long gps_update_interval;
  
  bool adaptive_fusion_enabled;
  float imu_confidence_weight;
  float mag_confidence_weight;
  float baro_confidence_weight;
  float gps_confidence_weight;
  
  bool extended_kalman_filter_enabled;
  float state_estimate[9];
  float covariance_matrix[9][9];
  float process_noise[9];
  float measurement_noise[4];
  
  bool cross_validation_enabled;
  float sensor_agreement_threshold;
  bool sensor_outlier_detection;
  float outlier_rejection_threshold;
  
  bool environmental_adaptation_enabled;
  float vibration_environment_factor;
  float temperature_compensation_factor;
  float magnetic_declination_auto;
};

struct SensorQuality {
  float imu_quality_score;
  float gps_accuracy_score;
  float magnetometer_quality_score;
  float barometer_stability_score;
  
  unsigned long last_quality_update;
  bool quality_assessment_enabled;
};

struct AdvancedFiltering {
  bool adaptive_complementary_enabled;
  float complementary_alpha;
  float complementary_beta;
  
  bool motion_adaptive_filtering;
  float motion_threshold;
  bool in_motion;
  float static_filter_gain;
  float dynamic_filter_gain;
  
  bool gravity_vector_estimation;
  float estimated_gravity[3];
  float gravity_confidence;
};

// Function declarations for sensor detection
void scan_i2c_devices();
bool detect_imu_sensors();
bool detect_magnetometer_sensors();
bool detect_barometer_sensors();
bool detect_gps_sensor();
bool detect_sonar_sensor();
bool detect_optical_flow_sensor();
bool detect_voltage_current_sensors();
void perform_full_sensor_detection();
String get_sensor_detection_report();
String get_sensor_capabilities_json();

// Individual sensor detection
bool detect_mpu6050(uint8_t address = 0x68);
bool detect_mpu9250(uint8_t address = 0x68);
bool detect_icm20948(uint8_t address = 0x69);
bool detect_icm42688p(uint8_t address = 0x68);
bool detect_bmi270(uint8_t address = 0x68);
bool detect_lsm6dso32(uint8_t address = 0x6A);
bool detect_bmi323(uint8_t address = 0x68);
bool detect_lsm6ds33(uint8_t address = 0x6B);

bool detect_hmc5883l(uint8_t address = 0x1E);
bool detect_qmc5883l(uint8_t address = 0x0D);
bool detect_rm3100(uint8_t address = 0x20);
bool detect_mmc5883ma(uint8_t address = 0x30);
bool detect_ist8310(uint8_t address = 0x0E);
bool detect_lis3mdl(uint8_t address = 0x1C);
bool detect_ak8963(uint8_t address = 0x0C);

bool detect_bmp280(uint8_t address = 0x76);
bool detect_bme280(uint8_t address = 0x76);
bool detect_bmp388(uint8_t address = 0x76);
bool detect_ms5611(uint8_t address = 0x77);
bool detect_lps22hb(uint8_t address = 0x5C);

// Optical flow sensor detection
bool detect_pmw3901_spi();
bool detect_paa5100_i2c();
bool detect_adns3080_spi();

// Enhanced sensor fusion functions
void init_enhanced_sensor_fusion();
void update_enhanced_sensor_fusion();
void calculate_sensor_quality_scores();
void apply_adaptive_fusion_weights();
void detect_sensor_outliers();
void compensate_environmental_factors();
void update_extended_kalman_filter();
void validate_sensor_cross_correlation();
void adapt_filters_to_motion();
void estimate_gravity_vector();

// Safety functions
bool critical_sensor_check_with_feedback();
bool enhanced_safety_check_for_arming_with_feedback();
void check_automatic_battery_failsafe();
void handle_rc_signal_loss();

// Missing function implementations will be added to prevent compile errors
BatteryStatus get_enhanced_battery_status();
bool is_system_calibrated();
bool check_gps_functions_configured();
bool is_gps_functionality_available();

// Global variables
extern EnhancedSensorFusion enhanced_fusion;
extern SensorQuality sensor_quality;
extern AdvancedFiltering advanced_filtering;
extern SensorDetectionStatus sensor_detection;

// Inline implementations for enhanced sensor fusion
inline void calculate_sensor_quality_scores() {
  extern SensorData sensor_data;
  extern SensorQuality sensor_quality;

  // Simple heuristic quality metrics (0..1)
  sensor_quality.imu_quality_score = sensor_data.imu.healthy ? 1.0f : 0.0f;
  sensor_quality.gps_accuracy_score = sensor_data.gps.fix ? (sensor_data.gps.satellites / 12.0f) : 0.0f;
  sensor_quality.magnetometer_quality_score = sensor_data.mag.healthy ? 1.0f : 0.0f;
  sensor_quality.barometer_stability_score = sensor_data.baro.healthy ? 1.0f : 0.0f;

  sensor_quality.last_quality_update = millis();
}

inline void apply_adaptive_fusion_weights() {
  extern EnhancedSensorFusion enhanced_fusion;
  extern SensorQuality sensor_quality;

  // Map quality scores directly to confidence weights
  enhanced_fusion.imu_confidence_weight  = sensor_quality.imu_quality_score;
  enhanced_fusion.mag_confidence_weight  = sensor_quality.magnetometer_quality_score;
  enhanced_fusion.baro_confidence_weight = sensor_quality.barometer_stability_score;
  enhanced_fusion.gps_confidence_weight  = sensor_quality.gps_accuracy_score;
}

inline void detect_sensor_outliers() {
  // Placeholder simplistic outlier detection: not enough data for true stats.
  // For now it just sets a flag if any sensor marked unhealthy.
  extern SensorData sensor_data;
  extern EnhancedSensorFusion enhanced_fusion;

  enhanced_fusion.sensor_outlier_detection =
      !(sensor_data.imu.healthy && sensor_data.mag.healthy &&
        sensor_data.baro.healthy && sensor_data.gps.healthy);
}

inline void compensate_environmental_factors() {
  extern EnhancedSensorFusion enhanced_fusion;
  // Basic placeholder: temperature compensation could be added here.
  enhanced_fusion.temperature_compensation_factor = 1.0f; // no-op
}

inline void update_extended_kalman_filter() {
  extern EnhancedSensorFusion enhanced_fusion;
  if (!enhanced_fusion.extended_kalman_filter_enabled) return;
  // Very simplified EKF one-step predict/update placeholder
  // In production, call into full EKF library here
}

inline void validate_sensor_cross_correlation() {
  // Simple check: if IMU and magnetometer yaw differ > threshold, flag
  extern SensorData sensor_data;
  extern EnhancedSensorFusion enhanced_fusion;
  float yaw_diff = fabsf(sensor_data.yaw - sensor_data.mag.heading);
  enhanced_fusion.sensor_agreement_threshold = yaw_diff;
}

inline void adapt_filters_to_motion() {
  extern SensorData sensor_data;
  extern AdvancedFiltering advanced_filtering;
  float accel_mag = sqrtf(sensor_data.imu.accel_x*sensor_data.imu.accel_x +
                          sensor_data.imu.accel_y*sensor_data.imu.accel_y +
                          sensor_data.imu.accel_z*sensor_data.imu.accel_z);
  bool moving = fabsf(accel_mag - 9.81f) > advanced_filtering.motion_threshold;
  advanced_filtering.in_motion = moving;
}

inline void estimate_gravity_vector() {
  extern SensorData sensor_data;
  extern AdvancedFiltering advanced_filtering;
  advanced_filtering.estimated_gravity[0] = sensor_data.imu.accel_x;
  advanced_filtering.estimated_gravity[1] = sensor_data.imu.accel_y;
  advanced_filtering.estimated_gravity[2] = sensor_data.imu.accel_z;
  advanced_filtering.gravity_confidence = 1.0f;
}

void apply_imu_calibration(IMUData* data);
void apply_magnetometer_calibration(MagnetometerData* data);

#endif // SENSORS_H 