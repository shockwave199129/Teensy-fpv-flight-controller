#include "dual_imu_manager.h"
#include "sensors.h"
#include "icm20948.h"

extern bool read_imu_data_mpu(IMUData* data);

DualIMUManager::DualIMUManager() {
  primary_imu_initialized = false;
  secondary_imu_initialized = false;
  using_primary_imu = true;
  validation_enabled = true;
  last_validation_time = 0;
  cross_check_failures = 0;
}

void DualIMUManager::init() {
  Serial.println("Initializing Dual IMU System...");
  
  // Initialize primary IMU (standard address)
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    primary_imu_initialized = true;
    Serial.println("Primary IMU detected at 0x68");
  }
  
  // Try alternate I2C address first
  Wire.beginTransmission(0x69);
  if (Wire.endTransmission() == 0) {
    secondary_imu_initialized = true;
    Serial.println("Secondary MPU6050 detected at 0x69");
  } else if (icm20948_init()) {
    secondary_imu_initialized = true;
    Serial.println("Secondary IMU is ICM20948 (SPI)");
  }
  
  if (!primary_imu_initialized && !secondary_imu_initialized) {
    Serial.println("ERROR: No IMUs detected!");
    return;
  }
  
  if (primary_imu_initialized && secondary_imu_initialized) {
    Serial.println("Dual IMU system active with cross-validation");
    validation_enabled = true;
  } else {
    Serial.println("Single IMU system active");
    validation_enabled = false;
    using_primary_imu = primary_imu_initialized;
  }
  
  // Initialize fused data
  fused_data.healthy = false;
  fused_data.last_update = 0;
}

void DualIMUManager::update() {
  if (!primary_imu_initialized && !secondary_imu_initialized) {
    return;
  }
  
  IMUData primary_data, secondary_data;
  bool primary_read_success = false;
  bool secondary_read_success = false;
  
  // Read primary IMU
  if (primary_imu_initialized) {
    primary_read_success = read_imu_data(&primary_data);
  }
  
  // Read secondary IMU  
  if (secondary_imu_initialized) {
    secondary_read_success = read_imu_data_secondary(&secondary_data);
  }
  
  // Perform cross-validation if both IMUs available
  if (validation_enabled && primary_read_success && secondary_read_success) {
    validate_imu_data(primary_data, secondary_data);
  }
  
  // Select best data source
  if (using_primary_imu && primary_read_success) {
    fused_data = primary_data;
  } else if (!using_primary_imu && secondary_read_success) {
    fused_data = secondary_data;
  } else if (primary_read_success) {
    fused_data = primary_data;
    using_primary_imu = true;
  } else if (secondary_read_success) {
    fused_data = secondary_data;
    using_primary_imu = false;
  } else {
    fused_data.healthy = false;
    Serial.println("ERROR: Both IMUs failed to read");
  }
  
  fused_data.last_update = millis();
}

void DualIMUManager::validate_imu_data(const IMUData& primary, const IMUData& secondary) {
  unsigned long current_time = millis();
  
  if (current_time - last_validation_time < 100) { // Validate at 10Hz
    return;
  }
  
  last_validation_time = current_time;
  
  // Calculate differences in accelerometer readings
  float accel_diff = sqrt(pow(primary.accel_x - secondary.accel_x, 2) +
                         pow(primary.accel_y - secondary.accel_y, 2) +
                         pow(primary.accel_z - secondary.accel_z, 2));
  
  // Calculate differences in gyroscope readings  
  float gyro_diff = sqrt(pow(primary.gyro_x - secondary.gyro_x, 2) +
                        pow(primary.gyro_y - secondary.gyro_y, 2) +
                        pow(primary.gyro_z - secondary.gyro_z, 2));
  
  // Validation thresholds
  const float ACCEL_DIFF_THRESHOLD = 2.0f; // m/s²
  const float GYRO_DIFF_THRESHOLD = 10.0f; // deg/s
  
  bool validation_passed = (accel_diff < ACCEL_DIFF_THRESHOLD) && 
                          (gyro_diff < GYRO_DIFF_THRESHOLD);
  
  if (!validation_passed) {
    cross_check_failures++;
    
    if (cross_check_failures > 10) { // 10 consecutive failures
      Serial.println("WARNING: IMU cross-validation failed - switching to single IMU mode");
      
      // Determine which IMU is more likely to be correct
      float primary_magnitude = sqrt(pow(primary.accel_x, 2) + pow(primary.accel_y, 2) + pow(primary.accel_z, 2));
      float secondary_magnitude = sqrt(pow(secondary.accel_x, 2) + pow(secondary.accel_y, 2) + pow(secondary.accel_z, 2));
      
      // Use the IMU with accelerometer magnitude closer to 9.81 m/s²
      float primary_error = abs(primary_magnitude - 9.81f);
      float secondary_error = abs(secondary_magnitude - 9.81f);
      
      using_primary_imu = (primary_error < secondary_error);
      
      Serial.print("Selected IMU: ");
      Serial.println(using_primary_imu ? "Primary" : "Secondary");
      
      cross_check_failures = 0; // Reset counter
    }
  } else {
    cross_check_failures = 0; // Reset on successful validation
  }
}

bool DualIMUManager::read_imu_data_secondary(IMUData* data) {
  // Attempt I2C secondary MPU first
  bool success = false;
  Wire.beginTransmission(0x69);
  if (Wire.endTransmission() == 0) {
    success = read_imu_data_mpu(data); // reuse primary helper with alt addr? simplification
  } else if (icm20948_read(data)) {
    success = true;
  }
  return success;
}

IMUData DualIMUManager::get_fused_data() {
  return fused_data;
}

bool DualIMUManager::is_healthy() {
  return fused_data.healthy;
}

bool DualIMUManager::using_primary() {
  return using_primary_imu;
} 