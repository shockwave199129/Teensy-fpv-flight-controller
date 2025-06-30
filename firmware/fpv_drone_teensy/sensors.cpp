#include "sensors.h"
#include "Arduino.h"
#include "config.h"
#include <math.h>
#include "safety_functions.h"
#include "icm20948.h"
#include "calibration_storage.h"
#include <Wire.h>

// Forward declaration for internal MPU read helper
bool read_imu_data_mpu(IMUData* data);

// Stub implementations for sensor functions

#if !defined(MPU6050_ADDR)
#define MPU6050_ADDR 0x68
#endif

// Quaternion state for fusion
static float q0=1,q1=0,q2=0,q3=0;
static unsigned long lastFusionTime = 0;

// Mahony gains
#define MAHONY_TWO_KP 2.0f
#define MAHONY_TWO_KI 0.0f
#define MAHONY_AHRS_IMPLEMENTED 1
static float integralFBx=0, integralFBy=0, integralFBz=0;

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (3.14159265359f/180.0f)
#endif

#ifndef PI
#define PI 3.14159265359f
#endif

static bool init_imu_mpu() {
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) return false;
  // Wake up MPU6050 (write 0 to power-mgmt-1)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  return Wire.endTransmission() == 0;
}

static bool init_imu_icm() {
  return icm20948_init();
}

static enum {IMU_NONE, IMU_MPU, IMU_ICM} active_imu = IMU_NONE;

bool init_imu() {
  if (init_imu_mpu()) { active_imu = IMU_MPU; return true; }
  if (init_imu_icm()) { active_imu = IMU_ICM; return true; }
  return false;
}

bool init_magnetometer() {
  extern SensorData sensor_data;
  bool found = false;

  if (detect_hmc5883l()) { found = true; }
  else if (detect_qmc5883l()) { found = true; }
  else if (detect_rm3100())   { found = true; }
  else if (detect_lis3mdl())  { found = true; }

  sensor_data.mag.healthy = found;
  Serial.print("Magnetometer init: "); Serial.println(found ? "OK" : "NOT FOUND");
  return found;
}

bool init_barometer() {
  extern SensorData sensor_data;
  bool found = false;
  if (detect_bmp280())  found = true;
  else if (detect_bmp388()) found = true;
  else if (detect_ms5611()) found = true;

  sensor_data.baro.healthy = found;
  Serial.print("Barometer init: "); Serial.println(found ? "OK" : "NOT FOUND");
  return found;
}

void init_gps() {
  extern SensorData sensor_data;
  Serial1.begin(9600);
  sensor_data.gps.healthy = true; // we assume GPS module present; fix comes later
  Serial.println("GPS UART initialised (9600 baud)");
}

void init_sonar() {
  extern SensorData sensor_data;
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  sensor_data.sonar.healthy = true;
  Serial.println("Sonar pins configured");
}

bool read_imu_data(IMUData* data) {
  switch(active_imu) {
    case IMU_MPU: return read_imu_data_mpu(data);
    case IMU_ICM: return icm20948_read(data);
    default: return false;
  }
}

bool read_imu_data_mpu(IMUData* data) {
  // Read 14 bytes starting at accel X high (0x3B)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU6050_ADDR, 14);
  if (Wire.available() < 14) return false;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t tempRaw = (Wire.read() << 8) | Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Convert to physical units
  data->accel_x = ax / 16384.0f * 9.81f;
  data->accel_y = ay / 16384.0f * 9.81f;
  data->accel_z = az / 16384.0f * 9.81f;
  data->gyro_x = gx / 131.0f; // deg/s
  data->gyro_y = gy / 131.0f;
  data->gyro_z = gz / 131.0f;
  data->temp = tempRaw / 340.0f + 36.53f;
  data->healthy = true;
  data->last_update = millis();
  return true;
}

// Helper to check if an address was detected
static bool addr_found(uint8_t addr) {
  extern SensorDetectionStatus sensor_detection;
  for (uint8_t i = 0; i < sensor_detection.i2c_device_count; i++) {
    if (sensor_detection.i2c_devices[i] == addr) return true;
  }
  return false;
}

void scan_i2c_devices() {
  extern SensorDetectionStatus sensor_detection;

  sensor_detection.i2c_device_count = 0;
  for (uint8_t addr = 1; addr < 127 && sensor_detection.i2c_device_count < 16; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      sensor_detection.i2c_devices[sensor_detection.i2c_device_count++] = addr;
    }
  }

  sensor_detection.last_detection_scan = millis();
}

void perform_full_sensor_detection() {
  extern SensorDetectionStatus sensor_detection;

  // Clear previous results
  memset(&sensor_detection, 0, sizeof(SensorDetectionStatus));

  Serial.println("Scanning I2C bus for sensors ...");
  scan_i2c_devices();

  // --- IMUs ---
  sensor_detection.mpu6050_present   = addr_found(0x68);
  sensor_detection.mpu9250_present   = addr_found(0x68); // same addr as MPU6050
  sensor_detection.icm20948_present  = addr_found(0x69);
  sensor_detection.icm42688p_present = addr_found(0x68);
  sensor_detection.bmi270_present    = addr_found(0x68);
  sensor_detection.lsm6dso32_present = addr_found(0x6A);
  sensor_detection.bmi323_present    = addr_found(0x68);
  sensor_detection.icm20602_present  = addr_found(0x68);
  sensor_detection.lsm6ds33_present  = addr_found(0x6B);

  // --- Magnetometers ---
  sensor_detection.hmc5883l_present  = addr_found(0x1E);
  sensor_detection.qmc5883l_present  = addr_found(0x0D);
  sensor_detection.rm3100_present    = addr_found(0x20);
  sensor_detection.mmc5883ma_present = addr_found(0x30);
  sensor_detection.ist8310_present   = addr_found(0x0E);
  sensor_detection.lis3mdl_present   = addr_found(0x1C);
  sensor_detection.ak8963_present    = addr_found(0x0C);

  // --- Barometers ---
  sensor_detection.bmp280_present    = addr_found(0x76);
  sensor_detection.bme280_present    = addr_found(0x76);
  sensor_detection.bmp388_present    = addr_found(0x76);
  sensor_detection.ms5611_present    = addr_found(0x77);
  sensor_detection.lps22hb_present   = addr_found(0x5C);

  // Other (presence not detectable via I2C here)
  sensor_detection.gps_present       = true; // Assume GPS on UART for now
  sensor_detection.sonar_present     = true; // Assume sonar initialized elsewhere

  sensor_detection.detection_complete = true;

  // Print summary
  Serial.println("I2C devices found:");
  for (uint8_t i = 0; i < sensor_detection.i2c_device_count; i++) {
    Serial.print("  0x"); Serial.println(sensor_detection.i2c_devices[i], HEX);
  }

  Serial.println("Sensor presence flags:");
  Serial.print("  MPU6050: "); Serial.println(sensor_detection.mpu6050_present);
  Serial.print("  ICM20948: "); Serial.println(sensor_detection.icm20948_present);
  Serial.print("  HMC5883L: "); Serial.println(sensor_detection.hmc5883l_present);
  Serial.print("  BMP280: "); Serial.println(sensor_detection.bmp280_present);
  // Add more prints as needed
}

String get_sensor_detection_report() {
  extern SensorDetectionStatus sensor_detection;
  String report = "{\n";
  report += "  \"mpu6050\": " + String(sensor_detection.mpu6050_present ? "true" : "false") + ",\n";
  report += "  \"icm20948\": " + String(sensor_detection.icm20948_present ? "true" : "false") + ",\n";
  report += "  \"hmc5883l\": " + String(sensor_detection.hmc5883l_present ? "true" : "false") + ",\n";
  report += "  \"bmp280\": " + String(sensor_detection.bmp280_present ? "true" : "false") + "\n";
  report += "}";
  return report;
}

//-------------------------------------------------------------
// IMU & Magnetometer Calibration
//-------------------------------------------------------------

void calibrate_imu() {
  extern CalibrationData calibration_data;
  Serial.println("=== IMU calibration starting ===");
  Serial.println("Please keep the aircraft completely still...");

  const int SAMPLE_COUNT = 500;
  float gyro_sum[3]  = {0};
  float accel_sum[3] = {0};
  int collected = 0;

  while (collected < SAMPLE_COUNT) {
    IMUData d;
    if (read_imu_data(&d)) {
      gyro_sum[0]  += d.gyro_x;
      gyro_sum[1]  += d.gyro_y;
      gyro_sum[2]  += d.gyro_z;

      accel_sum[0] += d.accel_x;
      accel_sum[1] += d.accel_y;
      accel_sum[2] += d.accel_z;

      collected++;
    }
    delay(2); // ~500 Hz sampling
  }

  // Compute averages
  calibration_data.gyro_offsets[0] = gyro_sum[0] / SAMPLE_COUNT;
  calibration_data.gyro_offsets[1] = gyro_sum[1] / SAMPLE_COUNT;
  calibration_data.gyro_offsets[2] = gyro_sum[2] / SAMPLE_COUNT;

  calibration_data.accel_offsets[0] = accel_sum[0] / SAMPLE_COUNT;
  calibration_data.accel_offsets[1] = accel_sum[1] / SAMPLE_COUNT;
  calibration_data.accel_offsets[2] = (accel_sum[2] / SAMPLE_COUNT) - 9.81f; // remove 1 g

  // Identity scale matrices (no scaling yet).
  for (int i=0;i<3;i++) {
    calibration_data.accel_scale_matrix[i][0] = (i==0);
    calibration_data.accel_scale_matrix[i][1] = (i==1);
    calibration_data.accel_scale_matrix[i][2] = (i==2);
  }

  calibration_data.gyro_calibrated  = true;
  calibration_data.accel_calibrated = true;
  calibration_data.system_calibrated = calibration_data.mag_calibrated;
  calibration_data.calibration_timestamp = millis();

  CalibrationStorage::save(calibration_data);

  Serial.println("IMU calibration complete ✔");
}

void calibrate_magnetometer() {
  extern CalibrationData calibration_data;
  Serial.println("=== Magnetometer calibration ===");
  Serial.println("Move the aircraft in a slow figure-8 until told to stop...");

  const unsigned long CAL_DURATION_MS = 15000; // 15 s
  float mag_min[3] = {  1e6,  1e6,  1e6};
  float mag_max[3] = { -1e6, -1e6, -1e6};

  unsigned long start = millis();
  extern SensorData sensor_data;
  while (millis() - start < CAL_DURATION_MS) {
    MagnetometerData m = sensor_data.mag; // Use latest mag sample populated by update_sensors()
    if (m.healthy) {
      mag_min[0] = min(mag_min[0], m.mag_x);
      mag_min[1] = min(mag_min[1], m.mag_y);
      mag_min[2] = min(mag_min[2], m.mag_z);

      mag_max[0] = max(mag_max[0], m.mag_x);
      mag_max[1] = max(mag_max[1], m.mag_y);
      mag_max[2] = max(mag_max[2], m.mag_z);
    }
    delay(20);
  }

  // Offsets are mid-points
  for (int i=0;i<3;i++) {
    calibration_data.mag_offsets[i] = (mag_max[i] + mag_min[i]) / 2.0f;
  }

  // Simple soft-iron scale factors
  float delta[3] = { mag_max[0]-mag_min[0], mag_max[1]-mag_min[1], mag_max[2]-mag_min[2] };
  float avg_delta = (delta[0] + delta[1] + delta[2]) / 3.0f;
  for (int i=0;i<3;i++) {
    float scale = avg_delta / delta[i];
    calibration_data.mag_scale_matrix[i][0] = (i==0)?scale:0;
    calibration_data.mag_scale_matrix[i][1] = (i==1)?scale:0;
    calibration_data.mag_scale_matrix[i][2] = (i==2)?scale:0;
  }

  calibration_data.mag_calibrated = true;
  calibration_data.system_calibrated = calibration_data.gyro_calibrated && calibration_data.accel_calibrated;
  calibration_data.calibration_timestamp = millis();

  CalibrationStorage::save(calibration_data);

  Serial.println("Magnetometer calibration done ✔");
}

// Re-introduced minimal stubs required by main.ino
void init_enhanced_sensor_fusion() {
  Serial.println("Enhanced Sensor Fusion (Mahony) init");
  init_mahony_filter();
  lastFusionTime = millis();
}

void update_enhanced_sensor_fusion() {
  extern SensorData sensor_data; // from main .ino

  unsigned long now = millis();
  float dt = (now - lastFusionTime) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  lastFusionTime = now;

  // Use Mahony filter for attitude estimation
  extern void mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float dt);
  mahony_ahrs_update_imu(sensor_data.imu.gyro_x * DEG_TO_RAD, sensor_data.imu.gyro_y * DEG_TO_RAD, sensor_data.imu.gyro_z * DEG_TO_RAD,
                         sensor_data.imu.accel_x, sensor_data.imu.accel_y, sensor_data.imu.accel_z, dt);

  // convert quaternion to Euler
  extern float q0,q1,q2,q3;
  sensor_data.roll  = atan2f(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) * 180.0f/PI;
  sensor_data.pitch = asinf(2*(q0*q2 - q3*q1)) * 180.0f/PI;
  sensor_data.yaw   = atan2f(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) * 180.0f/PI;
}

void update_magnetometer_timer() {
  // Periodically apply calibration and (optionally) trigger recalibration if
  // the magnetometer is healthy but not yet calibrated.
  extern SensorData sensor_data;       // Provided by main .ino
  extern CalibrationData calibration_data;

  static unsigned long last_apply_time = 0;
  static unsigned long last_recal_attempt = 0;

  const unsigned long APPLY_INTERVAL_MS  = 50;        // Re-apply every 50 ms
  const unsigned long RECAL_INTERVAL_MS  = 10UL * 60UL * 1000UL; // Try to recal every 10 min

  unsigned long now = millis();

  // Re-apply calibration frequently so heading stays correct
  if (now - last_apply_time >= APPLY_INTERVAL_MS) {
    apply_magnetometer_calibration(&sensor_data.mag);
    last_apply_time = now;
  }

  // If we are missing a calibration but the sensor looks good, attempt one
  if (!calibration_data.mag_calibrated && sensor_data.mag.healthy &&
      now - last_recal_attempt >= 1000UL) {
    calibrate_magnetometer();
    last_recal_attempt = now;
  }

  // Optionally refresh calibration on a long interval (drift compensation)
  if (calibration_data.mag_calibrated &&
      now - last_recal_attempt >= RECAL_INTERVAL_MS) {
    // Here you could implement a quality check and only recal if needed.
    // For now we just schedule a future recal attempt flag.
    last_recal_attempt = now;
  }
}

#if 0
bool perform_safety_checks() {
  return enhanced_safety_check_for_arming_with_feedback();
}
#endif

void mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  // Normalise accelerometer measurement
  float recipNorm = sqrtf(ax*ax + ay*ay + az*az);
  if(recipNorm <= 0.0f) return; // invalid accel
  recipNorm = 1.0f/recipNorm;
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  // Estimated direction of gravity
  float vx = 2*(q1*q3 - q0*q2);
  float vy = 2*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Error is cross product between estimated and measured gravity
  float ex = (ay*vz - az*vy);
  float ey = (az*vx - ax*vz);
  float ez = (ax*vy - ay*vx);

  // Integral (optional)
  integralFBx += MAHONY_TWO_KI* ex * dt;
  integralFBy += MAHONY_TWO_KI* ey * dt;
  integralFBz += MAHONY_TWO_KI* ez * dt;

  // Apply feedback
  gx += MAHONY_TWO_KP*ex + integralFBx;
  gy += MAHONY_TWO_KP*ey + integralFBy;
  gz += MAHONY_TWO_KP*ez + integralFBz;

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
  float qa=q0, qb=q1, qc=q2;
  q0 += (-qb*gx - qc*gy - q3*gz);
  q1 += (qa*gx + qc*gz - q3*gy);
  q2 += (qa*gy - qb*gz + q3*gx);
  q3 += (qa*gz + qb*gy - qc*gx);

  // Normalise quaternion
  recipNorm = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
  recipNorm = 1.0f/recipNorm;
  q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;
}

#if 0
void load_config() {
  extern CalibrationData calibration_data;
  if (CalibrationStorage::load(calibration_data)) {
    Serial.println("Calibration data loaded ✔");
  } else {
    Serial.println("No valid calibration data found, using defaults");
    memset(&calibration_data, 0, sizeof(CalibrationData));
    calibration_data.system_calibrated = false;
  }
}
#endif

// -------------------------------------------------
// Generic I2C presence helper
static bool i2c_device_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Magnetometer detection functions
bool detect_hmc5883l(uint8_t address) { return i2c_device_present(address); }
bool detect_qmc5883l(uint8_t address) { return i2c_device_present(address); }
bool detect_rm3100(uint8_t address)   { return i2c_device_present(address); }
bool detect_lis3mdl(uint8_t address)  { return i2c_device_present(address); }

// Barometer detection functions
bool detect_bmp280(uint8_t address)   { return i2c_device_present(address); }
bool detect_bmp388(uint8_t address)   { return i2c_device_present(address); }
bool detect_ms5611(uint8_t address)   { return i2c_device_present(address); }

// Provide weak fallbacks for others referenced in headers but unused
bool detect_bme280(uint8_t address) __attribute__((weak));
bool detect_bme280(uint8_t address) { return i2c_device_present(address); }
bool detect_lps22hb(uint8_t address) __attribute__((weak));
bool detect_lps22hb(uint8_t address) { return i2c_device_present(address); } 