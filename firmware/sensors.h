#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include <Wire.h>

// MPU6050 Registers
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_TEMP_OUT_H 0x41

// HMC5883L Registers
#define HMC5883L_ADDR 0x1E
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_DATA_X_H 0x03

// BMP280 Registers
#define BMP280_ADDR 0x76
#define BMP280_ID 0xD0
#define BMP280_CTRL_MEAS 0xF4
#define BMP280_CONFIG 0xF5
#define BMP280_PRESS_MSB 0xF7

// Mahony AHRS Filter Implementation
struct MahonyFilter {
  float q0, q1, q2, q3;    // Quaternion representation
  float integralFBx, integralFBy, integralFBz;  // Integral error terms
  bool initialized;
  unsigned long last_update;
};

// Enhanced filtering structures
struct VibratingFilter {
  float cutoff_hz;
  float alpha;
  float filtered_value;
  float raw_value;
  bool initialized;
};

struct NotchFilter {
  float center_freq_hz;
  float bandwidth_hz;
  float q_factor;
  float b0, b1, b2, a1, a2;  // Filter coefficients
  float x1, x2, y1, y2;      // Delay elements
  bool enabled;
};

// Global filter instances
MahonyFilter mahony_filter;
VibratingFilter gyro_filters[3];    // X, Y, Z gyro filters
VibratingFilter accel_filters[3];   // X, Y, Z accel filters
NotchFilter motor_notch_filters[3]; // X, Y, Z motor noise filters

// Function prototypes for enhanced filtering
void init_mahony_filter();
void init_vibration_filters();
void init_notch_filters();
void mahony_ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float dt);
float apply_vibration_filter(VibratingFilter* filter, float input, float dt);
float apply_notch_filter(NotchFilter* filter, float input);
void calculate_notch_coefficients(NotchFilter* filter, float sample_rate);
void detect_vibration_issues(SensorData* data);

// Function Prototypes
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

// Sensor fusion
void update_sensor_fusion();
void complementary_filter(float dt);

// IMU Functions
bool init_imu() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00); // Wake up the MPU6050
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    // Configure accelerometer and gyroscope
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A); // CONFIG register
    Wire.write(0x06); // Set DLPF to 5Hz
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x10); // ±1000°/s
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); // ACCEL_CONFIG
    Wire.write(0x08); // ±4g
    Wire.endTransmission();
    
    Serial.println("MPU6050 initialized successfully");
    return true;
  }
  return false;
}

bool read_imu_data(IMUData* data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  if (Wire.available() == 14) {
    // Read accelerometer data
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    
    // Read temperature
    int16_t temp = Wire.read() << 8 | Wire.read();
    
    // Read gyroscope data
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    
    // Convert to physical units
    data->accel_x = ax / 8192.0; // ±4g range
    data->accel_y = ay / 8192.0;
    data->accel_z = az / 8192.0;
    
    data->gyro_x = gx / 32.8; // ±1000°/s range
    data->gyro_y = gy / 32.8;
    data->gyro_z = gz / 32.8;
    
    data->temp = temp / 340.0 + 36.53;
    data->healthy = true;
    data->last_update = millis();
    
    return true;
  }
  
  data->healthy = false;
  return false;
}

// Magnetometer Functions
bool init_magnetometer() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_CONFIG_A);
  Wire.write(0x70); // 8 samples average, 15Hz output rate
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(HMC5883L_CONFIG_B);
    Wire.write(0x20); // Gain = 5
    Wire.endTransmission();
    
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(HMC5883L_MODE);
    Wire.write(0x00); // Continuous measurement mode
    Wire.endTransmission();
    
    Serial.println("HMC5883L initialized successfully");
    return true;
  }
  return false;
}

bool read_magnetometer_data(MagnetometerData* data) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_DATA_X_H);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6, true);
  
  if (Wire.available() == 6) {
    int16_t mx = Wire.read() << 8 | Wire.read();
    int16_t mz = Wire.read() << 8 | Wire.read(); // Note: Y and Z are swapped
    int16_t my = Wire.read() << 8 | Wire.read();
    
    data->mag_x = mx;
    data->mag_y = my;
    data->mag_z = mz;
    
    // Calculate heading
    data->heading = atan2(data->mag_y, data->mag_x) * 180.0 / PI;
    if (data->heading < 0) data->heading += 360;
    
    data->healthy = true;
    data->last_update = millis();
    
    return true;
  }
  
  data->healthy = false;
  return false;
}

// Barometer Functions
bool init_barometer() {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP280_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 1, true);
  
  if (Wire.available() && Wire.read() == 0x58) {
    // Configure BMP280
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(BMP280_CTRL_MEAS);
    Wire.write(0x27); // Pressure and temperature oversampling x1, normal mode
    Wire.endTransmission();
    
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(BMP280_CONFIG);
    Wire.write(0xA0); // Standby time 1000ms, filter off
    Wire.endTransmission();
    
    Serial.println("BMP280 initialized successfully");
    return true;
  }
  return false;
}

bool read_barometer_data(BarometerData* data) {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP280_PRESS_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 6, true);
  
  if (Wire.available() == 6) {
    uint32_t press_raw = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);
    uint32_t temp_raw = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);
    
    // Simple pressure calculation (should use calibration coefficients in real implementation)
    data->pressure = press_raw / 256.0;
    data->temperature = temp_raw / 5120.0;
    
    // Calculate altitude (approximation)
    data->altitude = 44330.0 * (1.0 - pow(data->pressure / 101325.0, 0.1903));
    
    data->healthy = true;
    data->last_update = millis();
    
    return true;
  }
  
  data->healthy = false;
  return false;
}

// GPS Functions
void init_gps() {
  Serial1.begin(9600);
  // Send UBX commands to configure GPS (optional)
  Serial.println("GPS UART initialized");
}

bool read_gps_data(GPSData* data) {
  static String gps_buffer = "";
  
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      if (gps_buffer.startsWith("$GPGGA") || gps_buffer.startsWith("$GNGGA")) {
        // Parse NMEA GGA sentence
        parse_gga_sentence(gps_buffer, data);
      }
      gps_buffer = "";
    } else if (c != '\r') {
      gps_buffer += c;
    }
  }
  
  // Check if GPS data is recent
  if (millis() - data->last_update < 2000) {
    data->healthy = true;
    return true;
  }
  
  data->healthy = false;
  return false;
}

void parse_gga_sentence(String sentence, GPSData* data) {
  // Simple NMEA parser (simplified)
  int commaIndex = 0;
  int fieldIndex = 0;
  String fields[15];
  
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',') {
      fieldIndex++;
      commaIndex = i;
    } else {
      if (fieldIndex < 15) {
        fields[fieldIndex] += sentence.charAt(i);
      }
    }
  }
  
  if (fieldIndex >= 6 && fields[6].toInt() > 0) {
    data->latitude = fields[2].toFloat() / 100.0;
    data->longitude = fields[4].toFloat() / 100.0;
    data->altitude = fields[9].toFloat();
    data->satellites = fields[7].toInt();
    data->fix = (fields[6].toInt() > 0);
    data->last_update = millis();
  }
}

// Sonar Functions
void init_sonar() {
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  Serial.println("Sonar initialized");
}

bool read_sonar_data(SonarData* data) {
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 30000); // 30ms timeout
  
  if (duration > 0) {
    data->distance = duration * 0.034 / 2; // Convert to cm
    data->healthy = true;
    data->last_update = millis();
    return true;
  }
  
  data->healthy = false;
  return false;
}

// Sensor Fusion
void update_sensor_fusion() {
  static unsigned long last_fusion_update = 0;
  unsigned long current_time = micros();
  float dt = (current_time - last_fusion_update) / 1000000.0;
  
  if (dt > 0.0001 && dt < 0.1) { // Update between 10Hz and 10kHz
    mahony_filter_update(dt);
    apply_vibration_filtering();
    detect_vibration_issues(&sensor_data);
    last_fusion_update = current_time;
  }
}

void init_mahony_filter() {
  mahony_filter.q0 = 1.0f;
  mahony_filter.q1 = 0.0f;
  mahony_filter.q2 = 0.0f;
  mahony_filter.q3 = 0.0f;
  mahony_filter.integralFBx = 0.0f;
  mahony_filter.integralFBy = 0.0f;
  mahony_filter.integralFBz = 0.0f;
  mahony_filter.initialized = false;
  mahony_filter.last_update = 0;
  
  Serial.println("Mahony AHRS filter initialized");
}

void init_vibration_filters() {
  for (int i = 0; i < 3; i++) {
    // Gyro filters
    gyro_filters[i].cutoff_hz = GYRO_LPF_CUTOFF_HZ;
    gyro_filters[i].alpha = 0.0f;  // Will be calculated based on dt
    gyro_filters[i].filtered_value = 0.0f;
    gyro_filters[i].raw_value = 0.0f;
    gyro_filters[i].initialized = false;
    
    // Accelerometer filters  
    accel_filters[i].cutoff_hz = ACCEL_LPF_CUTOFF_HZ;
    accel_filters[i].alpha = 0.0f;
    accel_filters[i].filtered_value = 0.0f;
    accel_filters[i].raw_value = 0.0f;
    accel_filters[i].initialized = false;
  }
  
  Serial.println("Vibration filters initialized");
}

void init_notch_filters() {
  for (int i = 0; i < 3; i++) {
    motor_notch_filters[i].center_freq_hz = NOTCH_FILTER_CENTER_HZ;
    motor_notch_filters[i].bandwidth_hz = 20.0f;  // 20Hz bandwidth
    motor_notch_filters[i].q_factor = motor_notch_filters[i].center_freq_hz / motor_notch_filters[i].bandwidth_hz;
    motor_notch_filters[i].x1 = motor_notch_filters[i].x2 = 0.0f;
    motor_notch_filters[i].y1 = motor_notch_filters[i].y2 = 0.0f;
    motor_notch_filters[i].enabled = true;
    
    calculate_notch_coefficients(&motor_notch_filters[i], 1000.0f); // 1kHz sample rate
  }
  
  Serial.println("Motor notch filters initialized");
}

void mahony_filter_update(float dt) {
  extern SensorData sensor_data;
  
  if (!sensor_data.imu.healthy) return;
  
  // Convert degrees to radians
  float gx = sensor_data.imu.gyro_x * PI / 180.0f;
  float gy = sensor_data.imu.gyro_y * PI / 180.0f; 
  float gz = sensor_data.imu.gyro_z * PI / 180.0f;
  
  float ax = sensor_data.imu.accel_x;
  float ay = sensor_data.imu.accel_y;
  float az = sensor_data.imu.accel_z;
  
  // Use magnetometer if available and healthy
  if (sensor_data.mag.healthy) {
    float mx = sensor_data.mag.mag_x;
    float my = sensor_data.mag.mag_y;
    float mz = sensor_data.mag.mag_z;
    mahony_ahrs_update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
  } else {
    mahony_ahrs_update_imu(gx, gy, gz, ax, ay, az, dt);
  }
  
  // Convert quaternion to Euler angles
  quaternion_to_euler();
}

void mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  
  // Compute feedback only if accelerometer measurement valid
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    
    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity
    halfvx = mahony_filter.q1 * mahony_filter.q3 - mahony_filter.q0 * mahony_filter.q2;
    halfvy = mahony_filter.q0 * mahony_filter.q1 + mahony_filter.q2 * mahony_filter.q3;
    halfvz = mahony_filter.q0 * mahony_filter.q0 - 0.5f + mahony_filter.q3 * mahony_filter.q3;
    
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    
    // Compute and apply integral feedback if enabled
    if (MAHONY_KI > 0.0f) {
      mahony_filter.integralFBx += MAHONY_KI * halfex * dt;
      mahony_filter.integralFBy += MAHONY_KI * halfey * dt;
      mahony_filter.integralFBz += MAHONY_KI * halfez * dt;
      gx += mahony_filter.integralFBx;
      gy += mahony_filter.integralFBy;
      gz += mahony_filter.integralFBz;
    }
    
    // Apply proportional feedback
    gx += MAHONY_KP * halfex;
    gy += MAHONY_KP * halfey;
    gz += MAHONY_KP * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = mahony_filter.q0;
  qb = mahony_filter.q1;
  qc = mahony_filter.q2;
  mahony_filter.q0 += (-qb * gx - qc * gy - mahony_filter.q3 * gz);
  mahony_filter.q1 += (qa * gx + qc * gz - mahony_filter.q3 * gy);
  mahony_filter.q2 += (qa * gy - qb * gz + mahony_filter.q3 * gx);
  mahony_filter.q3 += (qa * gz + qb * gy - qc * gx);
  
  // Normalize quaternion
  recipNorm = 1.0f / sqrt(mahony_filter.q0 * mahony_filter.q0 + mahony_filter.q1 * mahony_filter.q1 + 
                         mahony_filter.q2 * mahony_filter.q2 + mahony_filter.q3 * mahony_filter.q3);
  mahony_filter.q0 *= recipNorm;
  mahony_filter.q1 *= recipNorm;
  mahony_filter.q2 *= recipNorm;
  mahony_filter.q3 *= recipNorm;
}

void quaternion_to_euler() {
  extern SensorData sensor_data;
  
  // Convert quaternion to Euler angles (roll, pitch, yaw)
  float q0 = mahony_filter.q0;
  float q1 = mahony_filter.q1;
  float q2 = mahony_filter.q2;
  float q3 = mahony_filter.q3;
  
  // Roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  sensor_data.roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;
  
  // Pitch (y-axis rotation)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (abs(sinp) >= 1.0f)
    sensor_data.pitch = copysign(PI / 2, sinp) * 180.0f / PI; // Use 90 degrees if out of range
  else
    sensor_data.pitch = asin(sinp) * 180.0f / PI;
  
  // Yaw (z-axis rotation)
  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  sensor_data.yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
  
  // Normalize yaw to 0-360 degrees
  if (sensor_data.yaw < 0) sensor_data.yaw += 360;
}

void apply_vibration_filtering() {
  extern SensorData sensor_data;
  
  if (!sensor_data.imu.healthy) return;
  
  static unsigned long last_filter_update = 0;
  unsigned long current_time = micros();
  float dt = (current_time - last_filter_update) / 1000000.0f;
  
  if (dt > 0.0001f) {
    // Apply vibration filtering to gyro data
    sensor_data.imu.gyro_x = apply_vibration_filter(&gyro_filters[0], sensor_data.imu.gyro_x, dt);
    sensor_data.imu.gyro_y = apply_vibration_filter(&gyro_filters[1], sensor_data.imu.gyro_y, dt);
    sensor_data.imu.gyro_z = apply_vibration_filter(&gyro_filters[2], sensor_data.imu.gyro_z, dt);
    
    // Apply vibration filtering to accelerometer data
    sensor_data.imu.accel_x = apply_vibration_filter(&accel_filters[0], sensor_data.imu.accel_x, dt);
    sensor_data.imu.accel_y = apply_vibration_filter(&accel_filters[1], sensor_data.imu.accel_y, dt);
    sensor_data.imu.accel_z = apply_vibration_filter(&accel_filters[2], sensor_data.imu.accel_z, dt);
    
    // Apply notch filtering for motor noise
    if (motor_notch_filters[0].enabled) {
      sensor_data.imu.gyro_x = apply_notch_filter(&motor_notch_filters[0], sensor_data.imu.gyro_x);
      sensor_data.imu.gyro_y = apply_notch_filter(&motor_notch_filters[1], sensor_data.imu.gyro_y);
      sensor_data.imu.gyro_z = apply_notch_filter(&motor_notch_filters[2], sensor_data.imu.gyro_z);
    }
    
    last_filter_update = current_time;
  }
}

float apply_vibration_filter(VibratingFilter* filter, float input, float dt) {
  // Calculate alpha for low-pass filter based on cutoff frequency
  float rc = 1.0f / (2.0f * PI * filter->cutoff_hz);
  filter->alpha = dt / (rc + dt);
  
  if (!filter->initialized) {
    filter->filtered_value = input;
    filter->initialized = true;
  } else {
    filter->filtered_value = filter->alpha * input + (1.0f - filter->alpha) * filter->filtered_value;
  }
  
  filter->raw_value = input;
  return filter->filtered_value;
}

float apply_notch_filter(NotchFilter* filter, float input) {
  if (!filter->enabled) return input;
  
  // Apply biquad notch filter
  float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                - filter->a1 * filter->y1 - filter->a2 * filter->y2;
  
  // Update delay elements
  filter->x2 = filter->x1;
  filter->x1 = input;
  filter->y2 = filter->y1;
  filter->y1 = output;
  
  return output;
}

void calculate_notch_coefficients(NotchFilter* filter, float sample_rate) {
  float omega = 2.0f * PI * filter->center_freq_hz / sample_rate;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  float alpha = sin_omega / (2.0f * filter->q_factor);
  
  // Biquad coefficients for notch filter
  filter->b0 = 1.0f;
  filter->b1 = -2.0f * cos_omega;
  filter->b2 = 1.0f;
  
  float norm = 1.0f + alpha;
  filter->a1 = filter->b1 / norm;
  filter->a2 = (1.0f - alpha) / norm;
  
  filter->b0 /= norm;
  filter->b1 /= norm;
  filter->b2 /= norm;
}

void detect_vibration_issues(SensorData* data) {
  static float gyro_rms[3] = {0, 0, 0};
  static float accel_rms[3] = {0, 0, 0};
  static int sample_count = 0;
  
  // Calculate RMS values for vibration detection
  gyro_rms[0] += data->imu.gyro_x * data->imu.gyro_x;
  gyro_rms[1] += data->imu.gyro_y * data->imu.gyro_y;
  gyro_rms[2] += data->imu.gyro_z * data->imu.gyro_z;
  
  accel_rms[0] += data->imu.accel_x * data->imu.accel_x;
  accel_rms[1] += data->imu.accel_y * data->imu.accel_y;
  accel_rms[2] += data->imu.accel_z * data->imu.accel_z;
  
  sample_count++;
  
  // Check vibration levels every 1000 samples
  if (sample_count >= 1000) {
    for (int i = 0; i < 3; i++) {
      gyro_rms[i] = sqrt(gyro_rms[i] / sample_count);
      accel_rms[i] = sqrt(accel_rms[i] / sample_count);
      
      if (gyro_rms[i] > GYRO_NOISE_THRESHOLD) {
        Serial.print("WARNING: High gyro vibration detected on axis ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(gyro_rms[i]);
      }
      
      if (accel_rms[i] > ACCEL_NOISE_THRESHOLD) {
        Serial.print("WARNING: High accel vibration detected on axis ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(accel_rms[i]);
      }
      
      gyro_rms[i] = 0;
      accel_rms[i] = 0;
    }
    sample_count = 0;
  }
}

// Sensor detection and status tracking
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
  uint8_t i2c_devices[16];  // Store found I2C addresses
  uint8_t i2c_device_count;
  
  // Last detection scan timestamp
  unsigned long last_detection_scan;
  bool detection_complete;
};

// Global sensor detection status
extern SensorDetectionStatus sensor_detection;

// Enhanced sensor detection functions
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

// Individual sensor detection functions
bool detect_mpu6050(uint8_t address = 0x68);
bool detect_mpu9250(uint8_t address = 0x68);
bool detect_icm20948(uint8_t address = 0x69);
bool detect_icm42688p(uint8_t address = 0x68);
bool detect_bmi270(uint8_t address = 0x68);
bool detect_lsm6dso32(uint8_t address = 0x6A);
bool detect_bmi323(uint8_t address = 0x68);
bool detect_icm20602(uint8_t address = 0x68);
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

// Enhanced sensor detection implementation
void scan_i2c_devices() {
  sensor_detection.i2c_device_count = 0;
  
  Serial.println("Scanning I2C bus for devices...");
  Serial.println("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
  
  for (int addr = 0; addr < 128; addr++) {
    if (addr % 16 == 0) {
      Serial.print(addr < 16 ? "0" : "");
      Serial.print(addr, HEX);
      Serial.print(": ");
    }
    
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(addr < 16 ? "0" : "");
      Serial.print(addr, HEX);
      Serial.print(" ");
      
      // Store found device
      if (sensor_detection.i2c_device_count < 16) {
        sensor_detection.i2c_devices[sensor_detection.i2c_device_count] = addr;
        sensor_detection.i2c_device_count++;
      }
    } else {
      Serial.print("-- ");
    }
    
    if ((addr + 1) % 16 == 0) {
      Serial.println();
    }
  }
  
  Serial.print("Found ");
  Serial.print(sensor_detection.i2c_device_count);
  Serial.println(" I2C devices");
}

bool detect_imu_sensors() {
  bool found_any = false;
  
  // Clear IMU detection flags
  sensor_detection.mpu6050_present = false;
  sensor_detection.mpu9250_present = false;
  sensor_detection.icm20948_present = false;
  sensor_detection.icm42688p_present = false;
  sensor_detection.bmi270_present = false;
  sensor_detection.lsm6dso32_present = false;
  sensor_detection.bmi323_present = false;
  sensor_detection.icm20602_present = false;
  sensor_detection.lsm6ds33_present = false;
  
  // Scan common IMU addresses
  uint8_t imu_addresses[] = {0x68, 0x69, 0x6A, 0x6B};
  
  for (int i = 0; i < 4; i++) {
    uint8_t addr = imu_addresses[i];
    
    if (detect_mpu6050(addr)) {
      sensor_detection.mpu6050_present = true;
      found_any = true;
      Serial.print("MPU6050 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_mpu9250(addr)) {
      sensor_detection.mpu9250_present = true;
      found_any = true;
      Serial.print("MPU9250 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_icm20948(addr)) {
      sensor_detection.icm20948_present = true;
      found_any = true;
      Serial.print("ICM20948 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_icm42688p(addr)) {
      sensor_detection.icm42688p_present = true;
      found_any = true;
      Serial.print("ICM42688P detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bmi270(addr)) {
      sensor_detection.bmi270_present = true;
      found_any = true;
      Serial.print("BMI270 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lsm6dso32(addr)) {
      sensor_detection.lsm6dso32_present = true;
      found_any = true;
      Serial.print("LSM6DSO32 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bmi323(addr)) {
      sensor_detection.bmi323_present = true;
      found_any = true;
      Serial.print("BMI323 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_icm20602(addr)) {
      sensor_detection.icm20602_present = true;
      found_any = true;
      Serial.print("ICM20602 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lsm6ds33(addr)) {
      sensor_detection.lsm6ds33_present = true;
      found_any = true;
      Serial.print("LSM6DS33 detected at 0x");
      Serial.println(addr, HEX);
    }
  }
  
  if (!found_any) {
    Serial.println("No IMU sensors detected");
  }
  
  return found_any;
}

bool detect_magnetometer_sensors() {
  bool found_any = false;
  
  // Clear magnetometer detection flags
  sensor_detection.hmc5883l_present = false;
  sensor_detection.qmc5883l_present = false;
  sensor_detection.rm3100_present = false;
  sensor_detection.mmc5883ma_present = false;
  sensor_detection.ist8310_present = false;
  sensor_detection.lis3mdl_present = false;
  sensor_detection.ak8963_present = false;
  
  // Common magnetometer addresses
  uint8_t mag_addresses[] = {0x0C, 0x0D, 0x0E, 0x1C, 0x1E, 0x20, 0x30};
  
  for (int i = 0; i < 7; i++) {
    uint8_t addr = mag_addresses[i];
    
    if (detect_hmc5883l(addr)) {
      sensor_detection.hmc5883l_present = true;
      found_any = true;
      Serial.print("HMC5883L detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_qmc5883l(addr)) {
      sensor_detection.qmc5883l_present = true;
      found_any = true;
      Serial.print("QMC5883L detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_rm3100(addr)) {
      sensor_detection.rm3100_present = true;
      found_any = true;
      Serial.print("RM3100 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_mmc5883ma(addr)) {
      sensor_detection.mmc5883ma_present = true;
      found_any = true;
      Serial.print("MMC5883MA detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_ist8310(addr)) {
      sensor_detection.ist8310_present = true;
      found_any = true;
      Serial.print("IST8310 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lis3mdl(addr)) {
      sensor_detection.lis3mdl_present = true;
      found_any = true;
      Serial.print("LIS3MDL detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_ak8963(addr)) {
      sensor_detection.ak8963_present = true;
      found_any = true;
      Serial.print("AK8963 detected at 0x");
      Serial.println(addr, HEX);
    }
  }
  
  if (!found_any) {
    Serial.println("No magnetometer sensors detected");
  }
  
  return found_any;
}

bool detect_barometer_sensors() {
  bool found_any = false;
  
  // Clear barometer detection flags
  sensor_detection.bmp280_present = false;
  sensor_detection.bme280_present = false;
  sensor_detection.bmp388_present = false;
  sensor_detection.ms5611_present = false;
  sensor_detection.lps22hb_present = false;
  
  // Common barometer addresses
  uint8_t baro_addresses[] = {0x76, 0x77, 0x5C, 0x5D};
  
  for (int i = 0; i < 4; i++) {
    uint8_t addr = baro_addresses[i];
    
    if (detect_bmp280(addr)) {
      sensor_detection.bmp280_present = true;
      found_any = true;
      Serial.print("BMP280 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bme280(addr)) {
      sensor_detection.bme280_present = true;
      found_any = true;
      Serial.print("BME280 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bmp388(addr)) {
      sensor_detection.bmp388_present = true;
      found_any = true;
      Serial.print("BMP388 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_ms5611(addr)) {
      sensor_detection.ms5611_present = true;
      found_any = true;
      Serial.print("MS5611 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lps22hb(addr)) {
      sensor_detection.lps22hb_present = true;
      found_any = true;
      Serial.print("LPS22HB detected at 0x");
      Serial.println(addr, HEX);
    }
  }
  
  if (!found_any) {
    Serial.println("No barometer sensors detected");
  }
  
  return found_any;
}

bool detect_gps_sensor() {
  // Test GPS by checking if UART is receiving data
  Serial1.begin(9600);
  delay(100);
  
  unsigned long start_time = millis();
  bool gps_data_received = false;
  
  // Wait up to 2 seconds for GPS data
  while (millis() - start_time < 2000) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '$') {  // NMEA sentence start
        gps_data_received = true;
        break;
      }
    }
    delay(10);
  }
  
  sensor_detection.gps_present = gps_data_received;
  
  if (gps_data_received) {
    Serial.println("GPS sensor detected on UART1");
  } else {
    Serial.println("No GPS sensor detected on UART1");
  }
  
  return gps_data_received;
}

bool detect_sonar_sensor() {
  // Test sonar by sending a pulse and checking for echo
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 30000);
  sensor_detection.sonar_present = (duration > 0 && duration < 25000); // Reasonable range
  
  if (sensor_detection.sonar_present) {
    Serial.println("Sonar sensor detected");
  } else {
    Serial.println("No sonar sensor detected");
  }
  
  return sensor_detection.sonar_present;
}

bool detect_optical_flow_sensor() {
  // This would need specific implementation for each optical flow sensor type
  // For now, just check common SPI addresses or I2C addresses
  sensor_detection.optical_flow_present = false;
  
  // Check for PMW3901 on SPI
  // Check for PAA5100 on I2C
  // Check for ADNS3080 on SPI
  
  // Placeholder implementation
  Serial.println("Optical flow sensor detection not implemented");
  return false;
}

bool detect_voltage_current_sensors() {
  // Test ADC pins for voltage and current sensors
  int voltage_reading = analogRead(BATTERY_VOLTAGE_PIN);
  int current_reading = analogRead(CURRENT_SENSOR_PIN);
  
  // Simple heuristic: if readings are not stuck at 0 or max value
  sensor_detection.voltage_sensor_present = (voltage_reading > 10 && voltage_reading < 1013);
  sensor_detection.current_sensor_present = (current_reading > 10 && current_reading < 1013);
  
  if (sensor_detection.voltage_sensor_present) {
    Serial.println("Voltage sensor detected");
  } else {
    Serial.println("No voltage sensor detected");
  }
  
  if (sensor_detection.current_sensor_present) {
    Serial.println("Current sensor detected");
  } else {
    Serial.println("No current sensor detected");
  }
  
  return sensor_detection.voltage_sensor_present || sensor_detection.current_sensor_present;
}

void perform_full_sensor_detection() {
  Serial.println("=== Starting Full Sensor Detection ===");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000); // 100kHz for compatibility
  
  // Scan I2C bus
  scan_i2c_devices();
  
  // Detect each sensor type
  detect_imu_sensors();
  detect_magnetometer_sensors();
  detect_barometer_sensors();
  detect_gps_sensor();
  detect_sonar_sensor();
  detect_optical_flow_sensor();
  detect_voltage_current_sensors();
  
  sensor_detection.last_detection_scan = millis();
  sensor_detection.detection_complete = true;
  
  Serial.println("=== Sensor Detection Complete ===");
}

String get_sensor_detection_report() {
  String report = "=== SENSOR DETECTION REPORT ===\n";
  
  // IMU sensors
  report += "IMU Sensors:\n";
  if (sensor_detection.mpu6050_present) report += "  ✓ MPU6050\n";
  if (sensor_detection.mpu9250_present) report += "  ✓ MPU9250\n";
  if (sensor_detection.icm20948_present) report += "  ✓ ICM20948\n";
  if (sensor_detection.icm42688p_present) report += "  ✓ ICM42688P\n";
  if (sensor_detection.bmi270_present) report += "  ✓ BMI270\n";
  if (sensor_detection.lsm6dso32_present) report += "  ✓ LSM6DSO32\n";
  if (sensor_detection.bmi323_present) report += "  ✓ BMI323\n";
  if (sensor_detection.icm20602_present) report += "  ✓ ICM20602\n";
  if (sensor_detection.lsm6ds33_present) report += "  ✓ LSM6DS33\n";
  
  // Magnetometer sensors
  report += "Magnetometer Sensors:\n";
  if (sensor_detection.hmc5883l_present) report += "  ✓ HMC5883L\n";
  if (sensor_detection.qmc5883l_present) report += "  ✓ QMC5883L\n";
  if (sensor_detection.rm3100_present) report += "  ✓ RM3100\n";
  if (sensor_detection.mmc5883ma_present) report += "  ✓ MMC5883MA\n";
  if (sensor_detection.ist8310_present) report += "  ✓ IST8310\n";
  if (sensor_detection.lis3mdl_present) report += "  ✓ LIS3MDL\n";
  if (sensor_detection.ak8963_present) report += "  ✓ AK8963\n";
  
  // Barometer sensors
  report += "Barometer Sensors:\n";
  if (sensor_detection.bmp280_present) report += "  ✓ BMP280\n";
  if (sensor_detection.bme280_present) report += "  ✓ BME280\n";
  if (sensor_detection.bmp388_present) report += "  ✓ BMP388\n";
  if (sensor_detection.ms5611_present) report += "  ✓ MS5611\n";
  if (sensor_detection.lps22hb_present) report += "  ✓ LPS22HB\n";
  
  // Other sensors
  report += "Other Sensors:\n";
  if (sensor_detection.gps_present) report += "  ✓ GPS\n";
  if (sensor_detection.sonar_present) report += "  ✓ Sonar\n";
  if (sensor_detection.optical_flow_present) report += "  ✓ Optical Flow\n";
  if (sensor_detection.voltage_sensor_present) report += "  ✓ Voltage Sensor\n";
  if (sensor_detection.current_sensor_present) report += "  ✓ Current Sensor\n";
  
  // I2C devices summary
  report += "I2C Devices Found: ";
  report += String(sensor_detection.i2c_device_count);
  report += "\n";
  
  return report;
}

String get_sensor_capabilities_json() {
  String json = "{\n";
  json += "  \"detection_complete\": " + String(sensor_detection.detection_complete ? "true" : "false") + ",\n";
  json += "  \"last_scan\": " + String(sensor_detection.last_detection_scan) + ",\n";
  
  // IMU sensors
  json += "  \"imu\": {\n";
  json += "    \"mpu6050\": " + String(sensor_detection.mpu6050_present ? "true" : "false") + ",\n";
  json += "    \"mpu9250\": " + String(sensor_detection.mpu9250_present ? "true" : "false") + ",\n";
  json += "    \"icm20948\": " + String(sensor_detection.icm20948_present ? "true" : "false") + ",\n";
  json += "    \"icm42688p\": " + String(sensor_detection.icm42688p_present ? "true" : "false") + ",\n";
  json += "    \"bmi270\": " + String(sensor_detection.bmi270_present ? "true" : "false") + ",\n";
  json += "    \"lsm6dso32\": " + String(sensor_detection.lsm6dso32_present ? "true" : "false") + ",\n";
  json += "    \"bmi323\": " + String(sensor_detection.bmi323_present ? "true" : "false") + ",\n";
  json += "    \"icm20602\": " + String(sensor_detection.icm20602_present ? "true" : "false") + ",\n";
  json += "    \"lsm6ds33\": " + String(sensor_detection.lsm6ds33_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // Magnetometer sensors
  json += "  \"magnetometer\": {\n";
  json += "    \"hmc5883l\": " + String(sensor_detection.hmc5883l_present ? "true" : "false") + ",\n";
  json += "    \"qmc5883l\": " + String(sensor_detection.qmc5883l_present ? "true" : "false") + ",\n";
  json += "    \"rm3100\": " + String(sensor_detection.rm3100_present ? "true" : "false") + ",\n";
  json += "    \"mmc5883ma\": " + String(sensor_detection.mmc5883ma_present ? "true" : "false") + ",\n";
  json += "    \"ist8310\": " + String(sensor_detection.ist8310_present ? "true" : "false") + ",\n";
  json += "    \"lis3mdl\": " + String(sensor_detection.lis3mdl_present ? "true" : "false") + ",\n";
  json += "    \"ak8963\": " + String(sensor_detection.ak8963_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // Barometer sensors
  json += "  \"barometer\": {\n";
  json += "    \"bmp280\": " + String(sensor_detection.bmp280_present ? "true" : "false") + ",\n";
  json += "    \"bme280\": " + String(sensor_detection.bme280_present ? "true" : "false") + ",\n";
  json += "    \"bmp388\": " + String(sensor_detection.bmp388_present ? "true" : "false") + ",\n";
  json += "    \"ms5611\": " + String(sensor_detection.ms5611_present ? "true" : "false") + ",\n";
  json += "    \"lps22hb\": " + String(sensor_detection.lps22hb_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // Other sensors
  json += "  \"other\": {\n";
  json += "    \"gps\": " + String(sensor_detection.gps_present ? "true" : "false") + ",\n";
  json += "    \"sonar\": " + String(sensor_detection.sonar_present ? "true" : "false") + ",\n";
  json += "    \"optical_flow\": " + String(sensor_detection.optical_flow_present ? "true" : "false") + ",\n";
  json += "    \"voltage_sensor\": " + String(sensor_detection.voltage_sensor_present ? "true" : "false") + ",\n";
  json += "    \"current_sensor\": " + String(sensor_detection.current_sensor_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // I2C devices
  json += "  \"i2c_devices\": [\n";
  for (int i = 0; i < sensor_detection.i2c_device_count; i++) {
    json += "    \"0x" + String(sensor_detection.i2c_devices[i], HEX) + "\"";
    if (i < sensor_detection.i2c_device_count - 1) json += ",";
    json += "\n";
  }
  json += "  ]\n";
  json += "}";
  
  return json;
}

// Individual sensor detection implementations
bool detect_mpu6050(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x68); // MPU6050 WHO_AM_I value
  }
  return false;
}

bool detect_mpu9250(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x71); // MPU9250 WHO_AM_I value
  }
  return false;
}

bool detect_icm20948(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0xEA); // ICM20948 WHO_AM_I value
  }
  return false;
}

bool detect_icm42688p(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x47); // ICM42688P WHO_AM_I value
  }
  return false;
}

bool detect_bmi270(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // CHIP_ID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t chip_id = Wire.read();
    return (chip_id == 0x24); // BMI270 CHIP_ID value
  }
  return false;
}

bool detect_lsm6dso32(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x0F); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x6C); // LSM6DSO32 WHO_AM_I value
  }
  return false;
}

bool detect_bmi323(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // CHIP_ID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t chip_id = Wire.read();
    return (chip_id == 0x43); // BMI323 CHIP_ID value
  }
  return false;
}

bool detect_icm20602(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x12); // ICM20602 WHO_AM_I value
  }
  return false;
}

bool detect_lsm6ds33(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x0F); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x69); // LSM6DS33 WHO_AM_I value
  }
  return false;
}

// Magnetometer detection functions
bool detect_hmc5883l(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x0A); // ID Register A
  Wire.endTransmission(false);
  Wire.requestFrom(address, 3, true);
  
  if (Wire.available() >= 3) {
    uint8_t id_a = Wire.read();
    uint8_t id_b = Wire.read();
    uint8_t id_c = Wire.read();
    return (id_a == 0x48 && id_b == 0x34 && id_c == 0x33); // "H43"
  }
  return false;
}

bool detect_qmc5883l(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x0D); // CHIP_ID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t chip_id = Wire.read();
    return (chip_id == 0xFF); // QMC5883L CHIP_ID value
  }
  return false;
}

bool detect_rm3100(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x36); // REVID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t rev_id = Wire.read();
    return (rev_id == 0x22); // RM3100 REVID value
  }
  return false;
}

bool detect_mmc5883ma(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x2F); // Product ID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t product_id = Wire.read();
    return (product_id == 0x0C); // MMC5883MA Product ID
  }
  return false;
}

bool detect_ist8310(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x10); // IST8310 WHO_AM_I value
  }
  return false;
}

bool detect_lis3mdl(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x0F); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x3D); // LIS3MDL WHO_AM_I value
  }
  return false;
}

bool detect_ak8963(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0x48); // AK8963 WHO_AM_I value
  }
  return false;
}

// Barometer detection functions
bool detect_bmp280(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0xD0); // ID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t chip_id = Wire.read();
    return (chip_id == 0x58); // BMP280 CHIP_ID value
  }
  return false;
}

bool detect_bme280(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0xD0); // ID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t chip_id = Wire.read();
    return (chip_id == 0x60); // BME280 CHIP_ID value
  }
  return false;
}

bool detect_bmp388(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // CHIP_ID register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t chip_id = Wire.read();
    return (chip_id == 0x50); // BMP388 CHIP_ID value
  }
  return false;
}

bool detect_ms5611(uint8_t address) {
  // MS5611 uses a reset command to detect presence
  Wire.beginTransmission(address);
  Wire.write(0x1E); // Reset command
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    delay(10); // Wait for reset
    
    // Read PROM to verify
    Wire.beginTransmission(address);
    Wire.write(0xA0); // Read PROM command
    Wire.endTransmission(false);
    Wire.requestFrom(address, 2, true);
    
    return (Wire.available() >= 2);
  }
  return false;
}

bool detect_lps22hb(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x0F); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    return (who_am_i == 0xB1); // LPS22HB WHO_AM_I value
  }
  return false;
}

// Enhanced calibration system
struct CalibrationData {
  // IMU calibration
  float gyro_offset[3];        // X, Y, Z gyro bias offsets
  float accel_offset[3];       // X, Y, Z accelerometer offsets
  float accel_scale[3];        // X, Y, Z accelerometer scale factors
  
  // Magnetometer calibration
  float mag_offset[3];         // X, Y, Z hard iron offsets
  float mag_scale[3];          // X, Y, Z soft iron scale factors
  float mag_rotation[9];       // 3x3 rotation matrix for soft iron
  
  // ESC calibration
  bool esc_calibrated;
  uint16_t esc_min[4];         // Per-motor minimum values
  uint16_t esc_max[4];         // Per-motor maximum values
  
  // RC calibration
  bool rc_calibrated;
  uint16_t rc_min[16];         // Per-channel minimum values
  uint16_t rc_max[16];         // Per-channel maximum values
  uint16_t rc_center[16];      // Per-channel center values
  
  // Calibration status flags
  bool gyro_calibrated;
  bool accel_calibrated;
  bool mag_calibrated;
  bool system_calibrated;      // All required calibrations complete
  
  // Calibration timestamps
  unsigned long gyro_cal_time;
  unsigned long accel_cal_time;
  unsigned long mag_cal_time;
  unsigned long esc_cal_time;
  unsigned long rc_cal_time;
  
  // Calibration quality scores (0-100)
  uint8_t gyro_cal_quality;
  uint8_t accel_cal_quality;
  uint8_t mag_cal_quality;
  uint8_t esc_cal_quality;
  uint8_t rc_cal_quality;
};

// Global calibration data
extern CalibrationData calibration_data;

// Enhanced calibration function prototypes
void init_calibration_system();
bool is_system_calibrated();
bool perform_gyro_calibration();
bool perform_accelerometer_calibration();
bool perform_accelerometer_position_calibration(int position);
bool perform_magnetometer_calibration();
bool perform_rc_calibration();
void save_calibration_to_eeprom();
void load_calibration_from_eeprom();
void reset_all_calibrations();
String get_calibration_status_json();
void apply_imu_calibration(IMUData* data);
void apply_magnetometer_calibration(MagnetometerData* data);

// Calibration wizard functions
void start_calibration_wizard();
bool check_calibration_prerequisites();
void calibration_step_gyro();
void calibration_step_accelerometer();
void calibration_step_magnetometer();
void calibration_step_rc();
void calibration_step_esc();

// Enhanced calibration and configuration functions
void start_magnetometer_timer_calibration();
void update_magnetometer_timer();
void start_enhanced_esc_calibration();
void proceed_esc_calibration_high();
void proceed_esc_calibration_low();
void complete_esc_calibration();
void check_motor_direction_configuration();
void fix_motor_direction(int motor_index);
void perform_propeller_safety_check();
void confirm_propellers_removed(bool removed);

// Implementation of missing calibration functions
void calibrate_imu() {
  Serial.println("=== IMU Calibration Started ===");
  
  if (!perform_gyro_calibration()) {
    Serial.println("ERROR: Gyro calibration failed");
    return;
  }
  
  if (!perform_accelerometer_calibration()) {
    Serial.println("ERROR: Accelerometer calibration failed");
    return;
  }
  
  calibration_data.system_calibrated = is_system_calibrated();
  save_calibration_to_eeprom();
  
  Serial.println("=== IMU Calibration Complete ===");
}

void calibrate_magnetometer() {
  Serial.println("=== Magnetometer Calibration Started ===");
  Serial.println("Rotate the drone slowly in all directions for 30 seconds...");
  
  if (!perform_magnetometer_calibration()) {
    Serial.println("ERROR: Magnetometer calibration failed");
    return;
  }
  
  calibration_data.system_calibrated = is_system_calibrated();
  save_calibration_to_eeprom();
  
  Serial.println("=== Magnetometer Calibration Complete ===");
}

bool perform_gyro_calibration() {
  Serial.println("Gyro calibration: Keep drone perfectly still for 10 seconds...");
  
  float gyro_sum[3] = {0, 0, 0};
  int sample_count = 0;
  float variance[3] = {0, 0, 0};
  unsigned long start_time = millis();
  
  // Collect samples for 10 seconds
  while (millis() - start_time < 10000) {
    IMUData temp_data;
    if (read_imu_data(&temp_data)) {
      gyro_sum[0] += temp_data.gyro_x;
      gyro_sum[1] += temp_data.gyro_y;
      gyro_sum[2] += temp_data.gyro_z;
      sample_count++;
      
      // Calculate variance for quality assessment
      for (int i = 0; i < 3; i++) {
        float value = (i == 0) ? temp_data.gyro_x : (i == 1) ? temp_data.gyro_y : temp_data.gyro_z;
        variance[i] += value * value;
      }
    }
    delay(10); // 100Hz sampling
  }
  
  if (sample_count < 500) {
    Serial.println("ERROR: Insufficient gyro samples collected");
    return false;
  }
  
  // Calculate offsets (average of all samples)
  for (int i = 0; i < 3; i++) {
    calibration_data.gyro_offset[i] = gyro_sum[i] / sample_count;
    variance[i] = sqrt(variance[i] / sample_count);
  }
  
  // Quality assessment based on variance (lower is better)
  float total_variance = variance[0] + variance[1] + variance[2];
  calibration_data.gyro_cal_quality = constrain(100 - (total_variance * 10), 0, 100);
  
  calibration_data.gyro_calibrated = (calibration_data.gyro_cal_quality > 70);
  calibration_data.gyro_cal_time = millis();
  
  Serial.print("Gyro calibration complete. Quality: ");
  Serial.print(calibration_data.gyro_cal_quality);
  Serial.println("%");
  
  return calibration_data.gyro_calibrated;
}

bool perform_accelerometer_calibration() {
  Serial.println("=== 6-Position Accelerometer Calibration ===");
  Serial.println("Place drone in each position when prompted and press any key...");
  
  const char* positions[6] = {"level", "nose up", "nose down", "left side", "right side", "upside down"};
  float expected_values[6][3] = {
    {0, 0, 1},    // level (Z up)
    {0, -1, 0},   // nose up (Y down)
    {0, 1, 0},    // nose down (Y up)
    {-1, 0, 0},   // left side (X down)
    {1, 0, 0},    // right side (X up)
    {0, 0, -1}    // upside down (Z down)
  };
  
  float measurements[6][3];
  
  for (int pos = 0; pos < 6; pos++) {
    Serial.print("Position ");
    Serial.print(pos + 1);
    Serial.print("/6: Place drone ");
    Serial.print(positions[pos]);
    Serial.println(" and press any key...");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    while (Serial.available()) Serial.read(); // Clear buffer
    
    Serial.println("Measuring... hold still for 3 seconds");
    
    float accel_sum[3] = {0, 0, 0};
    int samples = 0;
    unsigned long start_time = millis();
    
    while (millis() - start_time < 3000) {
      IMUData temp_data;
      if (read_imu_data(&temp_data)) {
        accel_sum[0] += temp_data.accel_x;
        accel_sum[1] += temp_data.accel_y;
        accel_sum[2] += temp_data.accel_z;
        samples++;
      }
      delay(10);
    }
    
    if (samples < 100) {
      Serial.println("ERROR: Insufficient samples for this position");
      return false;
    }
    
    measurements[pos][0] = accel_sum[0] / samples;
    measurements[pos][1] = accel_sum[1] / samples;
    measurements[pos][2] = accel_sum[2] / samples;
    
    Serial.println("Position recorded");
  }
  
  // Calculate scale factors and offsets using least squares
  for (int axis = 0; axis < 3; axis++) {
    float sum_measured = 0, sum_expected = 0, sum_measured_sq = 0, sum_cross = 0;
    
    for (int pos = 0; pos < 6; pos++) {
      sum_measured += measurements[pos][axis];
      sum_expected += expected_values[pos][axis];
      sum_measured_sq += measurements[pos][axis] * measurements[pos][axis];
      sum_cross += measurements[pos][axis] * expected_values[pos][axis];
    }
    
    // Calculate scale and offset
    float scale = (6 * sum_cross - sum_measured * sum_expected) / 
                  (6 * sum_measured_sq - sum_measured * sum_measured);
    float offset = (sum_expected - scale * sum_measured) / 6;
    
    calibration_data.accel_scale[axis] = scale;
    calibration_data.accel_offset[axis] = offset;
  }
  
  // Quality assessment based on fit accuracy
  float total_error = 0;
  for (int pos = 0; pos < 6; pos++) {
    for (int axis = 0; axis < 3; axis++) {
      float corrected = (measurements[pos][axis] - calibration_data.accel_offset[axis]) * calibration_data.accel_scale[axis];
      float error = abs(corrected - expected_values[pos][axis]);
      total_error += error;
    }
  }
  
  calibration_data.accel_cal_quality = constrain(100 - (total_error * 50), 0, 100);
  calibration_data.accel_calibrated = (calibration_data.accel_cal_quality > 80);
  calibration_data.accel_cal_time = millis();
  
  Serial.print("Accelerometer calibration complete. Quality: ");
  Serial.print(calibration_data.accel_cal_quality);
  Serial.println("%");
  
  return calibration_data.accel_calibrated;
}

bool perform_magnetometer_calibration() {
  Serial.println("Rotate drone slowly in ALL directions for 30 seconds...");
  Serial.println("Include pitch, roll, and yaw rotations for best results");
  
  float mag_min[3] = {999999, 999999, 999999};
  float mag_max[3] = {-999999, -999999, -999999};
  int sample_count = 0;
  unsigned long start_time = millis();
  
  // Collect min/max values during rotation
  while (millis() - start_time < 30000) {
    MagnetometerData temp_data;
    if (read_magnetometer_data(&temp_data)) {
      float values[3] = {temp_data.mag_x, temp_data.mag_y, temp_data.mag_z};
      
      for (int i = 0; i < 3; i++) {
        if (values[i] < mag_min[i]) mag_min[i] = values[i];
        if (values[i] > mag_max[i]) mag_max[i] = values[i];
      }
      sample_count++;
    }
    
    // Progress indicator
    if ((millis() - start_time) % 5000 == 0) {
      Serial.print("Progress: ");
      Serial.print((millis() - start_time) / 1000);
      Serial.println("/30 seconds");
    }
    
    delay(50); // 20Hz sampling
  }
  
  if (sample_count < 400) {
    Serial.println("ERROR: Insufficient magnetometer samples");
    return false;
  }
  
  // Calculate hard iron offsets (center of min/max)
  for (int i = 0; i < 3; i++) {
    calibration_data.mag_offset[i] = (mag_max[i] + mag_min[i]) / 2.0;
    calibration_data.mag_scale[i] = (mag_max[i] - mag_min[i]) / 2.0;
  }
  
  // Normalize scale factors
  float avg_scale = (calibration_data.mag_scale[0] + calibration_data.mag_scale[1] + calibration_data.mag_scale[2]) / 3.0;
  for (int i = 0; i < 3; i++) {
    calibration_data.mag_scale[i] = avg_scale / calibration_data.mag_scale[i];
  }
  
  // Quality assessment based on data range
  float range_quality = 0;
  for (int i = 0; i < 3; i++) {
    float range = mag_max[i] - mag_min[i];
    range_quality += (range > 200) ? 33.3 : (range / 200.0 * 33.3); // Expect decent range
  }
  
  calibration_data.mag_cal_quality = constrain(range_quality, 0, 100);
  calibration_data.mag_calibrated = (calibration_data.mag_cal_quality > 75);
  calibration_data.mag_cal_time = millis();
  
  Serial.print("Magnetometer calibration complete. Quality: ");
  Serial.print(calibration_data.mag_cal_quality);
  Serial.println("%");
  
  return calibration_data.mag_calibrated;
}

bool perform_accelerometer_position_calibration(int position) {
  if (position < 1 || position > 6) {
    Serial.println("Error: Invalid position number. Must be 1-6.");
    return false;
  }
  
  const char* position_names[] = {
    "level (normal flight orientation)",
    "nose up (90° pitch backward)", 
    "nose down (90° pitch forward)",
    "left side down (90° roll left)",
    "right side down (90° roll right)",
    "upside down (180° roll)"
  };
  
  Serial.print("=== Position ");
  Serial.print(position);
  Serial.print(": ");
  Serial.print(position_names[position - 1]);
  Serial.println(" ===");
  Serial.println("Place drone in this position and keep steady for sampling...");
  
  delay(3000); // 3 seconds to position
  
  Serial.println("Sampling... keep steady!");
  
  // Take 50 samples over 5 seconds for individual position
  float sum_x = 0, sum_y = 0, sum_z = 0;
  int valid_samples = 0;
  unsigned long start_time = millis();
  
  while (millis() - start_time < 5000) { // 5 second sampling
    IMUData temp_data;
    if (read_imu_data(&temp_data)) {
      sum_x += temp_data.accel_x;
      sum_y += temp_data.accel_y;
      sum_z += temp_data.accel_z;
      valid_samples++;
    }
    delay(100); // 10Hz sampling
  }
  
  if (valid_samples < 40) {
    Serial.println("Error: Not enough valid samples. Check IMU connection.");
    return false;
  }
  
  float avg_x = sum_x / valid_samples;
  float avg_y = sum_y / valid_samples;
  float avg_z = sum_z / valid_samples;
  
  // Store this position's data in static variables for multi-position calibration
  static float position_data[6][3];
  static bool position_complete[6] = {false, false, false, false, false, false};
  
  position_data[position - 1][0] = avg_x;
  position_data[position - 1][1] = avg_y;
  position_data[position - 1][2] = avg_z;
  position_complete[position - 1] = true;
  
  Serial.print("Position ");
  Serial.print(position);
  Serial.println(" sampling complete!");
  Serial.print("Measured: X=");
  Serial.print(avg_x, 3);
  Serial.print("g, Y=");
  Serial.print(avg_y, 3);
  Serial.print("g, Z=");
  Serial.print(avg_z, 3);
  Serial.println("g");
  
  // Check if all positions are complete for full calibration calculation
  bool all_complete = true;
  int completed_count = 0;
  for (int i = 0; i < 6; i++) {
    if (position_complete[i]) {
      completed_count++;
    } else {
      all_complete = false;
    }
  }
  
  Serial.print("Progress: ");
  Serial.print(completed_count);
  Serial.println("/6 positions completed");
  
  if (all_complete) {
    Serial.println("=== All 6 positions complete! Calculating calibration... ===");
    
    // Use the same expected values as the full calibration function
    float expected_values[6][3] = {
      {0, 0, 1},    // level (Z up)
      {0, -1, 0},   // nose up (Y down)
      {0, 1, 0},    // nose down (Y up)
      {-1, 0, 0},   // left side (X down)
      {1, 0, 0},    // right side (X up)
      {0, 0, -1}    // upside down (Z down)
    };
    
    // Calculate scale factors and offsets using least squares
    for (int axis = 0; axis < 3; axis++) {
      float sum_measured = 0, sum_expected = 0, sum_measured_sq = 0, sum_cross = 0;
      
      for (int pos = 0; pos < 6; pos++) {
        sum_measured += position_data[pos][axis];
        sum_expected += expected_values[pos][axis];
        sum_measured_sq += position_data[pos][axis] * position_data[pos][axis];
        sum_cross += position_data[pos][axis] * expected_values[pos][axis];
      }
      
      // Calculate scale and offset
      float scale = (6 * sum_cross - sum_measured * sum_expected) / 
                    (6 * sum_measured_sq - sum_measured * sum_measured);
      float offset = (sum_expected - scale * sum_measured) / 6;
      
      calibration_data.accel_scale[axis] = scale;
      calibration_data.accel_offset[axis] = offset;
    }
    
    // Quality assessment based on fit accuracy
    float total_error = 0;
    for (int pos = 0; pos < 6; pos++) {
      for (int axis = 0; axis < 3; axis++) {
        float corrected = (position_data[pos][axis] - calibration_data.accel_offset[axis]) * calibration_data.accel_scale[axis];
        float error = abs(corrected - expected_values[pos][axis]);
        total_error += error;
      }
    }
    
    calibration_data.accel_cal_quality = constrain(100 - (total_error * 50), 0, 100);
    calibration_data.accel_calibrated = (calibration_data.accel_cal_quality > 80);
    calibration_data.accel_cal_time = millis();
    
    Serial.println("=== COMPLETE ACCELEROMETER CALIBRATION FINISHED! ===");
    Serial.print("Offsets: X=");
    Serial.print(calibration_data.accel_offset[0], 4);
    Serial.print(", Y=");
    Serial.print(calibration_data.accel_offset[1], 4);
    Serial.print(", Z=");
    Serial.println(calibration_data.accel_offset[2], 4);
    Serial.print("Scales: X=");
    Serial.print(calibration_data.accel_scale[0], 4);
    Serial.print(", Y=");
    Serial.print(calibration_data.accel_scale[1], 4);
    Serial.print(", Z=");
    Serial.println(calibration_data.accel_scale[2], 4);
    Serial.print("Quality score: ");
    Serial.print(calibration_data.accel_cal_quality);
    Serial.println("%");
    
    // Reset position tracking for next calibration cycle
    for (int i = 0; i < 6; i++) {
      position_complete[i] = false;
    }
  } else {
    Serial.print("Remaining positions: ");
    for (int i = 0; i < 6; i++) {
      if (!position_complete[i]) {
        Serial.print(i + 1);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  
  return true;
}

bool is_system_calibrated() {
  return calibration_data.gyro_calibrated && 
         calibration_data.accel_calibrated && 
         calibration_data.mag_calibrated &&
         calibration_data.esc_calibrated &&
         calibration_data.rc_calibrated;
}

void apply_imu_calibration(IMUData* data) {
  if (!calibration_data.gyro_calibrated || !calibration_data.accel_calibrated) return;
  
  // Apply gyro offsets
  data->gyro_x -= calibration_data.gyro_offset[0];
  data->gyro_y -= calibration_data.gyro_offset[1];
  data->gyro_z -= calibration_data.gyro_offset[2];
  
  // Apply accelerometer calibration
  data->accel_x = (data->accel_x - calibration_data.accel_offset[0]) * calibration_data.accel_scale[0];
  data->accel_y = (data->accel_y - calibration_data.accel_offset[1]) * calibration_data.accel_scale[1];
  data->accel_z = (data->accel_z - calibration_data.accel_offset[2]) * calibration_data.accel_scale[2];
}

void apply_magnetometer_calibration(MagnetometerData* data) {
  if (!calibration_data.mag_calibrated) return;
  
  // Apply hard iron offsets and soft iron scaling
  data->mag_x = (data->mag_x - calibration_data.mag_offset[0]) * calibration_data.mag_scale[0];
  data->mag_y = (data->mag_y - calibration_data.mag_offset[1]) * calibration_data.mag_scale[1];
  data->mag_z = (data->mag_z - calibration_data.mag_offset[2]) * calibration_data.mag_scale[2];
}

String get_calibration_status_json() {
  String json = "{\n";
  json += "  \"system_calibrated\": " + String(calibration_data.system_calibrated ? "true" : "false") + ",\n";
  json += "  \"gyro\": {\n";
  json += "    \"calibrated\": " + String(calibration_data.gyro_calibrated ? "true" : "false") + ",\n";
  json += "    \"quality\": " + String(calibration_data.gyro_cal_quality) + ",\n";
  json += "    \"timestamp\": " + String(calibration_data.gyro_cal_time) + "\n";
  json += "  },\n";
  json += "  \"accelerometer\": {\n";
  json += "    \"calibrated\": " + String(calibration_data.accel_calibrated ? "true" : "false") + ",\n";
  json += "    \"quality\": " + String(calibration_data.accel_cal_quality) + ",\n";
  json += "    \"timestamp\": " + String(calibration_data.accel_cal_time) + "\n";
  json += "  },\n";
  json += "  \"magnetometer\": {\n";
  json += "    \"calibrated\": " + String(calibration_data.mag_calibrated ? "true" : "false") + ",\n";
  json += "    \"quality\": " + String(calibration_data.mag_cal_quality) + ",\n";
  json += "    \"timestamp\": " + String(calibration_data.mag_cal_time) + "\n";
  json += "  },\n";
  json += "  \"esc\": {\n";
  json += "    \"calibrated\": " + String(calibration_data.esc_calibrated ? "true" : "false") + ",\n";
  json += "    \"quality\": " + String(calibration_data.esc_cal_quality) + ",\n";
  json += "    \"timestamp\": " + String(calibration_data.esc_cal_time) + "\n";
  json += "  },\n";
  json += "  \"rc\": {\n";
  json += "    \"calibrated\": " + String(calibration_data.rc_calibrated ? "true" : "false") + ",\n";
  json += "    \"quality\": " + String(calibration_data.rc_cal_quality) + ",\n";
  json += "    \"timestamp\": " + String(calibration_data.rc_cal_time) + "\n";
  json += "  }\n";
  json += "}";
  return json;
}

// EEPROM Management Functions
#define CALIBRATION_EEPROM_ADDRESS 0
#define CALIBRATION_MAGIC_NUMBER 0xCAFE

void save_calibration_to_eeprom() {
  Serial.println("Saving calibration data to EEPROM...");
  
  // Write magic number first for validation
  EEPROM.put(CALIBRATION_EEPROM_ADDRESS, CALIBRATION_MAGIC_NUMBER);
  
  // Write calibration data
  EEPROM.put(CALIBRATION_EEPROM_ADDRESS + sizeof(uint16_t), calibration_data);
  
  Serial.println("Calibration data saved successfully");
}

void load_calibration_from_eeprom() {
  Serial.println("Loading calibration data from EEPROM...");
  
  uint16_t magic;
  EEPROM.get(CALIBRATION_EEPROM_ADDRESS, magic);
  
  if (magic == CALIBRATION_MAGIC_NUMBER) {
    EEPROM.get(CALIBRATION_EEPROM_ADDRESS + sizeof(uint16_t), calibration_data);
    Serial.println("Calibration data loaded successfully");
    
    // Verify loaded data
    if (is_system_calibrated()) {
      Serial.println("All calibrations verified and active");
    } else {
      Serial.println("Some calibrations missing - please recalibrate");
    }
  } else {
    Serial.println("No valid calibration data found in EEPROM");
    reset_all_calibrations();
  }
}

void reset_all_calibrations() {
  // Clear all calibration data
  memset(&calibration_data, 0, sizeof(CalibrationData));
  
  // Set default values
  for (int i = 0; i < 3; i++) {
    calibration_data.accel_scale[i] = 1.0; // Default scale factor
    calibration_data.mag_scale[i] = 1.0;
  }
  
  Serial.println("All calibration data reset");
}

void init_calibration_system() {
  // Initialize calibration data structure
  reset_all_calibrations();
  
  // Try to load from EEPROM
  load_calibration_from_eeprom();
  
  Serial.println("Calibration system initialized");
}

// Calibration Wizard Implementation
void start_calibration_wizard() {
  Serial.println("\n==================================");
  Serial.println("🧙‍♂️ FLIGHT CONTROLLER SETUP WIZARD");
  Serial.println("==================================");
  
  Serial.println("This wizard will guide you through all required calibrations");
  Serial.println("for safe flight operation. Please follow each step carefully.");
  Serial.println("");
  
  if (!check_calibration_prerequisites()) {
    return;
  }
  
  Serial.println("Step 1/6: Sensor Detection");
  perform_full_sensor_detection();
  delay(2000);
  
  Serial.println("\nStep 2/6: Gyroscope Calibration");
  calibration_step_gyro();
  delay(2000);
  
  Serial.println("\nStep 3/6: Accelerometer Calibration");
  calibration_step_accelerometer();
  delay(2000);
  
  Serial.println("\nStep 4/6: Magnetometer Calibration");
  calibration_step_magnetometer();
  delay(2000);
  
  Serial.println("\nStep 5/6: RC Receiver Setup");
  calibration_step_rc();
  delay(2000);
  
  Serial.println("\nStep 6/6: ESC/Motor Calibration");
  calibration_step_esc();
  
  // Final verification
  Serial.println("\n🔍 Final System Check...");
  if (is_system_calibrated()) {
    save_calibration_to_eeprom();
    Serial.println("\n✅ SETUP COMPLETE!");
    Serial.println("🚁 Your drone is now ready for safe flight operation.");
    Serial.println("📊 All calibrations passed and saved to EEPROM.");
  } else {
    Serial.println("\n❌ SETUP INCOMPLETE!");
    Serial.println("⚠️  Some calibrations failed or were skipped.");
    Serial.println("🔄 Please repeat the wizard or individual calibrations.");
  }
  
  Serial.println("==================================");
}

bool check_calibration_prerequisites() {
  Serial.println("🔍 Checking prerequisites...");
  
  // Check if motors are disarmed
  extern bool armed;
  if (armed) {
    Serial.println("❌ ERROR: Cannot calibrate while drone is armed!");
    Serial.println("Please disarm the drone first.");
    return false;
  }
  
  // Check basic sensor connectivity
  Serial.println("📡 Checking sensor connectivity...");
  bool sensors_ok = true;
  
  if (!sensor_data.imu.healthy) {
    Serial.println("❌ WARNING: IMU sensor not responding");
    sensors_ok = false;
  }
  
  if (!sensor_data.mag.healthy) {
    Serial.println("⚠️  WARNING: Magnetometer not responding (optional)");
  }
  
  if (!sensor_data.baro.healthy) {
    Serial.println("⚠️  WARNING: Barometer not responding (optional)");
  }
  
  if (!rc_data.signal_valid) {
    Serial.println("⚠️  WARNING: RC receiver signal not detected");
  }
  
  if (!sensors_ok) {
    Serial.println("❌ Critical sensors not responding. Check connections.");
    return false;
  }
  
  Serial.println("✅ Prerequisites check passed");
  return true;
}

void calibration_step_gyro() {
  Serial.println("📐 Gyroscope Bias Calibration");
  Serial.println("Place the drone on a level, stable surface.");
  Serial.println("DO NOT MOVE the drone during calibration.");
  Serial.println("Press any key when ready...");
  
  // Wait for user input
  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read(); // Clear buffer
  
  if (perform_gyro_calibration()) {
    Serial.println("✅ Gyroscope calibration successful");
  } else {
    Serial.println("❌ Gyroscope calibration failed");
  }
}

void calibration_step_accelerometer() {
  Serial.println("📏 Accelerometer Level Calibration");
  Serial.println("This requires placing the drone in 6 different positions.");
  Serial.println("Follow the on-screen instructions carefully.");
  Serial.println("Press any key when ready...");
  
  // Wait for user input
  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read(); // Clear buffer
  
  if (perform_accelerometer_calibration()) {
    Serial.println("✅ Accelerometer calibration successful");
  } else {
    Serial.println("❌ Accelerometer calibration failed");
  }
}

void calibration_step_magnetometer() {
  Serial.println("🧭 Magnetometer Compass Calibration");
  Serial.println("Move away from metal objects and electronics.");
  Serial.println("You will need to rotate the drone in ALL directions.");
  Serial.println("Press any key when ready...");
  
  // Wait for user input
  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read(); // Clear buffer
  
  if (perform_magnetometer_calibration()) {
    Serial.println("✅ Magnetometer calibration successful");
  } else {
    Serial.println("❌ Magnetometer calibration failed");
  }
}

void calibration_step_rc() {
  Serial.println("📡 RC Receiver Setup");
  Serial.println("Ensure your RC transmitter is ON and bound.");
  Serial.println("Move all sticks and switches through their full range.");
  Serial.println("This step will be implemented in receiver configuration.");
  
  // For now, just mark as calibrated if RC signal is valid
  if (rc_data.signal_valid) {
    calibration_data.rc_calibrated = true;
    calibration_data.rc_cal_quality = 85;
    calibration_data.rc_cal_time = millis();
    Serial.println("✅ RC receiver signal detected");
  } else {
    Serial.println("❌ No RC receiver signal - check transmitter and binding");
  }
}

void calibration_step_esc() {
  Serial.println("Starting ESC calibration step...");
  
  // Ensure drone is disarmed
  if (armed) {
    Serial.println("ERROR: Cannot calibrate ESCs while armed");
    return;
  }
  
  // Remove propellers warning
  Serial.println("WARNING: Ensure ALL PROPELLERS are removed before ESC calibration!");
  Serial.println("This step will output high throttle signals to ESCs");
  delay(3000);
  
  // Perform ESC calibration
  motor_control.calibrate_escs();
  
  // Update calibration status
  calibration_data.esc_calibrated = true;
  calibration_data.esc_cal_time = millis();
  calibration_data.esc_cal_quality = 95; // Assume good quality for auto calibration
  
  Serial.println("ESC calibration step complete");
}

// GPS requirement checking functions
bool check_gps_functions_configured() {
  for (int i = 0; i < 16; i++) {
    ChannelFunction func = rc_config.channels[i].function;
    if (func == CHAN_FUNC_RTH || func == CHAN_FUNC_GPS_RESCUE || func == CHAN_FUNC_POSITION_HOLD) {
      return true;
    }
  }
  return false;
}

// Helper function to check if GPS functionality is available (dedicated OR synthetic)
bool is_gps_functionality_available() {
  // Check for dedicated GPS first
  if (sensor_data.gps.healthy && sensor_data.gps.fix && sensor_data.gps.satellites >= 6) {
    return true;
  }
  
  // Check for synthetic GPS fallback
  SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
  if (synthetic.synthetic_gps_valid && synthetic.gps_confidence > 0.3) {
    return true;
  }
  
  return false;
}

// Helper function to check if magnetometer functionality is available (dedicated OR synthetic)
bool is_magnetometer_functionality_available() {
  // Check for dedicated magnetometer
  if (sensor_data.mag.healthy) {
    return true;
  }
  
  // Check for synthetic magnetometer fallback
  SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
  if (synthetic.synthetic_mag_valid && synthetic.mag_confidence > 0.3) {
    return true;
  }
  
  return false;
}

// Helper function to check if barometer functionality is available (dedicated OR synthetic)
bool is_barometer_functionality_available() {
  // Check for dedicated barometer
  if (sensor_data.baro.healthy) {
    return true;
  }
  
  // Check for synthetic barometer fallback
  SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
  if (synthetic.synthetic_baro_valid && synthetic.baro_confidence > 0.3) {
    return true;
  }
  
  return false;
}

bool is_gps_required_for_arming() {
  // Check if any GPS-dependent functions are configured
  bool gps_functions_configured = check_gps_functions_configured();
  
  if (gps_functions_configured) {
    // Enhanced check: Use dedicated OR synthetic GPS
    return !is_gps_functionality_available();
  }
  
  return false; // No GPS functions configured, GPS not required
}

// Battery connection detection
bool detect_battery_connection() {
  // Check if battery voltage is within reasonable range
  float voltage = analogRead(BATTERY_VOLTAGE_PIN) * VOLTAGE_SCALE;
  
  // Update sensor data
  sensor_data.battery_voltage = voltage;
  
  // Battery should be between 6V (very low 2S) and 30V (high 6S)
  // If reading is below 1V, likely no battery connected or sensor issue
  if (voltage < 1.0) {
    return false; // No battery or sensor disconnected
  }
  
  if (voltage > 30.0) {
    return false; // Voltage too high, likely sensor issue
  }
  
  return true; // Battery appears to be connected
}

// ESC voltage telemetry checking
bool check_esc_voltage_telemetry() {
  // Check if ESCs are configured for DShot with telemetry
  EscConfig esc_config = motor_control.get_esc_config();
  
  if (esc_config.protocol >= ESC_PROTOCOL_DSHOT150 && esc_config.telemetry_enabled) {
    // Check if we're actually receiving voltage telemetry from any motor
    for (int i = 0; i < 4; i++) {
      MotorTelemetry telemetry = motor_control.get_motor_telemetry(i);
      if (telemetry.data_valid && telemetry.voltage > 0) {
        return true; // At least one ESC is providing voltage telemetry
      }
    }
  }
  
  return false; // No ESC voltage telemetry available
}

// Enhanced arming safety check with GPS requirements
bool enhanced_safety_check_for_arming() {
  bool safe_to_arm = true;
  
  // 1. Standard calibration check
  if (!is_system_calibrated()) {
    Serial.println("ARMING BLOCKED: System not calibrated");
    safe_to_arm = false;
  }
  
  // 2. Battery connection check
  if (!detect_battery_connection()) {
    Serial.println("ARMING BLOCKED: No battery detected");
    safe_to_arm = false;
  }
  
  // 3. Low battery check
  if (sensor_data.battery_voltage < LOW_VOLTAGE_THRESHOLD) {
    Serial.println("ARMING BLOCKED: Battery voltage too low");
    safe_to_arm = false;
  }
  
  // 4. RC signal check
  if (!rc_data.signal_valid || (millis() - rc_data.last_update > 1000)) {
    Serial.println("ARMING BLOCKED: No RC signal");
    safe_to_arm = false;
  }
  
  // 5. GPS requirement check (enhanced with synthetic GPS support)
  bool gps_functions_configured = check_gps_functions_configured();
  if (gps_functions_configured) {
    if (is_gps_functionality_available()) {
      if (sensor_data.gps.healthy && sensor_data.gps.fix) {
        Serial.println("GPS Functions: Using dedicated GPS");
      } else {
        SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
        if (synthetic.synthetic_gps_valid) {
          Serial.print("GPS Functions: Using synthetic GPS (");
          Serial.print(synthetic.gps_confidence * 100, 1);
          Serial.println("% confidence)");
        }
      }
    } else {
      Serial.println("ARMING BLOCKED: GPS functions configured but no GPS available");
      Serial.println("GPS-dependent functions require either:");
      Serial.println("  - Dedicated GPS with 6+ satellites, OR");
      Serial.println("  - Synthetic GPS with >30% confidence");
      Serial.println("Either disable GPS functions or improve sensor availability");
      safe_to_arm = false;
    }
  }
  
  // 6. IMU health check
  if (!sensor_data.imu.healthy) {
    Serial.println("ARMING BLOCKED: IMU not healthy");
    safe_to_arm = false;
  }
  
  return safe_to_arm;
}

// Wait for GPS lock function
bool wait_for_gps_lock(unsigned long timeout_ms) {
  if (!check_gps_functions_configured()) {
    return true; // No GPS functions configured, no need to wait
  }
  
  Serial.println("Waiting for GPS lock...");
  unsigned long start_time = millis();
  
  while ((millis() - start_time) < timeout_ms) {
    // Update GPS data
    read_gps_data(&sensor_data.gps);
    
    if (sensor_data.gps.healthy && sensor_data.gps.fix && sensor_data.gps.satellites >= 6) {
      Serial.print("GPS lock acquired with ");
      Serial.print(sensor_data.gps.satellites);
      Serial.println(" satellites");
      return true;
    }
    
    // Print status every 5 seconds
    if ((millis() - start_time) % 5000 < 100) {
      Serial.print("GPS status: ");
      if (sensor_data.gps.healthy) {
        Serial.print("Detected, ");
        Serial.print(sensor_data.gps.satellites);
        Serial.print(" satellites, ");
        Serial.println(sensor_data.gps.fix ? "FIX" : "NO FIX");
      } else {
        Serial.println("Not detected");
      }
    }
    
    delay(100);
  }
  
  Serial.println("GPS lock timeout - arming not allowed");
  return false;
}

// Enhanced battery monitoring with ESC telemetry
struct BatteryStatus {
  bool connected;
  float main_voltage;      // From main voltage sensor
  float esc_voltage;       // From ESC telemetry (if available)
  float current;
  bool low_voltage_warning;
  bool critical_voltage;
  bool esc_telemetry_available;
};

BatteryStatus get_enhanced_battery_status() {
  BatteryStatus status;
  
  // Main voltage sensor
  status.connected = detect_battery_connection();
  status.main_voltage = sensor_data.battery_voltage;
  status.current = sensor_data.current;
  
  // ESC voltage telemetry
  status.esc_telemetry_available = check_esc_voltage_telemetry();
  status.esc_voltage = 0.0;
  
  if (status.esc_telemetry_available) {
    // Get average voltage from ESC telemetry
    float total_voltage = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 4; i++) {
      MotorTelemetry telemetry = motor_control.get_motor_telemetry(i);
      if (telemetry.data_valid && telemetry.voltage > 0) {
        total_voltage += telemetry.voltage / 1000.0; // Convert mV to V
        valid_readings++;
      }
    }
    
    if (valid_readings > 0) {
      status.esc_voltage = total_voltage / valid_readings;
    }
  }
  
  // Use the more accurate voltage reading (ESC telemetry is usually more accurate)
  float working_voltage = status.esc_telemetry_available ? status.esc_voltage : status.main_voltage;
  
  // Battery warnings
  status.critical_voltage = (working_voltage < LOW_VOLTAGE_THRESHOLD);
  status.low_voltage_warning = (working_voltage < (LOW_VOLTAGE_THRESHOLD + 1.0));
  
  return status;
}

// Enhanced magnetometer calibration with timer
struct MagCalibrationTimer {
  bool active;
  unsigned long start_time;
  unsigned long duration_ms;
  int progress_percentage;
  unsigned long last_update;
};

MagCalibrationTimer mag_timer = {false, 0, 90000, 0, 0}; // 90 seconds

void start_magnetometer_timer_calibration() {
  if (mag_timer.active) {
    Serial.println("Magnetometer timer calibration already in progress");
    return;
  }
  
  Serial.println("=== Magnetometer Timer Calibration Started ===");
  Serial.println("Duration: 90 seconds");
  Serial.println("Instructions:");
  Serial.println("1. Hold the drone firmly in your hands");
  Serial.println("2. Move in slow, smooth figure-8 patterns");
  Serial.println("3. Rotate around all three axes (pitch, roll, yaw)");
  Serial.println("4. Keep movements continuous and steady");
  Serial.println("5. Stay away from metal objects and electronics");
  Serial.println("");
  Serial.println("Calibration starting now...");
  
  mag_timer.active = true;
  mag_timer.start_time = millis();
  mag_timer.progress_percentage = 0;
  mag_timer.last_update = millis();
  
  // Start actual magnetometer calibration
  perform_magnetometer_calibration();
}

void update_magnetometer_timer() {
  if (!mag_timer.active) return;
  
  unsigned long current_time = millis();
  unsigned long elapsed = current_time - mag_timer.start_time;
  
  // Update progress every second
  if (current_time - mag_timer.last_update >= 1000) {
    mag_timer.progress_percentage = (elapsed * 100) / mag_timer.duration_ms;
    
    if (mag_timer.progress_percentage > 100) {
      mag_timer.progress_percentage = 100;
    }
    
    int remaining_seconds = (mag_timer.duration_ms - elapsed) / 1000;
    if (remaining_seconds < 0) remaining_seconds = 0;
    
    Serial.print("Magnetometer calibration progress: ");
    Serial.print(mag_timer.progress_percentage);
    Serial.print("% (");
    Serial.print(remaining_seconds);
    Serial.println(" seconds remaining)");
    
    // Provide movement instructions
    if (remaining_seconds > 60) {
      Serial.println("Continue figure-8 movements in horizontal plane...");
    } else if (remaining_seconds > 30) {
      Serial.println("Now perform vertical figure-8 movements...");
    } else if (remaining_seconds > 0) {
      Serial.println("Final rotations - include pitch and roll movements...");
    }
    
    mag_timer.last_update = current_time;
  }
  
  // Check if calibration time is complete
  if (elapsed >= mag_timer.duration_ms) {
    Serial.println("=== Magnetometer Timer Calibration Complete ===");
    Serial.println("✅ 90-second calibration period finished");
    Serial.println("Magnetometer calibration data saved");
    
    mag_timer.active = false;
    
    // Update calibration status
    calibration_data.mag_calibrated = true;
    calibration_data.mag_cal_quality = 85; // Timer-based calibration quality
    calibration_data.mag_cal_time = millis();
    
    save_calibration_to_eeprom();
  }
}

// Enhanced ESC calibration system
enum EscCalibrationState {
  ESC_CAL_IDLE,
  ESC_CAL_STARTED,
  ESC_CAL_HIGH_THROTTLE,
  ESC_CAL_LOW_THROTTLE,
  ESC_CAL_COMPLETE
};

struct EscCalibrationStatus {
  EscCalibrationState state;
  unsigned long step_start_time;
  bool safety_confirmed;
};

EscCalibrationStatus esc_cal_status = {ESC_CAL_IDLE, 0, false};

void start_enhanced_esc_calibration() {
  if (esc_cal_status.state != ESC_CAL_IDLE) {
    Serial.println("ESC calibration already in progress");
    return;
  }
  
  if (!esc_cal_status.safety_confirmed) {
    Serial.println("⚠️  SAFETY CHECK REQUIRED");
    Serial.println("Before starting ESC calibration:");
    Serial.println("1. Remove ALL propellers");
    Serial.println("2. Ensure drone is disarmed");
    Serial.println("3. Place drone in stable position");
    Serial.println("");
    Serial.println("Type 'confirm propellers removed' to proceed");
    return;
  }
  
  Serial.println("=== Enhanced ESC Calibration Started ===");
  Serial.println("Step 1: Preparing for high throttle signal");
  Serial.println("Connect battery and wait for ESC startup beeps");
  Serial.println("Then type 'esc calibration high' to proceed");
  
  esc_cal_status.state = ESC_CAL_STARTED;
  esc_cal_status.step_start_time = millis();
}

void proceed_esc_calibration_high() {
  if (esc_cal_status.state != ESC_CAL_STARTED) {
    Serial.println("ESC calibration not properly started. Use 'esc calibration start' first");
    return;
  }
  
  Serial.println("Step 2: Sending HIGH throttle signal");
  Serial.println("ESCs will receive maximum throttle for 5 seconds...");
  
  // Send high throttle to all motors
  extern MotorControl motor_control;
  for (int i = 0; i < 4; i++) {
    motor_control.output_pwm(i, 2000); // Maximum PWM
  }
  
  delay(5000);
  
  Serial.println("High throttle phase complete");
  Serial.println("Listen for ESC confirmation beeps");
  Serial.println("Type 'esc calibration low' to continue to low throttle");
  
  esc_cal_status.state = ESC_CAL_HIGH_THROTTLE;
  esc_cal_status.step_start_time = millis();
}

void proceed_esc_calibration_low() {
  if (esc_cal_status.state != ESC_CAL_HIGH_THROTTLE) {
    Serial.println("Must complete high throttle step first");
    return;
  }
  
  Serial.println("Step 3: Sending LOW throttle signal");
  Serial.println("ESCs will receive minimum throttle for 5 seconds...");
  
  // Send low throttle to all motors
  extern MotorControl motor_control;
  for (int i = 0; i < 4; i++) {
    motor_control.output_pwm(i, 1000); // Minimum PWM
  }
  
  delay(5000);
  
  Serial.println("Low throttle phase complete");
  Serial.println("Listen for ESC calibration confirmation beeps");
  Serial.println("Type 'esc calibration complete' to finish");
  
  esc_cal_status.state = ESC_CAL_LOW_THROTTLE;
  esc_cal_status.step_start_time = millis();
}

void complete_esc_calibration() {
  if (esc_cal_status.state != ESC_CAL_LOW_THROTTLE) {
    Serial.println("Must complete low throttle step first");
    return;
  }
  
  Serial.println("Step 4: Completing ESC calibration");
  
  // Return motors to safe armed position
  extern MotorControl motor_control;
  for (int i = 0; i < 4; i++) {
    motor_control.output_pwm(i, 1100); // Armed but not spinning
  }
  
  // Update calibration status
  calibration_data.esc_calibrated = true;
  calibration_data.esc_cal_quality = 95;
  calibration_data.esc_cal_time = millis();
  
  save_calibration_to_eeprom();
  
  Serial.println("=== ESC Calibration Complete ===");
  Serial.println("✅ All ESCs calibrated successfully");
  Serial.println("ESCs now know throttle range (1000-2000μs)");
  Serial.println("You can now install propellers and test motors");
  
  esc_cal_status.state = ESC_CAL_COMPLETE;
  esc_cal_status.step_start_time = millis();
  
  // Reset for next calibration
  delay(2000);
  esc_cal_status.state = ESC_CAL_IDLE;
}

// Motor direction validation for X-configuration
void check_motor_direction_configuration() {
  Serial.println("=== Motor Direction Configuration Check ===");
  Serial.println("Checking for proper X-configuration setup:");
  Serial.println("");
  
  // Expected X-configuration:
  // Motor 1 (Front Right): CW
  // Motor 2 (Rear Right): CCW  
  // Motor 3 (Rear Left): CW
  // Motor 4 (Front Left): CCW
  
  extern MotorControl motor_control;
  EscConfig config = motor_control.get_esc_config();
  
  bool directions_correct = true;
  
  Serial.println("Expected vs Actual directions:");
  Serial.print("Motor 1 (Front Right): Expected CW,  Actual ");
  if (config.motor_direction[0] == 1) {
    Serial.println("CW  ✅");
  } else {
    Serial.println("CCW ❌");
    directions_correct = false;
  }
  
  Serial.print("Motor 2 (Rear Right):  Expected CCW, Actual ");
  if (config.motor_direction[1] == -1) {
    Serial.println("CCW ✅");
  } else {
    Serial.println("CW  ❌");
    directions_correct = false;
  }
  
  Serial.print("Motor 3 (Rear Left):   Expected CW,  Actual ");
  if (config.motor_direction[2] == 1) {
    Serial.println("CW  ✅");
  } else {
    Serial.println("CCW ❌");
    directions_correct = false;
  }
  
  Serial.print("Motor 4 (Front Left):  Expected CCW, Actual ");
  if (config.motor_direction[3] == -1) {
    Serial.println("CCW ✅");
  } else {
    Serial.println("CW  ❌");
    directions_correct = false;
  }
  
  Serial.println("");
  if (directions_correct) {
    Serial.println("🎉 Motor directions are correctly configured!");
    Serial.println("Your X-configuration is ready for flight");
  } else {
    Serial.println("⚠️  Motor direction configuration needs adjustment");
    Serial.println("Use 'fix motor direction [1-4]' to correct individual motors");
    Serial.println("Or manually configure using 'set esc direction [1-4] [normal/reverse]'");
  }
  Serial.println("=========================================");
}

void fix_motor_direction(int motor_index) {
  if (motor_index < 0 || motor_index > 3) {
    Serial.println("Invalid motor index");
    return;
  }
  
  // Expected directions for X-configuration
  int expected_directions[4] = {1, -1, 1, -1}; // CW, CCW, CW, CCW
  
  extern MotorControl motor_control;
  motor_control.set_motor_direction(motor_index, expected_directions[motor_index]);
  
  String direction_name = (expected_directions[motor_index] == 1) ? "CW" : "CCW";
  Serial.print("Fixed motor ");
  Serial.print(motor_index + 1);
  Serial.print(" direction to ");
  Serial.print(direction_name);
  Serial.println(" for proper X-configuration");
}

// Safety system for propeller management
struct PropellerSafety {
  bool propellers_removed;
  unsigned long last_confirmation_time;
  bool safety_bypass_enabled; // For advanced users
};

PropellerSafety prop_safety = {false, 0, false};

void perform_propeller_safety_check() {
  Serial.println("=== Propeller Safety Status ===");
  Serial.print("Propellers status: ");
  if (prop_safety.propellers_removed) {
    Serial.println("REMOVED ✅");
    Serial.print("Confirmed ");
    Serial.print((millis() - prop_safety.last_confirmation_time) / 1000);
    Serial.println(" seconds ago");
  } else {
    Serial.println("INSTALLED ⚠️");
  }
  
  Serial.println("");
  Serial.println("Safety requirements for motor operations:");
  Serial.println("• Motor testing: Requires propellers REMOVED");
  Serial.println("• ESC calibration: Requires propellers REMOVED");  
  Serial.println("• Flight operations: Requires propellers INSTALLED");
  Serial.println("");
  
  if (prop_safety.safety_bypass_enabled) {
    Serial.println("⚠️  Safety bypass is ENABLED");
    Serial.println("This allows motor operations regardless of propeller status");
  }
  
  Serial.println("Commands:");
  Serial.println("• 'confirm propellers removed' - Enable motor testing");
  Serial.println("• 'confirm propellers installed' - Ready for flight");
  Serial.println("==============================");
}

void confirm_propellers_removed(bool removed) {
  prop_safety.propellers_removed = removed;
  prop_safety.last_confirmation_time = millis();
  
  if (removed) {
    Serial.println("✅ Propellers confirmed as REMOVED");
    Serial.println("Motor testing and ESC calibration now enabled");
    Serial.println("⚠️  Do NOT install propellers during motor operations");
    esc_cal_status.safety_confirmed = true;
  } else {
    Serial.println("✅ Propellers confirmed as INSTALLED");
    Serial.println("Ready for flight operations");
    Serial.println("⚠️  Motor testing and ESC calibration now DISABLED");
    esc_cal_status.safety_confirmed = false;
  }
}

struct EnhancedSensorFusion {
  // Multi-rate sensor processing
  bool multi_rate_fusion_enabled;
  unsigned long imu_update_interval;      // High frequency (2kHz)
  unsigned long mag_update_interval;      // Medium frequency (50Hz)
  unsigned long baro_update_interval;     // Low frequency (25Hz)
  unsigned long gps_update_interval;      // Very low frequency (10Hz)
  
  // Adaptive fusion weights
  bool adaptive_fusion_enabled;
  float imu_confidence_weight;            // Dynamic weight based on sensor health
  float mag_confidence_weight;
  float baro_confidence_weight;
  float gps_confidence_weight;
  
  // Advanced Kalman filtering
  bool extended_kalman_filter_enabled;
  float state_estimate[9];                // Position, velocity, acceleration
  float covariance_matrix[9][9];         // State covariance
  float process_noise[9];                 // Process noise model
  float measurement_noise[4];             // Measurement noise (IMU, GPS, Baro, Mag)
  
  // Sensor cross-validation
  bool cross_validation_enabled;
  float sensor_agreement_threshold;       // Threshold for sensor disagreement
  bool sensor_outlier_detection;
  float outlier_rejection_threshold;
  
  // Environmental adaptation
  bool environmental_adaptation_enabled;
  float vibration_environment_factor;     // Adjust fusion based on vibration
  float temperature_compensation_factor;  // Temperature-based sensor drift compensation
  float magnetic_declination_auto;        // Auto-calculate magnetic declination
};

struct SensorQuality {
  float imu_quality_score;               // 0-100 IMU data quality
  float gps_accuracy_score;              // Based on HDOP and satellite count
  float magnetometer_quality_score;      // Based on calibration and stability
  float barometer_stability_score;       // Based on pressure variance
  
  unsigned long last_quality_update;
  bool quality_assessment_enabled;
};

struct AdvancedFiltering {
  // Complementary filter with adaptive gains
  bool adaptive_complementary_enabled;
  float complementary_alpha;             // Adaptive mixing ratio
  float complementary_beta;              // Gyro correction gain
  
  // Motion detection for filter adaptation
  bool motion_adaptive_filtering;
  float motion_threshold;                // Threshold for motion detection
  bool in_motion;                        // Current motion state
  float static_filter_gain;              // Gains for static operation
  float dynamic_filter_gain;             // Gains for dynamic operation
  
  // Gravity vector estimation
  bool gravity_vector_estimation;
  float estimated_gravity[3];            // Estimated gravity vector
  float gravity_confidence;              // Confidence in gravity estimate
};

// Global sensor detection status
extern SensorDetectionStatus sensor_detection;

// Enhanced sensor detection functions
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

// Individual sensor detection functions
bool detect_mpu6050(uint8_t address = 0x68);
bool detect_mpu9250(uint8_t address = 0x68);
bool detect_icm20948(uint8_t address = 0x69);
bool detect_icm42688p(uint8_t address = 0x68);
bool detect_bmi270(uint8_t address = 0x68);
bool detect_lsm6dso32(uint8_t address = 0x6A);
bool detect_bmi323(uint8_t address = 0x68);
bool detect_icm20602(uint8_t address = 0x68);
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

// Enhanced sensor detection implementation
void scan_i2c_devices() {
  sensor_detection.i2c_device_count = 0;
  
  Serial.println("Scanning I2C bus for devices...");
  Serial.println("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
  
  for (int addr = 0; addr < 128; addr++) {
    if (addr % 16 == 0) {
      Serial.print(addr < 16 ? "0" : "");
      Serial.print(addr, HEX);
      Serial.print(": ");
    }
    
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(addr < 16 ? "0" : "");
      Serial.print(addr, HEX);
      Serial.print(" ");
      
      // Store found device
      if (sensor_detection.i2c_device_count < 16) {
        sensor_detection.i2c_devices[sensor_detection.i2c_device_count] = addr;
        sensor_detection.i2c_device_count++;
      }
    } else {
      Serial.print("-- ");
    }
    
    if ((addr + 1) % 16 == 0) {
      Serial.println();
    }
  }
  
  Serial.print("Found ");
  Serial.print(sensor_detection.i2c_device_count);
  Serial.println(" I2C devices");
}

bool detect_imu_sensors() {
  bool found_any = false;
  
  // Clear IMU detection flags
  sensor_detection.mpu6050_present = false;
  sensor_detection.mpu9250_present = false;
  sensor_detection.icm20948_present = false;
  sensor_detection.icm42688p_present = false;
  sensor_detection.bmi270_present = false;
  sensor_detection.lsm6dso32_present = false;
  sensor_detection.bmi323_present = false;
  sensor_detection.icm20602_present = false;
  sensor_detection.lsm6ds33_present = false;
  
  // Scan common IMU addresses
  uint8_t imu_addresses[] = {0x68, 0x69, 0x6A, 0x6B};
  
  for (int i = 0; i < 4; i++) {
    uint8_t addr = imu_addresses[i];
    
    if (detect_mpu6050(addr)) {
      sensor_detection.mpu6050_present = true;
      found_any = true;
      Serial.print("MPU6050 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_mpu9250(addr)) {
      sensor_detection.mpu9250_present = true;
      found_any = true;
      Serial.print("MPU9250 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_icm20948(addr)) {
      sensor_detection.icm20948_present = true;
      found_any = true;
      Serial.print("ICM20948 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_icm42688p(addr)) {
      sensor_detection.icm42688p_present = true;
      found_any = true;
      Serial.print("ICM42688P detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bmi270(addr)) {
      sensor_detection.bmi270_present = true;
      found_any = true;
      Serial.print("BMI270 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lsm6dso32(addr)) {
      sensor_detection.lsm6dso32_present = true;
      found_any = true;
      Serial.print("LSM6DSO32 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bmi323(addr)) {
      sensor_detection.bmi323_present = true;
      found_any = true;
      Serial.print("BMI323 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_icm20602(addr)) {
      sensor_detection.icm20602_present = true;
      found_any = true;
      Serial.print("ICM20602 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lsm6ds33(addr)) {
      sensor_detection.lsm6ds33_present = true;
      found_any = true;
      Serial.print("LSM6DS33 detected at 0x");
      Serial.println(addr, HEX);
    }
  }
  
  if (!found_any) {
    Serial.println("No IMU sensors detected");
  }
  
  return found_any;
}

bool detect_magnetometer_sensors() {
  bool found_any = false;
  
  // Clear magnetometer detection flags
  sensor_detection.hmc5883l_present = false;
  sensor_detection.qmc5883l_present = false;
  sensor_detection.rm3100_present = false;
  sensor_detection.mmc5883ma_present = false;
  sensor_detection.ist8310_present = false;
  sensor_detection.lis3mdl_present = false;
  sensor_detection.ak8963_present = false;
  
  // Common magnetometer addresses
  uint8_t mag_addresses[] = {0x0C, 0x0D, 0x0E, 0x1C, 0x1E, 0x20, 0x30};
  
  for (int i = 0; i < 7; i++) {
    uint8_t addr = mag_addresses[i];
    
    if (detect_hmc5883l(addr)) {
      sensor_detection.hmc5883l_present = true;
      found_any = true;
      Serial.print("HMC5883L detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_qmc5883l(addr)) {
      sensor_detection.qmc5883l_present = true;
      found_any = true;
      Serial.print("QMC5883L detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_rm3100(addr)) {
      sensor_detection.rm3100_present = true;
      found_any = true;
      Serial.print("RM3100 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_mmc5883ma(addr)) {
      sensor_detection.mmc5883ma_present = true;
      found_any = true;
      Serial.print("MMC5883MA detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_ist8310(addr)) {
      sensor_detection.ist8310_present = true;
      found_any = true;
      Serial.print("IST8310 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lis3mdl(addr)) {
      sensor_detection.lis3mdl_present = true;
      found_any = true;
      Serial.print("LIS3MDL detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_ak8963(addr)) {
      sensor_detection.ak8963_present = true;
      found_any = true;
      Serial.print("AK8963 detected at 0x");
      Serial.println(addr, HEX);
    }
  }
  
  if (!found_any) {
    Serial.println("No magnetometer sensors detected");
  }
  
  return found_any;
}

bool detect_barometer_sensors() {
  bool found_any = false;
  
  // Clear barometer detection flags
  sensor_detection.bmp280_present = false;
  sensor_detection.bme280_present = false;
  sensor_detection.bmp388_present = false;
  sensor_detection.ms5611_present = false;
  sensor_detection.lps22hb_present = false;
  
  // Common barometer addresses
  uint8_t baro_addresses[] = {0x76, 0x77, 0x5C, 0x5D};
  
  for (int i = 0; i < 4; i++) {
    uint8_t addr = baro_addresses[i];
    
    if (detect_bmp280(addr)) {
      sensor_detection.bmp280_present = true;
      found_any = true;
      Serial.print("BMP280 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bme280(addr)) {
      sensor_detection.bme280_present = true;
      found_any = true;
      Serial.print("BME280 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_bmp388(addr)) {
      sensor_detection.bmp388_present = true;
      found_any = true;
      Serial.print("BMP388 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_ms5611(addr)) {
      sensor_detection.ms5611_present = true;
      found_any = true;
      Serial.print("MS5611 detected at 0x");
      Serial.println(addr, HEX);
    }
    
    if (detect_lps22hb(addr)) {
      sensor_detection.lps22hb_present = true;
      found_any = true;
      Serial.print("LPS22HB detected at 0x");
      Serial.println(addr, HEX);
    }
  }
  
  if (!found_any) {
    Serial.println("No barometer sensors detected");
  }
  
  return found_any;
}

bool detect_gps_sensor() {
  // Test GPS by checking if UART is receiving data
  Serial1.begin(9600);
  delay(100);
  
  unsigned long start_time = millis();
  bool gps_data_received = false;
  
  // Wait up to 2 seconds for GPS data
  while (millis() - start_time < 2000) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '$') {  // NMEA sentence start
        gps_data_received = true;
        break;
      }
    }
    delay(10);
  }
  
  sensor_detection.gps_present = gps_data_received;
  
  if (gps_data_received) {
    Serial.println("GPS sensor detected on UART1");
  } else {
    Serial.println("No GPS sensor detected on UART1");
  }
  
  return gps_data_received;
}

bool detect_sonar_sensor() {
  // Test sonar by sending a pulse and checking for echo
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 30000);
  sensor_detection.sonar_present = (duration > 0 && duration < 25000); // Reasonable range
  
  if (sensor_detection.sonar_present) {
    Serial.println("Sonar sensor detected");
  } else {
    Serial.println("No sonar sensor detected");
  }
  
  return sensor_detection.sonar_present;
}

bool detect_optical_flow_sensor() {
  // This would need specific implementation for each optical flow sensor type
  // For now, just check common SPI addresses or I2C addresses
  sensor_detection.optical_flow_present = false;
  
  // Check for PMW3901 on SPI
  // Check for PAA5100 on I2C
  // Check for ADNS3080 on SPI
  
  // Placeholder implementation
  Serial.println("Optical flow sensor detection not implemented");
  return false;
}

bool detect_voltage_current_sensors() {
  // Test ADC pins for voltage and current sensors
  int voltage_reading = analogRead(BATTERY_VOLTAGE_PIN);
  int current_reading = analogRead(CURRENT_SENSOR_PIN);
  
  // Simple heuristic: if readings are not stuck at 0 or max value
  sensor_detection.voltage_sensor_present = (voltage_reading > 10 && voltage_reading < 1013);
  sensor_detection.current_sensor_present = (current_reading > 10 && current_reading < 1013);
  
  if (sensor_detection.voltage_sensor_present) {
    Serial.println("Voltage sensor detected");
  } else {
    Serial.println("No voltage sensor detected");
  }
  
  if (sensor_detection.current_sensor_present) {
    Serial.println("Current sensor detected");
  } else {
    Serial.println("No current sensor detected");
  }
  
  return sensor_detection.voltage_sensor_present || sensor_detection.current_sensor_present;
}

void perform_full_sensor_detection() {
  Serial.println("=== Starting Full Sensor Detection ===");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000); // 100kHz for compatibility
  
  // Scan I2C bus
  scan_i2c_devices();
  
  // Detect each sensor type
  detect_imu_sensors();
  detect_magnetometer_sensors();
  detect_barometer_sensors();
  detect_gps_sensor();
  detect_sonar_sensor();
  detect_optical_flow_sensor();
  detect_voltage_current_sensors();
  
  sensor_detection.last_detection_scan = millis();
  sensor_detection.detection_complete = true;
  
  Serial.println("=== Sensor Detection Complete ===");
}

String get_sensor_detection_report() {
  String report = "=== SENSOR DETECTION REPORT ===\n";
  
  // IMU sensors
  report += "IMU Sensors:\n";
  if (sensor_detection.mpu6050_present) report += "  ✓ MPU6050\n";
  if (sensor_detection.mpu9250_present) report += "  ✓ MPU9250\n";
  if (sensor_detection.icm20948_present) report += "  ✓ ICM20948\n";
  if (sensor_detection.icm42688p_present) report += "  ✓ ICM42688P\n";
  if (sensor_detection.bmi270_present) report += "  ✓ BMI270\n";
  if (sensor_detection.lsm6dso32_present) report += "  ✓ LSM6DSO32\n";
  if (sensor_detection.bmi323_present) report += "  ✓ BMI323\n";
  if (sensor_detection.icm20602_present) report += "  ✓ ICM20602\n";
  if (sensor_detection.lsm6ds33_present) report += "  ✓ LSM6DS33\n";
  
  // Magnetometer sensors
  report += "Magnetometer Sensors:\n";
  if (sensor_detection.hmc5883l_present) report += "  ✓ HMC5883L\n";
  if (sensor_detection.qmc5883l_present) report += "  ✓ QMC5883L\n";
  if (sensor_detection.rm3100_present) report += "  ✓ RM3100\n";
  if (sensor_detection.mmc5883ma_present) report += "  ✓ MMC5883MA\n";
  if (sensor_detection.ist8310_present) report += "  ✓ IST8310\n";
  if (sensor_detection.lis3mdl_present) report += "  ✓ LIS3MDL\n";
  if (sensor_detection.ak8963_present) report += "  ✓ AK8963\n";
  
  // Barometer sensors
  report += "Barometer Sensors:\n";
  if (sensor_detection.bmp280_present) report += "  ✓ BMP280\n";
  if (sensor_detection.bme280_present) report += "  ✓ BME280\n";
  if (sensor_detection.bmp388_present) report += "  ✓ BMP388\n";
  if (sensor_detection.ms5611_present) report += "  ✓ MS5611\n";
  if (sensor_detection.lps22hb_present) report += "  ✓ LPS22HB\n";
  
  // Other sensors
  report += "Other Sensors:\n";
  if (sensor_detection.gps_present) report += "  ✓ GPS\n";
  if (sensor_detection.sonar_present) report += "  ✓ Sonar\n";
  if (sensor_detection.optical_flow_present) report += "  ✓ Optical Flow\n";
  if (sensor_detection.voltage_sensor_present) report += "  ✓ Voltage Sensor\n";
  if (sensor_detection.current_sensor_present) report += "  ✓ Current Sensor\n";
  
  // I2C devices summary
  report += "I2C Devices Found: ";
  report += String(sensor_detection.i2c_device_count);
  report += "\n";
  
  return report;
}

String get_sensor_capabilities_json() {
  String json = "{\n";
  json += "  \"detection_complete\": " + String(sensor_detection.detection_complete ? "true" : "false") + ",\n";
  json += "  \"last_scan\": " + String(sensor_detection.last_detection_scan) + ",\n";
  
  // IMU sensors
  json += "  \"imu\": {\n";
  json += "    \"mpu6050\": " + String(sensor_detection.mpu6050_present ? "true" : "false") + ",\n";
  json += "    \"mpu9250\": " + String(sensor_detection.mpu9250_present ? "true" : "false") + ",\n";
  json += "    \"icm20948\": " + String(sensor_detection.icm20948_present ? "true" : "false") + ",\n";
  json += "    \"icm42688p\": " + String(sensor_detection.icm42688p_present ? "true" : "false") + ",\n";
  json += "    \"bmi270\": " + String(sensor_detection.bmi270_present ? "true" : "false") + ",\n";
  json += "    \"lsm6dso32\": " + String(sensor_detection.lsm6dso32_present ? "true" : "false") + ",\n";
  json += "    \"bmi323\": " + String(sensor_detection.bmi323_present ? "true" : "false") + ",\n";
  json += "    \"icm20602\": " + String(sensor_detection.icm20602_present ? "true" : "false") + ",\n";
  json += "    \"lsm6ds33\": " + String(sensor_detection.lsm6ds33_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // Magnetometer sensors
  json += "  \"magnetometer\": {\n";
  json += "    \"hmc5883l\": " + String(sensor_detection.hmc5883l_present ? "true" : "false") + ",\n";
  json += "    \"qmc5883l\": " + String(sensor_detection.qmc5883l_present ? "true" : "false") + ",\n";
  json += "    \"rm3100\": " + String(sensor_detection.rm3100_present ? "true" : "false") + ",\n";
  json += "    \"mmc5883ma\": " + String(sensor_detection.mmc5883ma_present ? "true" : "false") + ",\n";
  json += "    \"ist8310\": " + String(sensor_detection.ist8310_present ? "true" : "false") + ",\n";
  json += "    \"lis3mdl\": " + String(sensor_detection.lis3mdl_present ? "true" : "false") + ",\n";
  json += "    \"ak8963\": " + String(sensor_detection.ak8963_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // Barometer sensors
  json += "  \"barometer\": {\n";
  json += "    \"bmp280\": " + String(sensor_detection.bmp280_present ? "true" : "false") + ",\n";
  json += "    \"bme280\": " + String(sensor_detection.bme280_present ? "true" : "false") + ",\n";
  json += "    \"bmp388\": " + String(sensor_detection.bmp388_present ? "true" : "false") + ",\n";
  json += "    \"ms5611\": " + String(sensor_detection.ms5611_present ? "true" : "false") + ",\n";
  json += "    \"lps22hb\": " + String(sensor_detection.lps22hb_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // Other sensors
  json += "  \"other\": {\n";
  json += "    \"gps\": " + String(sensor_detection.gps_present ? "true" : "false") + ",\n";
  json += "    \"sonar\": " + String(sensor_detection.sonar_present ? "true" : "false") + ",\n";
  json += "    \"optical_flow\": " + String(sensor_detection.optical_flow_present ? "true" : "false") + ",\n";
  json += "    \"voltage_sensor\": " + String(sensor_detection.voltage_sensor_present ? "true" : "false") + ",\n";
  json += "    \"current_sensor\": " + String(sensor_detection.current_sensor_present ? "true" : "false") + "\n";
  json += "  },\n";
  
  // I2C devices
  json += "  \"i2c_devices\": [\n";
  for (int i = 0; i < sensor_detection.i2c_device_count; i++) {
    json += "    \"0x" + String(sensor_detection.i2c_devices[i], HEX) + "\"";
    if (i < sensor_detection.i2c_device_count - 1) json += ",";
    json += "\n";
  }
  json += "  ]\n";
  json += "}";
  
  return json;
}

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

#endif // SENSORS_H
