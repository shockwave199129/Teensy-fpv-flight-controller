#include "dynamic_filtering.h"
#include "sensors.h"
#include <math.h>
#include <arm_math.h>

#ifndef MAHONY_TWO_KP
#define MAHONY_TWO_KP 2.0f
#endif
#ifndef MAHONY_TWO_KI
#define MAHONY_TWO_KI 0.0f
#endif

DynamicFilteringSystem::DynamicFilteringSystem() {
  filtering_active = false;
  noise_level = 0.0f;
  last_analysis_time = 0;
  filter_update_rate = 50; // 50ms update rate
}

void DynamicFilteringSystem::init() {
  Serial.println("Initializing Dynamic Filtering System...");
  
  // Initialize spectral analysis buffers
  for (int i = 0; i < 256; i++) {
    fft_buffer[i] = 0.0f;
  }
  
  for (int i = 0; i < 128; i++) {
    frequency_bins[i] = 0.0f;
  }
  
  for (int i = 0; i < 8; i++) {
    peak_frequencies[i] = 0.0f;
    peak_amplitudes[i] = 0.0f;
  }
  
  num_peaks = 0;
  analysis_complete = false;
  
  // Initialize adaptive notch filters
  for (int i = 0; i < 4; i++) {
    adaptive_notch[i].center_freq_hz = 180.0f; // Default motor frequency
    adaptive_notch[i].bandwidth_hz = 20.0f;
    adaptive_notch[i].q_factor = 10.0f;
    adaptive_notch[i].enabled = true;
    target_frequencies[i] = 180.0f;
    
    // Calculate initial filter coefficients
    calculate_notch_coefficients(&adaptive_notch[i], 2000.0f); // 2kHz sample rate
  }
  
  filter_q_factor = 10.0f;
  auto_tune_enabled = true;
  noise_floor = 0.1f;
  signal_to_noise_ratio = 10.0f;
  
  filtering_active = true;
  Serial.println("Dynamic filtering initialized with 4 adaptive notch filters");
  
  arm_rfft_fast_init_f32(&fft_instance, 256);
}

void DynamicFilteringSystem::update(SensorData& sensors, RcData& rc) {
  if (!filtering_active) return;
  
  unsigned long current_time = millis();
  
  if (current_time - last_analysis_time < filter_update_rate) {
    return;
  }
  
  last_analysis_time = current_time;
  
  // Analyze gyro data for noise frequencies
  analyze_gyro_spectrum(sensors);
  
  // Update filter frequencies based on analysis
  update_filter_frequencies();
  
  // Apply dynamic filtering to sensor data
  apply_dynamic_filtering(sensors);
  
  // Update noise level estimation
  update_noise_level_estimation(sensors);
}

void DynamicFilteringSystem::analyze_gyro_spectrum(const SensorData& sensors) {
  static int buffer_index = 0;
  static bool buffer_full = false;
  
  // Collect gyro data for FFT analysis
  float gyro_magnitude = sqrt(pow(sensors.imu.gyro_x, 2) + 
                             pow(sensors.imu.gyro_y, 2) + 
                             pow(sensors.imu.gyro_z, 2));
  
  fft_buffer[buffer_index] = gyro_magnitude;
  buffer_index = (buffer_index + 1) % 256;
  
  if (buffer_index == 0) {
    buffer_full = true;
  }
  
  if (!buffer_full) {
    return; // Need full buffer for analysis
  }
  
  // Perform FFT-based peak detection
  detect_frequency_peaks_fft();
  
  analysis_complete = true;
}

void DynamicFilteringSystem::detect_frequency_peaks_fft() {
  // Perform 256-point real FFT at 2 kHz sample rate
  arm_rfft_fast_f32(&fft_instance, fft_buffer, frequency_bins, 0);

  // Convert complex bins to magnitude (only first 128 bins)
  float max_amp = 0;
  for (int i=1;i<128;i++) { // skip DC
    float real = frequency_bins[2*i];
    float imag = frequency_bins[2*i+1];
    float mag = sqrtf(real*real + imag*imag);
    frequency_bins[i] = mag;
    if (mag>max_amp) max_amp = mag;
  }

  num_peaks = 0;
  for(int i=3;i<128 && num_peaks<8;i++) { // bin 3 ~ 50Hz (2kHz/256*3)
    float freq = (2000.0f/256.0f) * i;
    if(freq < 50 || freq > 400) continue;
    float mag = frequency_bins[i];
    // simple local maxima
    if (mag > frequency_bins[i-1] && mag > frequency_bins[i+1] && mag > (0.2f*max_amp)) {
      peak_frequencies[num_peaks] = freq;
      peak_amplitudes[num_peaks] = mag;
      num_peaks++;
    }
  }
}

void DynamicFilteringSystem::update_filter_frequencies() {
  if (!auto_tune_enabled || num_peaks == 0) {
    return;
  }
  
  // Assign strongest peaks to filters
  for (int filter = 0; filter < 4 && filter < num_peaks; filter++) {
    float new_frequency = peak_frequencies[filter];
    
    // Only update if frequency changed significantly
    if (abs(new_frequency - target_frequencies[filter]) > 5.0f) {
      target_frequencies[filter] = new_frequency;
      adaptive_notch[filter].center_freq_hz = new_frequency;
      
      // Recalculate filter coefficients
      calculate_notch_coefficients(&adaptive_notch[filter], 2000.0f);
      
      Serial.print("Filter ");
      Serial.print(filter);
      Serial.print(" tuned to ");
      Serial.print(new_frequency);
      Serial.println(" Hz");
    }
  }
}

void DynamicFilteringSystem::apply_dynamic_filtering(SensorData& sensors) {
  // Apply notch filters to gyro data
  for (int filter = 0; filter < 4; filter++) {
    if (adaptive_notch[filter].enabled) {
      sensors.imu.gyro_x = apply_notch_filter(&adaptive_notch[filter], sensors.imu.gyro_x);
    }
  }
  
  // Apply different filters to pitch and yaw (would implement separate filter banks)
  for (int filter = 0; filter < 4; filter++) {
    if (adaptive_notch[filter].enabled) {
      sensors.imu.gyro_y = apply_notch_filter(&adaptive_notch[filter], sensors.imu.gyro_y);
      sensors.imu.gyro_z = apply_notch_filter(&adaptive_notch[filter], sensors.imu.gyro_z);
    }
  }
}

void DynamicFilteringSystem::update_noise_level_estimation(const SensorData& sensors) {
  // Calculate current noise level
  static float noise_history[10];
  static int noise_index = 0;
  static bool noise_buffer_full = false;
  
  float current_noise = sqrt(pow(sensors.imu.gyro_x, 2) + 
                           pow(sensors.imu.gyro_y, 2) + 
                           pow(sensors.imu.gyro_z, 2));
  
  noise_history[noise_index] = current_noise;
  noise_index = (noise_index + 1) % 10;
  
  if (noise_index == 0) {
    noise_buffer_full = true;
  }
  
  if (noise_buffer_full) {
    // Calculate average noise level
    float total_noise = 0.0f;
    for (int i = 0; i < 10; i++) {
      total_noise += noise_history[i];
    }
    
    noise_level = total_noise / 10.0f;
    
    // Update signal-to-noise ratio
    if (noise_level > 0.01f) {
      signal_to_noise_ratio = 20.0f * log10(1.0f / noise_level);
    }
  }
}

bool DynamicFilteringSystem::is_filtering_active() {
  return filtering_active;
}

float DynamicFilteringSystem::get_noise_level() {
  return noise_level;
}

void DynamicFilteringSystem::set_motor_notch_frequency(uint8_t motorIndex, float freq_hz) {
  if (motorIndex >= 4) return;
  if (fabsf(freq_hz - target_frequencies[motorIndex]) < 1.0f) return; // ignore tiny changes
  target_frequencies[motorIndex] = freq_hz;
  adaptive_notch[motorIndex].center_freq_hz = freq_hz;
  calculate_notch_coefficients(&adaptive_notch[motorIndex], 2000.0f);
}

// Stub implementations for filter functions

static MahonyFilter mahony = {1,0,0,0,0,0,0,false,0};

void init_mahony_filter() {
  mahony.q0 = 1.0f; mahony.q1 = mahony.q2 = mahony.q3 = 0.0f;
  mahony.integralFBx = mahony.integralFBy = mahony.integralFBz = 0.0f;
  mahony.initialized = true;
  mahony.last_update = millis();
  Serial.println("Mahony filter initialised");
}

static VibratingFilter vib_filters[3];

void init_vibration_filters() {
  for (int i=0;i<3;i++) {
    vib_filters[i].cutoff_hz = 30.0f; // default cutoff
    float dt = 1.0f / 1000.0f;        // assume 1 kHz update for alpha calc
    vib_filters[i].alpha = dt / (dt + (1.0f/(2.0f*PI*vib_filters[i].cutoff_hz)));
    vib_filters[i].filtered_value = 0.0f;
    vib_filters[i].initialized = true;
  }
  Serial.println("Basic vibration LPFs configured");
}

void init_notch_filters() {
  // Actual notch filters are dynamically configured by DynamicFilteringSystem.
  Serial.println("init_notch_filters(): handled by DynamicFilteringSystem");
}

void calculate_notch_coefficients(NotchFilter* filter, float sample_rate) {
  // Standard biquad notch design (RBJ cookbook)
  float w0 = 2.0f * PI * filter->center_freq_hz / sample_rate;
  float alpha = sin(w0) / (2.0f * filter->q_factor);

  float b0 = 1.0f;
  float b1 = -2.0f * cos(w0);
  float b2 = 1.0f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cos(w0);
  float a2 = 1.0f - alpha;

  // Normalize coefficients
  filter->b0 = b0 / a0;
  filter->b1 = b1 / a0;
  filter->b2 = b2 / a0;
  filter->a1 = a1 / a0;
  filter->a2 = a2 / a0;

  // Reset state
  filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0.0f;
}

float apply_notch_filter(NotchFilter* filter, float input) {
  if (!filter->enabled) return input;

  float y = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
            - filter->a1 * filter->y1 - filter->a2 * filter->y2;

  filter->x2 = filter->x1;
  filter->x1 = input;
  filter->y2 = filter->y1;
  filter->y1 = y;

  return y;
}

void mahony_ahrs_update(float gx, float gy, float gz, float ax, float ay, float az,
                        float mx, float my, float mz, float dt) {
    // Based on MahonyAHRS algorithm (see Mahony 2011).
    // Inputs: gx,gy,gz in rad/s; ax..az & mx..mz in any consistent units; dt in seconds.

    if (!mahony.initialized) {
        init_mahony_filter();
    }

    // Normalise accelerometer measurement
    float recipNorm = sqrtf(ax*ax + ay*ay + az*az);
    if (recipNorm <= 0.0f) return; // invalid accel
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = sqrtf(mx*mx + my*my + mz*mz);
    if (recipNorm <= 0.0f) {
        // fall back to IMU-only update
        mahony_ahrs_update_imu(gx, gy, gz, ax, ay, az, dt);
        return;
    }
    recipNorm = 1.0f / recipNorm;
    mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

    // Reference direction of Earth's magnetic field
    float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;

    float hx = 2.0f * (mx * (0.5f - q2*q2 - q3*q3) + my * (q1*q2 - q0*q3) + mz * (q1*q3 + q0*q2));
    float hy = 2.0f * (mx * (q1*q2 + q0*q3) + my * (0.5f - q1*q1 - q3*q3) + mz * (q2*q3 - q0*q1));
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = 2.0f * (mx * (q1*q3 - q0*q2) + my * (q2*q3 + q0*q1) + mz * (0.5f - q1*q1 - q2*q2));

    // Estimated direction of gravity and magnetic field (in body frame)
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float wx = 2.0f * bx * (0.5f - q2*q2 - q3*q3) + 2.0f * bz * (q1*q3 - q0*q2);
    float wy = 2.0f * bx * (q1*q2 - q0*q3) + 2.0f * bz * (q0*q1 + q2*q3);
    float wz = 2.0f * bx * (q0*q2 + q1*q3) + 2.0f * bz * (0.5f - q1*q1 - q2*q2);

    // Error is cross product between estimated and measured direction of gravity + magnetic field
    float ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    float ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    float ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    // Integral feedback
    const float twoKi = MAHONY_TWO_KI;
    const float twoKp = MAHONY_TWO_KP;

    if (twoKi > 0.0f) {
        mahony.integralFBx += twoKi * ex * dt; // integral error scaled by Ki
        mahony.integralFBy += twoKi * ey * dt;
        mahony.integralFBz += twoKi * ez * dt;
        gx += mahony.integralFBx; // apply integral feedback
        gy += mahony.integralFBy;
        gz += mahony.integralFBz;
    }

    // Apply proportional feedback
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa  * gx + qc * gz - q3 * gy);
    q2 += (qa  * gy - qb * gz + q3 * gx);
    q3 += (qa  * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    recipNorm = 1.0f / recipNorm;
    mahony.q0 = q0 * recipNorm;
    mahony.q1 = q1 * recipNorm;
    mahony.q2 = q2 * recipNorm;
    mahony.q3 = q3 * recipNorm;
}

// Weak stub removed; real implementation is provided in sensors.cpp

float apply_vibration_filter(VibratingFilter* filter, float input, float dt) {
    if (!filter->initialized) {
        filter->filtered_value = input;
        filter->initialized = true;
        return input;
    }

    // Re-compute alpha if dt changed significantly (simple adaptive LPF)
    if (dt > 0.0001f) {
        float rc = 1.0f / (2.0f * PI * filter->cutoff_hz);
        filter->alpha = dt / (dt + rc);
    }

    filter->filtered_value += filter->alpha * (input - filter->filtered_value);
    return filter->filtered_value;
}

void detect_vibration_issues(SensorData* data) {
    // Very basic vibration metric: high-frequency component of Z-accel
    static float vib_level = 0.0f;
    float filtered = apply_vibration_filter(&vib_filters[2], data->imu.accel_z, 0.001f);
    float high_freq = fabsf(data->imu.accel_z - filtered);
    vib_level = 0.95f * vib_level + 0.05f * high_freq;

    if (vib_level > 5.0f) { // arbitrary threshold
        Serial.println("WARNING: Excessive vibration detected!");
    }
}

void DynamicFilteringSystem::feed_motor_rpm(const uint16_t* rpm) {
  // Convert RPM to first harmonic frequency (Hz) and update target freq array
  for(int m=0;m<4;m++) {
    rpm_harmonics[m] = rpm[m] / 60.0f; // fundamental
    if (fabsf(rpm_harmonics[m] - target_frequencies[m]) > 5.0f) {
      target_frequencies[m] = rpm_harmonics[m];
      adaptive_notch[m].center_freq_hz = rpm_harmonics[m];
      calculate_notch_coefficients(&adaptive_notch[m], 2000.0f);
    }
  }
} 