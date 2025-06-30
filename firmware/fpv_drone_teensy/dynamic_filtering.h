#pragma once

#include "config.h"
#include <arm_math.h>
#include <stdint.h>

// Forward declarations of DSP helper functions implemented in dynamic_filtering.cpp
struct NotchFilter;
void calculate_notch_coefficients(NotchFilter* filter, float sample_rate);
float apply_notch_filter(NotchFilter* filter, float input);

class DynamicFilteringSystem {
public:
    DynamicFilteringSystem();
    void init();
    void update(SensorData& sensors, RcData& rc);
    void apply_dynamic_filtering(SensorData& sensors);
    void feed_motor_rpm(const uint16_t* rpmArray);
    bool is_filtering_active();
    float get_noise_level();
    
    // Allow external modules (e.g., motor control) to update notch center freq based on RPM
    void set_motor_notch_frequency(uint8_t motorIndex, float freq_hz);
    void update_noise_level_estimation(const SensorData& sensors);
private:
    void analyze_gyro_spectrum(const SensorData& sensors);
    void detect_frequency_peaks_fft();
    void update_filter_frequencies();
    // Helpers implemented as global functions but exposed here via inline wrappers below
    inline void calculate_notch_coefficients(NotchFilter* f, float sr) {
        ::calculate_notch_coefficients(f, sr);
    }
    inline float apply_notch_filter(NotchFilter* f, float in) {
        return ::apply_notch_filter(f, in);
    }

    // Data members copied from previous cpp
    bool filtering_active; float noise_level; unsigned long last_analysis_time; uint16_t filter_update_rate;
    float fft_buffer[256]; float frequency_bins[256];
    float peak_frequencies[8]; float peak_amplitudes[8]; uint8_t num_peaks; bool analysis_complete;
    NotchFilter adaptive_notch[4]; float target_frequencies[4]; float filter_q_factor; bool auto_tune_enabled;
    float noise_floor; float signal_to_noise_ratio; float rpm_harmonics[4];

    arm_rfft_fast_instance_f32 fft_instance;
}; 