#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Sensor Health Status Enumeration (for sensor redundancy)
typedef enum {
  SENSOR_HEALTHY = 0,
  SENSOR_DEGRADED = 1,
  SENSOR_FAILED = 2,
  SENSOR_UNAVAILABLE = 3
} SensorHealthStatus;

// Sensor Status Structure (for sensor redundancy)
struct SensorStatus {
  SensorHealthStatus imu_health;
  SensorHealthStatus gps_health;
  SensorHealthStatus mag_health;
  SensorHealthStatus baro_health;
  float quality_score;
  bool redundancy_active;
  unsigned long last_update;
  bool primary_imu_ok;
  bool secondary_imu_ok;
  bool gps_ok;
  bool mag_ok;
  bool baro_ok;
  bool synthetic_gps_available;
  bool synthetic_mag_available;
  bool synthetic_baro_available;
};
#include "constants.h"

// Pin Definitions (based on documentation)
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3
#define MOTOR3_PIN 4
#define MOTOR4_PIN 5
#define LED_PIN 6
#define SONAR_TRIG_PIN 22
#define SONAR_ECHO_PIN 23
#define BATTERY_VOLTAGE_PIN A0
#define CURRENT_SENSOR_PIN A1

// I2C Pins (Teensy 4.1 default)
#define SDA_PIN 18
#define SCL_PIN 19

// UART Pins
#define GPS_RX_PIN 0
#define GPS_TX_PIN 1
#define RC_RX_PIN 7
#define RC_TX_PIN 8

// LED Configuration
#define NUM_LEDS 8
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

// RC Receiver Protocols
enum RcProtocol {
  RC_PROTOCOL_PPM = 0,
  RC_PROTOCOL_IBUS,
  RC_PROTOCOL_SBUS,
  RC_PROTOCOL_ELRS
};

// ESC Protocols
enum EscProtocol {
  ESC_PROTOCOL_PWM = 0,
  ESC_PROTOCOL_ONESHOT125,
  ESC_PROTOCOL_ONESHOT42,
  ESC_PROTOCOL_MULTISHOT,
  ESC_PROTOCOL_DSHOT150,
  ESC_PROTOCOL_DSHOT300,
  ESC_PROTOCOL_DSHOT600,
  ESC_PROTOCOL_DSHOT1200
};

// ESC Firmware Types
enum EscFirmwareType {
  ESC_FIRMWARE_UNKNOWN = 0,
  ESC_FIRMWARE_BLHELI_S,
  ESC_FIRMWARE_BLHELI_32,
  ESC_FIRMWARE_BLUEJAY,
  ESC_FIRMWARE_AM32,
  ESC_FIRMWARE_ESCAPE32,
  ESC_FIRMWARE_GENERIC
};

// ESC Firmware Capabilities
struct EscFirmwareCapabilities {
  bool supports_rpm_telemetry;
  bool supports_temperature_telemetry;
  bool supports_voltage_telemetry;
  bool supports_current_telemetry;
  bool supports_bidirectional_dshot;
  bool supports_variable_pwm_frequency;
  bool supports_3d_mode;
  bool supports_beacon;
  bool supports_led_control;
  bool supports_settings_via_dshot;
  bool supports_firmware_update;
  uint32_t max_rpm;
  uint8_t pole_count;
};

// BLHeli-specific settings
struct BLHeliSettings {
  uint8_t startup_power;           // 0-255
  uint8_t temperature_limit;       // Celsius
  uint8_t current_limit;           // Amps
  uint8_t timing_advance;          // Degrees
  uint8_t pwm_frequency;           // kHz
  bool comp_pwm;                   // Complementary PWM
  bool motor_direction;            // Normal/Reversed
  bool low_rpm_power_protect;
  bool brake_on_stop;
  uint8_t demag_compensation;      // Off/Low/High
  uint8_t commutation_timing;      // Low/Medium/High
};

// DShot Commands
enum DshotCommand {
  DSHOT_CMD_MOTOR_STOP = 0,
  DSHOT_CMD_BEEP1 = 1,
  DSHOT_CMD_BEEP2 = 2,
  DSHOT_CMD_BEEP3 = 3,
  DSHOT_CMD_BEEP4 = 4,
  DSHOT_CMD_BEEP5 = 5,
  DSHOT_CMD_ESC_INFO = 6,
  DSHOT_CMD_ESC_CALIBRATION = 7,
  DSHOT_CMD_SPIN_DIRECTION_1 = 8,
  DSHOT_CMD_SPIN_DIRECTION_2 = 9,
  DSHOT_CMD_3D_MODE_OFF = 10,
  DSHOT_CMD_3D_MODE_ON = 11,
  DSHOT_CMD_SETTINGS_REQUEST = 12,
  DSHOT_CMD_SAVE_SETTINGS = 13,
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
  DSHOT_CMD_LED0_ON = 22,
  DSHOT_CMD_LED1_ON = 23,
  DSHOT_CMD_LED2_ON = 24,
  DSHOT_CMD_LED3_ON = 25,
  DSHOT_CMD_LED0_OFF = 26,
  DSHOT_CMD_LED1_OFF = 27,
  DSHOT_CMD_LED2_OFF = 28,
  DSHOT_CMD_LED3_OFF = 29,
  DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,
  DSHOT_CMD_SILENT_MODE_ON_OFF = 31,
  DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 32,
  DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY = 33
};

// Flight Modes
enum FlightMode {
  FLIGHT_MODE_MANUAL = 0,
  FLIGHT_MODE_STABILIZE,
  FLIGHT_MODE_ALTITUDE_HOLD,
  FLIGHT_MODE_POSITION_HOLD,
  FLIGHT_MODE_RETURN_TO_HOME,
  FLIGHT_MODE_HEADLESS
};

// LED Status
enum LedStatus {
  LED_DISARMED = 0,
  LED_ARMED,
  LED_GPS_LOCK,
  LED_RTH_ACTIVE,
  LED_LOW_BATTERY,
  LED_ERROR
};

// PWM Constants - Enhanced for multiple protocols
#define ESC_MIN_PWM 1000
#define ESC_MAX_PWM 2000
#define ESC_ARM_PWM 1000

// OneShot125 Constants (125us - 250us)
#define ONESHOT125_MIN 125
#define ONESHOT125_MAX 250

// OneShot42 Constants (42us - 84us)  
#define ONESHOT42_MIN 42
#define ONESHOT42_MAX 84

// Multishot Constants (5us - 25us)
#define MULTISHOT_MIN 5
#define MULTISHOT_MAX 25

// DShot timing constants (in nanoseconds)
#define DSHOT150_BIT_TIME 6667   // 150kHz
#define DSHOT300_BIT_TIME 3333   // 300kHz
#define DSHOT600_BIT_TIME 1667   // 600kHz
#define DSHOT1200_BIT_TIME 833   // 1200kHz

// DShot bit timing
#define DSHOT_BIT_0_HIGH_TIME_FACTOR 0.37  // ~37% high for bit 0
#define DSHOT_BIT_1_HIGH_TIME_FACTOR 0.74  // ~74% high for bit 1

// Use constants from constants.h for all other values
// Do not redefine: VOLTAGE_SCALE, CURRENT_SCALE, MAX_LOOP_TIME_US
// Do not redefine: DEFAULT_*_KP/KI/KD values, MAHONY_*, etc.

// Data Structures
struct IMUData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float temp;
  bool healthy;
  bool calibrated;
  unsigned long last_update;
};

struct MagnetometerData {
  float mag_x, mag_y, mag_z;
  float heading;
  bool healthy;
  bool calibrated;
  unsigned long last_update;
};

struct BarometerData {
  float pressure;
  float altitude;
  float temperature;
  bool healthy;
  bool calibrated;
  unsigned long last_update;
};

struct GPSData {
  float latitude, longitude;
  float altitude;
  float speed;
  float heading;
  int satellites;
  bool fix;
  bool healthy;
  bool calibrated;
  unsigned long last_update;
};

struct SonarData {
  float distance;
  bool healthy;
  bool calibrated;
  unsigned long last_update;
};

struct SensorData {
  IMUData imu;
  MagnetometerData mag;
  BarometerData baro;
  GPSData gps;
  SonarData sonar;
  float battery_voltage;
  float current;
  
  // Fused orientation
  float roll, pitch, yaw;
  float altitude_fused;
};

struct RcData {
  int throttle, roll, pitch, yaw;
  int aux1, aux2, aux3, aux4;
  bool signal_valid;
  unsigned long last_update;
};

struct PIDOutput {
  float roll, pitch, yaw, throttle;
};

struct MotorOutput {
  int motor1, motor2, motor3, motor4;
};

struct FlightState {
  bool armed;
  FlightMode flight_mode;
  float battery_voltage;
  bool gps_fix;
  float altitude;
  float home_lat, home_lon;
  bool home_set;
};

struct PIDGains {
  float kp, ki, kd;
  float integral;
  float last_error;
  float output_min, output_max;
};

struct EscConfig {
  EscProtocol protocol;
  EscFirmwareType firmware_type;
  EscFirmwareCapabilities capabilities;
  bool telemetry_enabled;
  bool bidirectional_enabled;
  int motor_direction[4];  // 1 = normal, -1 = reversed
  bool motor_enabled[4];   // Enable/disable individual motors
  uint16_t idle_throttle_percent; // 0-100
  uint16_t min_throttle_percent;  // 0-100 
  uint16_t max_throttle_percent;  // 0-100
  bool beep_on_startup;
  
  // Firmware-specific settings
  BLHeliSettings blheli_settings[4];  // Per-motor BLHeli settings
  bool auto_detect_firmware;         // Auto-detect ESC firmware type
  uint8_t motor_poles[4];            // Motor pole count per motor
  uint16_t motor_kv[4];              // Motor KV rating per motor
};

struct MotorTelemetry {
  uint16_t rpm;
  uint8_t temperature;  // Celsius
  uint16_t voltage;     // mV
  uint16_t current;     // mA
  uint16_t consumption;  // mAh
  bool data_valid;
  unsigned long last_update;
};

struct NotchFilter {
  float center_freq_hz;
  float bandwidth_hz;
  float q_factor;
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
  bool enabled;
};

struct RPMFilter {
  float motor_freq_hz[4];      // Current motor frequencies
  float notch_freq_hz[4];      // Calculated notch frequencies
  bool enabled[4];             // Per-motor enable
  NotchFilter rpm_notch[4][3]; // 3 harmonics per motor
  float rpm_lpf_hz;            // RPM low-pass filter
};

// All remaining structures follow similar pattern...
// Using constants from constants.h instead of redefining

// Additional Required Structures

// IMU Type Definitions (for wide sensor support)
enum IMUType {
  IMU_MPU6050 = 0,
  IMU_MPU9250,
  IMU_ICM20948,
  IMU_ICM42688P,
  IMU_BMI270,
  IMU_LSM6DSO32,
  IMU_BMI323,
  IMU_ICM20602,
  IMU_LSM6DS33
};

// Dual IMU Data Structures
struct DualIMUConfig {
  IMUType primary_type;
  IMUType secondary_type;
  uint8_t primary_address;
  uint8_t secondary_address;
  bool cross_validation_enabled;
  float divergence_threshold;     // Degrees for attitude divergence
  float gyro_divergence_threshold; // deg/s for gyro divergence
  bool auto_failover_enabled;
  unsigned long validation_timeout_ms;
};

struct IMUValidation {
  bool primary_healthy;
  bool secondary_healthy;
  bool cross_validation_passed;
  float attitude_divergence;
  float gyro_divergence;
  unsigned long last_validation;
  uint8_t primary_failures;
  uint8_t secondary_failures;
  bool using_primary;
};

// Dynamic Filtering System
struct SpectralAnalysis {
  float fft_buffer[256];          // FFT input buffer
  float frequency_bins[128];      // Frequency domain output
  float peak_frequencies[8];      // Detected noise peaks
  float peak_amplitudes[8];       // Peak amplitudes
  uint8_t num_peaks;
  bool analysis_complete;
  unsigned long last_analysis;
};

struct DynamicFilter {
  NotchFilter adaptive_notch[4];  // Auto-tuning notch filters
  float target_frequencies[4];    // Frequencies to filter
  float filter_q_factor;
  bool auto_tune_enabled;
  unsigned long last_update;
  float noise_floor;
  float signal_to_noise_ratio;
};

// Optical Flow Sensor Configuration
struct OpticalFlowData {
  float velocity_x, velocity_y;   // m/s
  float displacement_x, displacement_y; // Integrated position
  uint8_t surface_quality;        // 0-255 surface tracking quality
  bool data_valid;
  unsigned long last_update;
};

// Enhanced Magnetometer Options
enum MagnetometerType {
  MAG_HMC5883L = 0,
  MAG_QMC5883L,
  MAG_RM3100,
  MAG_MMC5883MA,
  MAG_IST8310,
  MAG_LIS3MDL,
  MAG_AK8963
};

// Advanced Flight Modes
enum AdvancedFlightMode {
  ADV_FLIGHT_MODE_ACRO_PLUS = 10,    // Rate mode with auto-recovery
  ADV_FLIGHT_MODE_SPORT,             // High performance mode
  ADV_FLIGHT_MODE_CINEMATIC,         // Ultra-smooth for video
  ADV_FLIGHT_MODE_GPS_RESCUE,        // Advanced RTH with obstacles
  ADV_FLIGHT_MODE_TURTLE,            // Upside-down recovery
  ADV_FLIGHT_MODE_LAUNCH_ASSIST,     // Automatic launch detection
  ADV_FLIGHT_MODE_LAND_ASSIST        // Automatic landing assistance
};

// Phase Detection for Dynamic Filtering
enum FlightPhase {
  PHASE_GROUND = 0,
  PHASE_TAKEOFF,
  PHASE_HOVER,
  PHASE_FORWARD_FLIGHT,
  PHASE_AGGRESSIVE_MANEUVERS,
  PHASE_LANDING
};

// RC Channel Functions - for mapping channels to specific functions
enum ChannelFunction {
  CHAN_FUNC_THROTTLE = 0,
  CHAN_FUNC_ROLL,
  CHAN_FUNC_PITCH,
  CHAN_FUNC_YAW,
  CHAN_FUNC_ARM_DISARM,
  CHAN_FUNC_FLIGHT_MODE,
  CHAN_FUNC_RTH,
  CHAN_FUNC_ALTITUDE_HOLD,
  CHAN_FUNC_POSITION_HOLD,
  CHAN_FUNC_HEADLESS_MODE,
  CHAN_FUNC_TURTLE_MODE,
  CHAN_FUNC_BEEPER,
  CHAN_FUNC_LED_CONTROL,
  CHAN_FUNC_CAMERA_TILT,
  CHAN_FUNC_RATE_PROFILE,
  CHAN_FUNC_GPS_RESCUE,
  CHAN_FUNC_LAUNCH_ASSIST,
  CHAN_FUNC_BLACKBOX,
  CHAN_FUNC_NONE,
  CHAN_FUNC_COUNT
};

// Switch Positions for 2-pos and 3-pos switches
enum SwitchPosition {
  SWITCH_LOW = 0,    // < 1300
  SWITCH_MID,        // 1300-1700
  SWITCH_HIGH        // > 1700
};

// After enum SwitchPosition definition
#define SWITCH_POS_LOW SWITCH_LOW
#define SWITCH_POS_MIDDLE SWITCH_MID
#define SWITCH_POS_HIGH SWITCH_HIGH

// Channel Mapping Configuration
struct ChannelMapping {
  uint8_t channel_number;          // 1-16 (0 = disabled)
  ChannelFunction function;        // What this channel controls
  bool reversed;                   // Reverse channel direction
  uint16_t min_value;             // Minimum channel value (default 1000)
  uint16_t center_value;          // Center value (default 1500)
  uint16_t max_value;             // Maximum channel value (default 2000)
  uint16_t deadband;              // Deadband around center (default 10)
  
  // For switch channels
  bool is_switch;                 // True if this is a switch channel
  uint8_t switch_positions;       // 2 or 3 position switch
  uint16_t switch_thresholds[2];  // Thresholds for switch positions
};

// RC Configuration Structure
struct RcConfig {
  RcProtocol protocol;
  uint8_t channel_count;           // Number of active channels (4-16)
  ChannelMapping channels[16];     // Mapping for up to 16 channels
  
  // Failsafe settings
  bool failsafe_enabled;
  uint16_t failsafe_throttle;      // Failsafe throttle value
  FlightMode failsafe_mode;        // Mode to enter on failsafe
  bool rth_on_failsafe;           // RTH on signal loss
  
  // Arming settings
  bool stick_arming_enabled;       // Enable stick arming
  bool switch_arming_enabled;      // Enable switch arming
  uint8_t arm_channel;            // Channel for arm/disarm switch
  uint16_t arm_sequence_time_ms;   // Time to hold sticks for arm/disarm
  
  // Rate profiles
  uint8_t current_rate_profile;    // 0-2
  uint8_t rate_profile_channel;    // Channel to switch rate profiles
};

// Rate Profile for different flight characteristics
struct RateProfile {
  char name[16];                   // Profile name
  
  // Rates (degrees per second)
  float max_roll_rate;
  float max_pitch_rate;
  float max_yaw_rate;
  
  // Expo curves (0.0-1.0)
  float roll_expo;
  float pitch_expo;
  float yaw_expo;
  
  // RC smoothing
  float rc_smoothing_factor;       // 0.0-1.0
  
  // Throttle curve
  float throttle_expo;
  float throttle_mid;              // 0.0-1.0
};

// Function Switch Configurations
struct FunctionSwitches {
  uint8_t arm_switch_channel;      // Channel for arm/disarm
  SwitchPosition arm_position;     // Switch position to arm
  
  uint8_t mode_switch_channel;     // Flight mode switch channel
  FlightMode mode_positions[3];    // Modes for low/mid/high positions
  
  uint8_t aux_switch_channels[8];  // Additional auxiliary switches
  ChannelFunction aux_functions[8]; // Functions for aux switches
  
  // Emergency functions
  uint8_t rth_channel;            // Return to home channel
  SwitchPosition rth_position;    // Switch position for RTH
  
  uint8_t turtle_channel;         // Turtle mode channel
  uint8_t beeper_channel;         // Beeper activation channel
  
  SwitchPosition arm_disarm;     // Current arm/disarm switch position
  SwitchPosition flight_mode;    // Current flight mode switch position
  SwitchPosition beeper;         // Current beeper switch position
  SwitchPosition rth;            // Current RTH switch position
};

// Enhanced RC Data with channel mapping
struct EnhancedRcData {
  // Raw channel values (1000-2000)
  uint16_t raw_channels[16];
  uint8_t channel_count;
  
  // Mapped control values
  float throttle;                 // 0.0-1.0
  float roll;                     // -1.0 to 1.0
  float pitch;                    // -1.0 to 1.0
  float yaw;                      // -1.0 to 1.0
  
  // Switch states
  bool armed;
  FlightMode flight_mode;
  bool rth_activated;
  bool altitude_hold_enabled;
  bool position_hold_enabled;
  bool headless_mode;
  bool turtle_mode;
  bool beeper_activated;
  
  // Status
  bool signal_valid;
  bool failsafe_active;
  unsigned long last_update;
  uint8_t rssi;                   // Signal strength (0-100)
  
  // Channel health
  bool channel_health[16];        // Per-channel signal health
  uint16_t channel_timeouts[16];  // Per-channel timeout counters
  
  bool failsafe;                 // Failsafe flag used by receivers.cpp
};

// Battery Status Structure
struct BatteryStatus {
  bool connected;
  bool low_voltage_warning;
  bool critical_voltage;
  float voltage;
  float percentage;
  float current;
  float remaining_capacity_mah;
  unsigned long last_update;
};

// Calibration Data Structure
struct CalibrationData {
  // Gyroscope calibration
  bool gyro_calibrated;
  float gyro_offsets[3];
  float gyro_scale_factors[3];
  
  // Accelerometer calibration
  bool accel_calibrated;
  float accel_offsets[3];
  float accel_scale_matrix[3][3];
  
  // Magnetometer calibration
  bool mag_calibrated;
  float mag_offsets[3];
  float mag_scale_matrix[3][3];
  float mag_declination;
  
  // ESC calibration
  bool esc_calibrated;
  uint16_t esc_min_values[4];
  uint16_t esc_max_values[4];
  uint8_t esc_cal_quality;        // ESC calibration quality score
  unsigned long esc_cal_time;     // ESC calibration timestamp
  
  // System calibration status
  bool system_calibrated;
  unsigned long calibration_timestamp;
};

// Sensor Redundancy Types
enum FlightCapability {
  FLIGHT_CAPABILITY_NONE = 0,
  FLIGHT_CAPABILITY_BASIC,
  FLIGHT_CAPABILITY_ENHANCED,
  FLIGHT_CAPABILITY_FULL_GPS
};

// Flight Tuning Structure
struct FlightTuning {
  float stability_gain_multiplier;
  float aggressiveness_factor;
  float response_time;
};

// Sensor Redundancy Configuration
struct SensorRedundancyConfig {
  bool dual_imu_enabled;
  bool synthetic_sensors_enabled;
  float failover_threshold;
  unsigned long validation_timeout_ms;
  bool auto_failover_enabled;
};

struct SyntheticSensorData {
  bool synthetic_gps_valid;
  float synthetic_lat, synthetic_lon;
  float gps_confidence;
  bool synthetic_mag_valid;
  float synthetic_heading;
  float mag_confidence;
  bool synthetic_baro_valid;
  float synthetic_altitude;
  float baro_confidence;
};

// Function Prototypes
void update_sensor_fusion();
void init_pins();

// Forward declarations - implementations now in separate files
class DualIMUManager;
class DynamicFilteringSystem;
class OpticalFlowSensor;
class AdvancedFlightModes;

// Phase 2 initialization function declaration
void init_dual_imu_system();

#endif // CONFIG_H
