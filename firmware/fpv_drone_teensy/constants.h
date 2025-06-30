#ifndef CONSTANTS_H
#define CONSTANTS_H

// =================== SAFETY THRESHOLDS ===================

// Battery voltage thresholds (configurable)
#define LOW_VOLTAGE_THRESHOLD 10.5f          // 3S LiPo minimum voltage (V)
#define CRITICAL_VOLTAGE_THRESHOLD 9.9f      // Emergency landing required (V)
#define BATTERY_CELL_COUNT 3                 // Default 3S battery
#define BATTERY_VOLTAGE_PER_CELL_MIN 3.3f    // Minimum per cell voltage
#define BATTERY_VOLTAGE_PER_CELL_CRITICAL 3.0f  // Critical per cell voltage

// RC signal timeouts (configurable)
#define RC_TIMEOUT_MS 1000                   // RC signal loss timeout (ms)
#define RC_FAILSAFE_THROTTLE 900             // Failsafe throttle value
#define RC_SIGNAL_LOST_THRESHOLD 100        // Consecutive missed packets for signal loss

// Arming/Disarming thresholds (configurable)
#define ARM_THROTTLE_THRESHOLD 1100          // Max throttle for arming
#define ARM_YAW_THRESHOLD 1700               // Min yaw for arming sequence
#define DISARM_YAW_THRESHOLD 1300            // Max yaw for disarming sequence
#define ARM_SEQUENCE_TIME_MS 2000            // Time to hold sticks for arm/disarm

// Sensor health timeouts (configurable)
#define IMU_TIMEOUT_MS 100                   // IMU data timeout
#define GPS_TIMEOUT_MS 5000                  // GPS data timeout
#define MAG_TIMEOUT_MS 1000                  // Magnetometer timeout
#define BARO_TIMEOUT_MS 1000                 // Barometer timeout

// GPS requirements (configurable)
#define GPS_MIN_SATELLITES 6                 // Minimum satellites for GPS functions
#define GPS_MAX_HDOP 2.0f                   // Maximum HDOP for good GPS
#define GPS_MIN_SPEED_FOR_HEADING 2.0f      // Min speed to trust GPS heading (m/s)

// Synthetic sensor confidence thresholds (configurable)
#define SYNTHETIC_GPS_MIN_CONFIDENCE 0.3f    // 30% minimum for GPS functions
#define SYNTHETIC_MAG_MIN_CONFIDENCE 0.4f    // 40% minimum for mag functions
#define SYNTHETIC_BARO_MIN_CONFIDENCE 0.5f   // 50% minimum for baro functions

// =================== CONTROL LOOP TIMING ===================

// Main loop timing (configurable)
#define MAIN_LOOP_FREQUENCY_HZ 2000          // 2kHz main loop
#define MAIN_LOOP_TIME_US (1000000 / MAIN_LOOP_FREQUENCY_HZ)
#define MAX_LOOP_TIME_US 10000               // Maximum acceptable loop time

// Sensor update rates (configurable)
#define IMU_UPDATE_FREQUENCY_HZ 2000         // 2kHz IMU
#define MAG_UPDATE_FREQUENCY_HZ 100          // 100Hz Magnetometer
#define BARO_UPDATE_FREQUENCY_HZ 50          // 50Hz Barometer
#define GPS_UPDATE_FREQUENCY_HZ 10           // 10Hz GPS
#define SONAR_UPDATE_FREQUENCY_HZ 20         // 20Hz Sonar

// Telemetry and logging rates (configurable)
#define TELEMETRY_RATE_HZ 10                 // 10Hz telemetry output
#define HEALTH_CHECK_RATE_HZ 1               // 1Hz health monitoring
#define LED_UPDATE_RATE_HZ 50                // 50Hz LED updates

// =================== PID CONTROLLER DEFAULTS ===================

// Rate loop PID gains (tunable)
#define DEFAULT_RATE_ROLL_KP 0.8f
#define DEFAULT_RATE_ROLL_KI 0.1f
#define DEFAULT_RATE_ROLL_KD 0.05f

#define DEFAULT_RATE_PITCH_KP 0.8f
#define DEFAULT_RATE_PITCH_KI 0.1f
#define DEFAULT_RATE_PITCH_KD 0.05f

#define DEFAULT_RATE_YAW_KP 1.2f
#define DEFAULT_RATE_YAW_KI 0.15f
#define DEFAULT_RATE_YAW_KD 0.01f

// Angle loop PID gains (tunable)
#define DEFAULT_ANGLE_ROLL_KP 4.5f
#define DEFAULT_ANGLE_ROLL_KI 0.02f
#define DEFAULT_ANGLE_ROLL_KD 0.2f

#define DEFAULT_ANGLE_PITCH_KP 4.5f
#define DEFAULT_ANGLE_PITCH_KI 0.02f
#define DEFAULT_ANGLE_PITCH_KD 0.2f

// Altitude PID gains (tunable)
#define DEFAULT_ALT_RATE_KP 2.0f
#define DEFAULT_ALT_RATE_KI 0.3f
#define DEFAULT_ALT_RATE_KD 0.1f

#define DEFAULT_ALT_POS_KP 1.5f
#define DEFAULT_ALT_POS_KI 0.1f
#define DEFAULT_ALT_POS_KD 0.05f

// PID limits (configurable)
#define PID_RATE_OUTPUT_LIMIT 500            // Rate loop output limit
#define PID_ANGLE_OUTPUT_LIMIT 200           // Angle loop output limit (deg/s)
#define PID_INTEGRAL_LIMIT_RATE 100          // Rate loop integral limit
#define PID_INTEGRAL_LIMIT_ANGLE 50          // Angle loop integral limit

// =================== MOTOR CONTROL LIMITS ===================

// Motor output limits (safety critical)
#define MOTOR_MIN_THROTTLE 1000              // Minimum motor output
#define MOTOR_MAX_THROTTLE 2000              // Maximum motor output
#define MOTOR_IDLE_THROTTLE 1100             // Idle throttle when armed
#define MOTOR_ARM_THROTTLE 1000              // Throttle for arming sequence

// ESC protocol timing (configurable)
#define DSHOT_TELEMETRY_TIMEOUT_MS 50        // DShot telemetry timeout
#define ESC_CALIBRATION_HIGH_THROTTLE 1800   // ESC calibration high point
#define ESC_CALIBRATION_LOW_THROTTLE 1000    // ESC calibration low point

// Motor protection (configurable)
#define MOTOR_TEMPERATURE_WARNING 80         // Motor temperature warning (°C)
#define MOTOR_TEMPERATURE_CRITICAL 100      // Motor temperature critical (°C)
#define MOTOR_RPM_MAX 30000                  // Maximum expected motor RPM

// =================== SENSOR FUSION PARAMETERS ===================

// Complementary filter parameters (tunable)
#define COMPLEMENTARY_FILTER_ALPHA 0.98f     // Gyro weight
#define COMPLEMENTARY_FILTER_BETA 0.02f      // Accel weight

// Mahony filter parameters (tunable)
#define MAHONY_KP 2.0f                       // Proportional gain
#define MAHONY_KI 0.1f                       // Integral gain

// Kalman filter parameters (tunable)
#define KALMAN_PROCESS_NOISE 0.01f           // Process noise
#define KALMAN_MEASUREMENT_NOISE_ACCEL 0.1f  // Accelerometer noise
#define KALMAN_MEASUREMENT_NOISE_GYRO 0.05f  // Gyroscope noise
#define KALMAN_MEASUREMENT_NOISE_MAG 0.2f    // Magnetometer noise
#define KALMAN_MEASUREMENT_NOISE_GPS 1.0f    // GPS noise

// Motion detection thresholds (configurable)
#define MOTION_DETECTION_ACCEL_THRESHOLD 2.0f   // Acceleration threshold for motion
#define MOTION_DETECTION_GYRO_THRESHOLD 10.0f   // Angular velocity threshold (deg/s)
#define STATIC_PERIOD_MIN_MS 1000               // Minimum static period for calibration

// =================== VIBRATION FILTERING ===================

// Vibration filter parameters (tunable)
#define VIBRATION_FILTER_CUTOFF_HZ 30.0f     // Low-pass filter cutoff
#define NOTCH_FILTER_Q_FACTOR 15.0f          // Notch filter Q factor
#define DYNAMIC_NOTCH_MIN_HZ 80.0f           // Minimum dynamic notch frequency
#define DYNAMIC_NOTCH_MAX_HZ 400.0f          // Maximum dynamic notch frequency

// Vibration detection (configurable)
#define VIBRATION_WARNING_THRESHOLD 5.0f     // Vibration warning level (g)
#define VIBRATION_CRITICAL_THRESHOLD 10.0f   // Vibration critical level (g)

// =================== FLIGHT MODE PARAMETERS ===================

// Flight mode limits (configurable)
#define MAX_ANGLE_STABILIZE 45.0f            // Max angle in stabilize mode (degrees)
#define MAX_ANGLE_ALTITUDE_HOLD 30.0f        // Max angle in altitude hold (degrees)
#define MAX_ANGLE_POSITION_HOLD 20.0f        // Max angle in position hold (degrees)

// Rate mode limits (configurable)
#define MAX_RATE_ROLL_PITCH 720.0f           // Max roll/pitch rate (deg/s)
#define MAX_RATE_YAW 360.0f                  // Max yaw rate (deg/s)

// Position hold parameters (configurable)
#define POSITION_HOLD_MAX_ERROR 10.0f        // Max position error before correction (m)
#define POSITION_HOLD_DEADBAND 2.0f          // Position hold deadband (m)

// Return to Home parameters (configurable)
#define RTH_CLIMB_HEIGHT 50.0f               // RTH climb height (m)
#define RTH_DESCENT_RATE 2.0f                // RTH descent rate (m/s)
#define RTH_LAND_SPEED 0.5f                  // Landing speed (m/s)
#define RTH_HOME_RADIUS 2.0f                 // Home position radius (m)

// =================== SENSOR CALIBRATION LIMITS ===================

// Calibration quality thresholds (configurable)
#define GYRO_CALIBRATION_SAMPLES 1000        // Samples for gyro calibration
#define GYRO_CALIBRATION_MAX_VARIANCE 0.1f   // Max variance for good gyro cal
#define ACCEL_CALIBRATION_MIN_POSITIONS 6    // Required accelerometer positions
#define ACCEL_CALIBRATION_POSITION_TIMEOUT 30000  // Position hold timeout (ms)
#define MAG_CALIBRATION_TIME_MS 60000        // Magnetometer calibration time
#define MAG_CALIBRATION_MIN_VARIANCE 10.0f   // Min variance for good mag cal

// Sensor health validation (configurable)
#define IMU_HEALTH_ACCEL_MIN 5.0f            // Min accelerometer magnitude (m/s²)
#define IMU_HEALTH_ACCEL_MAX 15.0f           // Max accelerometer magnitude (m/s²)
#define IMU_HEALTH_GYRO_MAX_DRIFT 1.0f       // Max gyro drift (deg/s)
#define MAG_HEALTH_MIN_STRENGTH 20.0f        // Min magnetic field strength (µT)
#define MAG_HEALTH_MAX_STRENGTH 80.0f        // Max magnetic field strength (µT)

// =================== COMMUNICATION TIMEOUTS ===================

// Serial communication (configurable)
#define SERIAL_BAUD_RATE 115200              // Serial baud rate
#define SERIAL_TIMEOUT_MS 1000               // Serial command timeout
#define COMMAND_BUFFER_SIZE 128              // Serial command buffer size

// Desktop app communication (configurable)
#define DESKTOP_APP_TIMEOUT_MS 5000          // Desktop app connection timeout
#define TELEMETRY_PACKET_SIZE 256            // Telemetry packet size
#define STATUS_UPDATE_INTERVAL_MS 1000       // Status update interval

// =================== EMERGENCY PROCEDURES ===================

// Emergency landing parameters (safety critical)
#define EMERGENCY_DESCENT_RATE 3.0f          // Emergency descent rate (m/s)
#define EMERGENCY_THROTTLE_CUT_ALTITUDE 2.0f // Altitude to cut throttle (m)
#define EMERGENCY_RESPONSE_TIME_MS 500       // Max emergency response time

// Sensor failure responses (configurable)
#define SENSOR_FAILURE_MAX_COUNT 5           // Max failures before emergency mode
#define SENSOR_RECOVERY_TIME_MS 10000        // Time to verify sensor recovery
#define REDUNDANCY_SWITCH_DELAY_MS 1000      // Delay before switching to backup

// =================== LED AND AUDIO FEEDBACK ===================

// LED patterns (configurable)
#define LED_BRIGHTNESS_NORMAL 128            // Normal LED brightness (0-255)
#define LED_BRIGHTNESS_LOW_BATTERY 255       // Full brightness for warnings
#define LED_FLASH_RATE_NORMAL 2              // Normal flash rate (Hz)
#define LED_FLASH_RATE_WARNING 5             // Warning flash rate (Hz)
#define LED_FLASH_RATE_CRITICAL 10           // Critical flash rate (Hz)

// Audio feedback (configurable)
#define BEEPER_FREQUENCY_NORMAL 1000         // Normal beep frequency (Hz)
#define BEEPER_FREQUENCY_WARNING 1500        // Warning beep frequency (Hz)
#define BEEPER_FREQUENCY_CRITICAL 2000       // Critical beep frequency (Hz)
#define BEEPER_DURATION_SHORT 100            // Short beep duration (ms)
#define BEEPER_DURATION_LONG 500             // Long beep duration (ms)

// =================== PHASE 2 ADVANCED FEATURES ===================

// Dual IMU parameters (configurable)
#define DUAL_IMU_DIVERGENCE_THRESHOLD 5.0f   // Max attitude divergence (degrees)
#define DUAL_IMU_GYRO_DIVERGENCE_THRESHOLD 10.0f  // Max gyro divergence (deg/s)
#define DUAL_IMU_VALIDATION_TIMEOUT_MS 5000  // Validation timeout

// Dynamic filtering (configurable)
#define DYNAMIC_FILTER_FFT_SIZE 256          // FFT buffer size
#define DYNAMIC_FILTER_UPDATE_RATE_HZ 10     // Filter update rate
#define DYNAMIC_FILTER_MIN_SNR 6.0f          // Minimum signal-to-noise ratio

// Optical flow parameters (configurable)
#define OPTICAL_FLOW_MAX_VELOCITY 5.0f       // Max measurable velocity (m/s)
#define OPTICAL_FLOW_MIN_SURFACE_QUALITY 30  // Min surface quality (0-255)
#define OPTICAL_FLOW_HEIGHT_LIMIT 10.0f      // Max height for optical flow (m)

// =================== HARDWARE-SPECIFIC CONSTANTS ===================

// Pin assignments (hardware specific)
#define MOTOR_PIN_1 2
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 4
#define MOTOR_PIN_4 5
#define LED_PIN 6
#define RC_RECEIVER_PIN 7
#define BEEPER_PIN 8
#define BATTERY_VOLTAGE_PIN A0
#define CURRENT_SENSOR_PIN A1
#define SONAR_TRIGGER_PIN 22
#define SONAR_ECHO_PIN 23
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

// Hardware limits (platform specific)
#define PWM_FREQUENCY 400                    // PWM frequency (Hz)
#define ANALOG_RESOLUTION 12                 // ADC resolution (bits)
#define CPU_FREQUENCY_MHZ 600               // Teensy 4.1 CPU frequency

// Voltage scaling (hardware calibration)
#define VOLTAGE_SCALE 0.01464f               // Voltage divider scaling
#define CURRENT_SCALE 0.01f                  // Current sensor scaling
#define VOLTAGE_REFERENCE 3.3f               // ADC reference voltage

// =================== USER CUSTOMIZABLE SETTINGS ===================

// These can be modified by users for their specific setup

// Aircraft configuration
#define AIRCRAFT_TYPE_QUADCOPTER_X           // Aircraft type
#define PROPELLER_COUNT 4                    // Number of propellers
#define MOTOR_POLE_COUNT 14                  // Motor pole count (for RPM calculation)

// Battery configuration (user specific)
#define BATTERY_CAPACITY_MAH 2200            // Battery capacity
#define BATTERY_TYPE_LIPO_3S                 // Battery type marker

// Weight and performance (user specific)
#define AIRCRAFT_WEIGHT_GRAMS 500            // Aircraft weight
#define MAX_THRUST_TO_WEIGHT_RATIO 3.0f      // Maximum thrust-to-weight ratio

// Flying style preferences (user tunable)
#define FLYING_STYLE_SPORT                   // SPORT, CINEMATIC, RACE, BEGINNER
#define AGGRESSIVENESS_LEVEL 1.2f            // 0.1-2.0 (conservative to aggressive)
#define SMOOTHNESS_LEVEL 1.0f                // 0.1-2.0 (twitchy to smooth)

// Environmental settings (location specific)
#define MAGNETIC_DECLINATION_DEGREES 0.0f    // Local magnetic declination
#define ALTITUDE_MSL_METERS 0                // Mean sea level altitude
#define TEMPERATURE_TYPICAL_CELSIUS 25       // Typical operating temperature

// =================== COMPILE-TIME FEATURE FLAGS ===================

// Enable/disable features at compile time
#define ENABLE_DUAL_IMU                      // Enable dual IMU support
#define ENABLE_DYNAMIC_FILTERING             // Enable dynamic notch filtering
#define ENABLE_OPTICAL_FLOW                  // Enable optical flow positioning
#define ENABLE_SENSOR_REDUNDANCY             // Enable sensor redundancy system
#define ENABLE_ADVANCED_TELEMETRY            // Enable detailed telemetry
#define ENABLE_BLACKBOX_LOGGING              // Enable flight data logging
#define ENABLE_GPS_RESCUE                    // Enable GPS rescue mode
#define ENABLE_TURTLE_MODE                   // Enable turtle mode recovery

// Debug and development features
//#define DEBUG_PID_TUNING                   // Enable PID debug output
//#define DEBUG_SENSOR_FUSION                // Enable sensor fusion debug
//#define DEBUG_MOTOR_CONTROL                // Enable motor control debug
//#define SIMULATE_SENSOR_FAILURES           // Enable sensor failure simulation

// =================== OPTICAL FLOW SENSOR TYPES ===================

// Optical flow sensor type definitions
#define OPTICAL_FLOW_TYPE_PMW3901   0
#define OPTICAL_FLOW_TYPE_PAA5100   1
#define OPTICAL_FLOW_TYPE_ADNS3080  2

// =================== ADVANCED FLIGHT MODE CONSTANTS ===================

// Advanced flight mode parameters
#define ACRO_PLUS_RECOVERY_ANGLE     45.0f
#define SPORT_MODE_RATE_MULTIPLIER   1.5f
#define CINEMATIC_EXPO_FACTOR        0.7f
#define GPS_RESCUE_CLIMB_RATE        3.0f
#define TURTLE_MODE_THROTTLE_LIMIT   0.8f

#endif // CONSTANTS_H 