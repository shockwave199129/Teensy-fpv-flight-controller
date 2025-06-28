/**
 * Application Constants
 * for FPV Drone Desktop Application
 */

// Application metadata
export const APP_INFO = {
  name: 'FPV Drone Controller',
  subtitle: 'Enhanced Safety Edition',
  version: '2.0.0',
  author: 'Flight Controller Project',
  description: 'Professional flight controller configuration and monitoring'
};

// Serial communication settings
export const SERIAL_CONFIG = {
  baudRate: 115200,
  dataBits: 8,
  parity: 'none',
  stopBits: 1,
  timeout: 5000,
  reconnectDelay: 2000,
  maxReconnectAttempts: 3
};

// UI refresh intervals (milliseconds)
export const REFRESH_INTERVALS = {
  telemetry: 100,        // 10Hz - Real-time flight data
  dashboard: 500,        // 2Hz - Dashboard updates
  safety: 1000,          // 1Hz - Safety checks
  sensors: 2000,         // 0.5Hz - Sensor status
  system: 5000,          // 0.2Hz - System information
  calibration: 250       // 4Hz - Calibration progress
};

// Flight controller limits and constraints
export const FLIGHT_LIMITS = {
  // Motor values
  motor: {
    min: 1000,
    max: 2000,
    idle: 1100,
    testMin: 1000,
    testMax: 1200
  },
  
  // PID values
  pid: {
    min: 0,
    max: 100,
    defaultStep: 0.1,
    precisionDecimals: 3
  },
  
  // RC channel values
  rc: {
    min: 1000,
    max: 2000,
    center: 1500,
    deadband: 10,
    switchLow: 1300,
    switchHigh: 1700
  },
  
  // Battery thresholds (3S LiPo)
  battery: {
    cellMin: 3.0,
    cellMax: 4.2,
    cellNominal: 3.7,
    warnVoltage: 11.1,    // 3.7V per cell
    criticalVoltage: 10.5, // 3.5V per cell
    landVoltage: 9.9      // 3.3V per cell
  },
  
  // GPS requirements
  gps: {
    minSatellites: 6,
    maxAccuracy: 5.0,     // meters
    fixTimeout: 60000     // 1 minute
  },
  
  // Safety margins
  safety: {
    armingTimeout: 30000,  // 30 seconds
    warningTimeout: 10000, // 10 seconds
    emergencyTimeout: 3000 // 3 seconds
  }
};

// ESC protocols and their configurations
export const ESC_PROTOCOLS = {
  PWM: {
    name: 'PWM',
    description: 'Standard servo protocol (1000-2000μs)',
    frequency: '50Hz',
    latency: 'High',
    telemetry: false,
    bidirectional: false
  },
  ONESHOT125: {
    name: 'OneShot125',
    description: 'Fast analog protocol (125-250μs)',
    frequency: '2kHz',
    latency: 'Medium',
    telemetry: false,
    bidirectional: false
  },
  ONESHOT42: {
    name: 'OneShot42',
    description: 'Faster analog protocol (42-84μs)',
    frequency: '8kHz',
    latency: 'Low',
    telemetry: false,
    bidirectional: false
  },
  MULTISHOT: {
    name: 'Multishot',
    description: 'Ultra-fast analog protocol (5-25μs)',
    frequency: '32kHz',
    latency: 'Very Low',
    telemetry: false,
    bidirectional: false
  },
  DSHOT150: {
    name: 'DShot150',
    description: 'Digital protocol at 150kbps',
    frequency: '150kbps',
    latency: 'Low',
    telemetry: true,
    bidirectional: true
  },
  DSHOT300: {
    name: 'DShot300',
    description: 'Digital protocol at 300kbps',
    frequency: '300kbps',
    latency: 'Low',
    telemetry: true,
    bidirectional: true
  },
  DSHOT600: {
    name: 'DShot600',
    description: 'Digital protocol at 600kbps (recommended)',
    frequency: '600kbps',
    latency: 'Very Low',
    telemetry: true,
    bidirectional: true
  },
  DSHOT1200: {
    name: 'DShot1200',
    description: 'Digital protocol at 1200kbps',
    frequency: '1200kbps',
    latency: 'Ultra Low',
    telemetry: true,
    bidirectional: true
  }
};

// RC protocols and their characteristics
export const RC_PROTOCOLS = {
  PPM: {
    name: 'PPM',
    description: 'Pulse Position Modulation',
    channels: 8,
    latency: 'High (22ms)',
    reliability: 'Good'
  },
  IBUS: {
    name: 'iBUS',
    description: 'FlySky iBUS protocol',
    channels: 14,
    latency: 'Medium (7ms)',
    reliability: 'Excellent'
  },
  SBUS: {
    name: 'SBUS',
    description: 'FrSky SBUS protocol',
    channels: 16,
    latency: 'Medium (9ms)',
    reliability: 'Excellent'
  },
  ELRS: {
    name: 'ELRS',
    description: 'ExpressLRS protocol',
    channels: 16,
    latency: 'Ultra Low (2ms)',
    reliability: 'Outstanding'
  }
};

// Flight modes and their requirements
export const FLIGHT_MODES = {
  MANUAL: {
    name: 'Manual (Rate)',
    description: 'Direct gyro rate control',
    requirements: ['IMU'],
    gpsRequired: false,
    difficulty: 'Expert'
  },
  STABILIZE: {
    name: 'Stabilize',
    description: 'Auto-level using IMU',
    requirements: ['IMU'],
    gpsRequired: false,
    difficulty: 'Intermediate'
  },
  ALTITUDE_HOLD: {
    name: 'Altitude Hold',
    description: 'Maintain current altitude',
    requirements: ['IMU', 'Barometer'],
    gpsRequired: false,
    difficulty: 'Beginner'
  },
  POSITION_HOLD: {
    name: 'Position Hold',
    description: 'GPS position lock',
    requirements: ['IMU', 'GPS', 'Barometer'],
    gpsRequired: true,
    difficulty: 'Beginner'
  },
  RETURN_TO_HOME: {
    name: 'Return to Home',
    description: 'Automatic return to takeoff point',
    requirements: ['IMU', 'GPS', 'Barometer'],
    gpsRequired: true,
    difficulty: 'Automatic'
  },
  HEADLESS: {
    name: 'Headless Mode',
    description: 'Forward direction locked to home',
    requirements: ['IMU', 'GPS'],
    gpsRequired: true,
    difficulty: 'Beginner'
  }
};

// Channel functions mapping
export const CHANNEL_FUNCTIONS = {
  THROTTLE: { name: 'Throttle', type: 'analog', required: true },
  ROLL: { name: 'Roll', type: 'analog', required: true },
  PITCH: { name: 'Pitch', type: 'analog', required: true },
  YAW: { name: 'Yaw', type: 'analog', required: true },
  ARM_DISARM: { name: 'Arm/Disarm', type: 'switch', required: true },
  FLIGHT_MODE: { name: 'Flight Mode', type: 'switch', required: false },
  RTH: { name: 'Return to Home', type: 'momentary', required: false },
  ALTITUDE_HOLD: { name: 'Altitude Hold', type: 'switch', required: false },
  POSITION_HOLD: { name: 'Position Hold', type: 'switch', required: false },
  HEADLESS_MODE: { name: 'Headless Mode', type: 'switch', required: false },
  TURTLE_MODE: { name: 'Turtle Mode', type: 'momentary', required: false },
  BEEPER: { name: 'Beeper', type: 'momentary', required: false },
  LED_CONTROL: { name: 'LED Control', type: 'switch', required: false },
  CAMERA_TILT: { name: 'Camera Tilt', type: 'analog', required: false },
  RATE_PROFILE: { name: 'Rate Profile', type: 'switch', required: false },
  GPS_RESCUE: { name: 'GPS Rescue', type: 'momentary', required: false },
  LAUNCH_ASSIST: { name: 'Launch Assist', type: 'switch', required: false },
  BLACKBOX: { name: 'Blackbox', type: 'switch', required: false },
  NONE: { name: 'Disabled', type: 'none', required: false }
};

// Sensor types and their I2C addresses
export const SENSOR_TYPES = {
  IMU: {
    MPU6050: { address: 0x68, name: 'MPU6050', description: 'Basic 6-axis IMU' },
    MPU9250: { address: 0x68, name: 'MPU9250', description: '9-axis IMU with magnetometer' },
    ICM20948: { address: 0x69, name: 'ICM20948', description: 'Advanced 9-axis IMU with DMP' },
    ICM42688P: { address: 0x68, name: 'ICM42688P', description: 'High-performance 6-axis IMU' },
    BMI270: { address: 0x68, name: 'BMI270', description: 'Low-power 6-axis IMU' },
    LSM6DSO32: { address: 0x6A, name: 'LSM6DSO32', description: 'High-g 6-axis IMU (±32g)' },
    BMI323: { address: 0x68, name: 'BMI323', description: 'Ultra-low power IMU' },
    ICM20602: { address: 0x68, name: 'ICM20602', description: 'Racing-grade IMU' },
    LSM6DS33: { address: 0x6A, name: 'LSM6DS33', description: 'Compact 6-axis IMU' }
  },
  MAGNETOMETER: {
    HMC5883L: { address: 0x1E, name: 'HMC5883L', description: 'Classic 3-axis magnetometer' },
    QMC5883L: { address: 0x0D, name: 'QMC5883L', description: 'Alternative to HMC5883L' },
    RM3100: { address: 0x20, name: 'RM3100', description: 'Professional 3-axis magnetometer' },
    MMC5883MA: { address: 0x30, name: 'MMC5883MA', description: 'Low-noise magnetometer' },
    IST8310: { address: 0x0C, name: 'IST8310', description: 'High-precision magnetometer' },
    LIS3MDL: { address: 0x1C, name: 'LIS3MDL', description: 'ST 3-axis magnetometer' },
    AK8963: { address: 0x0C, name: 'AK8963', description: 'Built into MPU9250' }
  },
  BAROMETER: {
    BMP280: { address: 0x76, name: 'BMP280', description: 'Basic pressure sensor' },
    BME280: { address: 0x76, name: 'BME280', description: 'Pressure + humidity sensor' },
    BMP388: { address: 0x76, name: 'BMP388', description: 'High-precision pressure sensor' },
    MS5611: { address: 0x77, name: 'MS5611', description: 'Aviation-grade barometer' },
    LPS22HB: { address: 0x5C, name: 'LPS22HB', description: 'Waterproof pressure sensor' }
  }
};

// Color schemes for status indicators
export const STATUS_COLORS = {
  good: '#10B981',      // green-500
  warning: '#F59E0B',   // amber-500
  error: '#EF4444',     // red-500
  info: '#3B82F6',      // blue-500
  neutral: '#6B7280',   // gray-500
  
  // Battery status colors
  battery: {
    good: '#10B981',    // > 11.5V
    medium: '#F59E0B',  // 10.5-11.5V
    low: '#F97316',     // 9.9-10.5V
    critical: '#EF4444' // < 9.9V
  },
  
  // Signal strength colors
  signal: {
    excellent: '#10B981', // > 80%
    good: '#84CC16',      // 60-80%
    fair: '#F59E0B',      // 40-60%
    poor: '#F97316',      // 20-40%
    none: '#EF4444'       // < 20%
  }
};

// Warning and error messages
export const MESSAGES = {
  warnings: {
    NO_GPS: 'GPS fix required for this function',
    LOW_BATTERY: 'Battery voltage is low - consider landing',
    CRITICAL_BATTERY: 'Critical battery voltage - land immediately',
    NO_RC_SIGNAL: 'RC signal lost - check transmitter',
    SENSOR_FAILURE: 'Sensor failure detected - check connections',
    CALIBRATION_REQUIRED: 'Sensor calibration required before arming',
    HIGH_VIBRATION: 'High vibration detected - check propeller balance'
  },
  
  errors: {
    CONNECTION_FAILED: 'Failed to connect to flight controller',
    COMMAND_FAILED: 'Command execution failed',
    CALIBRATION_FAILED: 'Calibration failed - try again',
    ARM_BLOCKED: 'Arming blocked - check safety status',
    EMERGENCY_STOP: 'Emergency stop activated'
  },
  
  success: {
    CONNECTED: 'Successfully connected to flight controller',
    CALIBRATION_COMPLETE: 'Calibration completed successfully',
    SETTINGS_SAVED: 'Settings saved to flight controller',
    ARM_SUCCESS: 'Aircraft armed successfully',
    DISARM_SUCCESS: 'Aircraft disarmed successfully'
  }
};

// Default configuration values
export const DEFAULT_CONFIG = {
  pid: {
    rate: {
      roll: { kp: 0.5, ki: 0.0, kd: 0.01 },
      pitch: { kp: 0.5, ki: 0.0, kd: 0.01 },
      yaw: { kp: 0.8, ki: 0.0, kd: 0.0 }
    },
    angle: {
      roll: { kp: 2.0, ki: 0.0, kd: 0.0 },
      pitch: { kp: 2.0, ki: 0.0, kd: 0.0 }
    },
    altitude: {
      rate: { kp: 2.0, ki: 0.5, kd: 0.1 },
      position: { kp: 1.0, ki: 0.0, kd: 0.0 }
    }
  },
  
  rates: [
    { name: 'Beginner', maxRollRate: 200, maxPitchRate: 200, maxYawRate: 180, expo: 0.0 },
    { name: 'Sport', maxRollRate: 400, maxPitchRate: 400, maxYawRate: 360, expo: 0.3 },
    { name: 'Acro', maxRollRate: 800, maxPitchRate: 800, maxYawRate: 720, expo: 0.5 }
  ],
  
  safety: {
    armingTimeout: 30,
    batteryWarning: 11.1,
    batteryCritical: 10.5,
    gpsMinSatellites: 6,
    rcFailsafeTimeout: 1000
  }
};

// UI theme configuration
export const THEME = {
  fonts: {
    primary: 'Inter, system-ui, sans-serif',
    mono: 'ui-monospace, "Cascadia Code", Consolas, monospace'
  },
  
  sizes: {
    sidebar: '320px',
    header: '64px',
    card: {
      padding: '24px',
      radius: '8px'
    }
  },
  
  animations: {
    fast: '150ms',
    medium: '300ms',
    slow: '500ms'
  }
};

export default {
  APP_INFO,
  SERIAL_CONFIG,
  REFRESH_INTERVALS,
  FLIGHT_LIMITS,
  ESC_PROTOCOLS,
  RC_PROTOCOLS,
  FLIGHT_MODES,
  CHANNEL_FUNCTIONS,
  SENSOR_TYPES,
  STATUS_COLORS,
  MESSAGES,
  DEFAULT_CONFIG,
  THEME
}; 