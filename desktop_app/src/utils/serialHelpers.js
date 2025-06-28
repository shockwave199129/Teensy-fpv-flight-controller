/**
 * Serial Communication Helper Functions
 * for FPV Drone Desktop Application
 */

// Command validation and formatting
export const validateCommand = (command) => {
  if (!command || typeof command !== 'string') {
    throw new Error('Command must be a non-empty string');
  }
  
  // Remove dangerous characters and validate
  const sanitized = command.trim().replace(/[^\w\s\-_.]/g, '');
  if (sanitized.length === 0) {
    throw new Error('Command contains no valid characters');
  }
  
  return sanitized;
};

// Parse telemetry data from serial input
export const parseTelemetryData = (rawData) => {
  try {
    // Handle JSON telemetry
    if (rawData.includes('"telemetry":')) {
      const parsed = JSON.parse(rawData);
      return parsed.telemetry;
    }
    
    // Handle key-value pair format
    const telemetry = {};
    const lines = rawData.split('\n');
    
    lines.forEach(line => {
      if (line.includes(':')) {
        const [key, value] = line.split(':').map(s => s.trim());
        
        // Convert numeric values
        if (!isNaN(value) && value !== '') {
          telemetry[key.toLowerCase().replace(/\s+/g, '_')] = parseFloat(value);
        } else {
          telemetry[key.toLowerCase().replace(/\s+/g, '_')] = value;
        }
      }
    });
    
    return telemetry;
  } catch (error) {
    console.warn('Failed to parse telemetry data:', error);
    return null;
  }
};

// Parse sensor status from serial data
export const parseSensorStatus = (data) => {
  const sensors = {
    imu: { healthy: false, quality: 0 },
    gps: { healthy: false, fix: false, satellites: 0 },
    magnetometer: { healthy: false, quality: 0 },
    barometer: { healthy: false, quality: 0 },
    battery: { connected: false, voltage: 0, status: 'unknown' }
  };
  
  // Parse IMU status
  if (data.includes('IMU: OK') || data.includes('IMU healthy')) {
    sensors.imu.healthy = true;
  }
  
  // Parse GPS status
  const gpsMatch = data.match(/GPS.*?(\d+)\s+satellites/i);
  if (gpsMatch) {
    sensors.gps.satellites = parseInt(gpsMatch[1]);
    sensors.gps.healthy = sensors.gps.satellites >= 4;
    sensors.gps.fix = sensors.gps.satellites >= 6;
  }
  
  // Parse battery voltage
  const voltageMatch = data.match(/Battery.*?(\d+\.?\d*)\s*[Vv]/);
  if (voltageMatch) {
    sensors.battery.voltage = parseFloat(voltageMatch[1]);
    sensors.battery.connected = sensors.battery.voltage > 1.0;
    
    if (sensors.battery.voltage > 11.5) sensors.battery.status = 'good';
    else if (sensors.battery.voltage > 10.5) sensors.battery.status = 'low';
    else sensors.battery.status = 'critical';
  }
  
  return sensors;
};

// Parse calibration status
export const parseCalibrationStatus = (data) => {
  const calibration = {
    system_calibrated: false,
    gyro: { calibrated: false, quality: 0 },
    accelerometer: { calibrated: false, quality: 0 },
    magnetometer: { calibrated: false, quality: 0 },
    esc: { calibrated: false, quality: 0 },
    rc: { calibrated: false, quality: 0 }
  };
  
  // Check for system ready status
  if (data.includes('CALIBRATION_OK') || data.includes('System calibrated')) {
    calibration.system_calibrated = true;
  }
  
  // Parse individual sensor calibrations
  const calibrationRegex = /(\w+).*?calibrated.*?Quality[:\s]*(\d+)%/gi;
  let match;
  
  while ((match = calibrationRegex.exec(data)) !== null) {
    const sensor = match[1].toLowerCase();
    const quality = parseInt(match[2]);
    
    if (calibration[sensor]) {
      calibration[sensor].calibrated = true;
      calibration[sensor].quality = quality;
    }
  }
  
  return calibration;
};

// Parse safety check results
export const parseSafetyCheck = (data) => {
  const safety = {
    safe_to_arm: false,
    checks: {
      calibration: false,
      battery: false,
      gps: false,
      rc_signal: false,
      sensors: false
    },
    warnings: [],
    errors: []
  };
  
  // Check for overall safety status
  if (data.includes('SAFE TO ARM') || data.includes('✓ SAFE TO ARM')) {
    safety.safe_to_arm = true;
  }
  
  // Parse individual safety checks
  if (data.includes('✓') && data.includes('Calibration')) safety.checks.calibration = true;
  if (data.includes('✓') && data.includes('Battery')) safety.checks.battery = true;
  if (data.includes('✓') && data.includes('GPS')) safety.checks.gps = true;
  if (data.includes('✓') && data.includes('RC')) safety.checks.rc_signal = true;
  if (data.includes('✓') && data.includes('Sensors')) safety.checks.sensors = true;
  
  // Parse warnings and errors
  const lines = data.split('\n');
  lines.forEach(line => {
    if (line.includes('WARNING:') || line.includes('⚠️')) {
      safety.warnings.push(line.replace(/WARNING:|⚠️/g, '').trim());
    }
    if (line.includes('ERROR:') || line.includes('❌')) {
      safety.errors.push(line.replace(/ERROR:|❌/g, '').trim());
    }
  });
  
  return safety;
};

// Format commands for sending
export const formatCommand = (command, parameters = {}) => {
  let formattedCommand = command;
  
  // Add parameters if provided
  Object.entries(parameters).forEach(([key, value]) => {
    formattedCommand += ` ${key} ${value}`;
  });
  
  return formattedCommand.trim();
};

// Command templates for common operations
export const CommandTemplates = {
  CALIBRATION: {
    GYRO: 'calibrate gyro',
    ACCELEROMETER: 'calibrate accel',
    MAGNETOMETER: 'calibrate mag',
    ESC: 'calibrate esc',
    RC: 'calibrate rc'
  },
  
  MOTOR: {
    TEST: (motor, value) => `test motor ${motor} ${value}`,
    ARM: 'arm',
    DISARM: 'disarm',
    EMERGENCY_STOP: 'emergency_stop'
  },
  
  CONFIG: {
    SET_PID: (axis, param, value) => `set pid ${axis} ${param} ${value}`,
    SET_PROTOCOL: (type, protocol) => `set ${type} protocol ${protocol}`,
    SAVE_CONFIG: 'save config'
  },
  
  STATUS: {
    SYSTEM: 'status',
    CALIBRATION: 'calibration check',
    SAFETY: 'safety check',
    SENSORS: 'sensor_status'
  }
};

// Validation helpers
export const isValidMotorNumber = (motor) => {
  return Number.isInteger(motor) && motor >= 1 && motor <= 4;
};

export const isValidPWMValue = (value) => {
  return Number.isInteger(value) && value >= 1000 && value <= 2000;
};

export const isValidPIDValue = (value) => {
  return !isNaN(value) && value >= 0 && value <= 100;
};

export default {
  validateCommand,
  parseTelemetryData,
  parseSensorStatus,
  parseCalibrationStatus,
  parseSafetyCheck,
  formatCommand,
  CommandTemplates,
  isValidMotorNumber,
  isValidPWMValue,
  isValidPIDValue
}; 