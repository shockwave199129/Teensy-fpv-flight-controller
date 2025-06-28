/**
 * Flight Mathematics and Conversion Utilities
 * for FPV Drone Desktop Application
 */

// Unit conversions
export const convertUnits = {
  // Temperature conversions
  celsiusToFahrenheit: (celsius) => (celsius * 9/5) + 32,
  fahrenheitToCelsius: (fahrenheit) => (fahrenheit - 32) * 5/9,
  
  // Distance conversions
  metersToFeet: (meters) => meters * 3.28084,
  feetToMeters: (feet) => feet / 3.28084,
  kilometersToMiles: (km) => km * 0.621371,
  milesToKilometers: (miles) => miles * 1.60934,
  
  // Speed conversions
  mpsToKmh: (mps) => mps * 3.6,
  kmhToMps: (kmh) => kmh / 3.6,
  mpsToMph: (mps) => mps * 2.23694,
  mphToMps: (mph) => mph / 2.23694,
  
  // Angle conversions
  degreesToRadians: (degrees) => degrees * (Math.PI / 180),
  radiansToDegrees: (radians) => radians * (180 / Math.PI),
  
  // Pressure conversions
  pascalToHpa: (pascal) => pascal / 100,
  hpaToPascal: (hpa) => hpa * 100,
  hpaToInHg: (hpa) => hpa * 0.029530,
  inHgToHpa: (inHg) => inHg / 0.029530
};

// Battery calculations
export const batteryCalculations = {
  // Calculate battery percentage based on voltage (for LiPo batteries)
  voltageToPercentage: (voltage, cellCount = 3) => {
    const voltagePerCell = voltage / cellCount;
    const minVoltage = 3.0;  // Empty cell voltage
    const maxVoltage = 4.2;  // Full cell voltage
    
    if (voltagePerCell <= minVoltage) return 0;
    if (voltagePerCell >= maxVoltage) return 100;
    
    // Non-linear LiPo discharge curve approximation
    const normalizedVoltage = (voltagePerCell - minVoltage) / (maxVoltage - minVoltage);
    return Math.round(Math.pow(normalizedVoltage, 0.7) * 100);
  },
  
  // Estimate flight time remaining
  estimateFlightTime: (currentVoltage, capacity, currentDraw, cellCount = 3) => {
    const minVoltage = 3.0 * cellCount;  // Minimum safe voltage
    const remainingCapacity = batteryCalculations.voltageToPercentage(currentVoltage, cellCount) / 100 * capacity;
    
    if (currentDraw <= 0) return 0;
    
    const timeRemaining = (remainingCapacity / currentDraw) * 60; // minutes
    return Math.max(0, timeRemaining);
  },
  
  // Calculate power consumption
  calculatePower: (voltage, current) => voltage * current,
  
  // Calculate C-rating usage
  calculateCRating: (current, capacity) => current / capacity
};

// Flight calculations
export const flightCalculations = {
  // Calculate distance between two GPS coordinates
  calculateDistance: (lat1, lon1, lat2, lon2) => {
    const R = 6371000; // Earth's radius in meters
    const φ1 = convertUnits.degreesToRadians(lat1);
    const φ2 = convertUnits.degreesToRadians(lat2);
    const Δφ = convertUnits.degreesToRadians(lat2 - lat1);
    const Δλ = convertUnits.degreesToRadians(lon2 - lon1);
    
    const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ/2) * Math.sin(Δλ/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    
    return R * c; // Distance in meters
  },
  
  // Calculate bearing between two GPS coordinates
  calculateBearing: (lat1, lon1, lat2, lon2) => {
    const φ1 = convertUnits.degreesToRadians(lat1);
    const φ2 = convertUnits.degreesToRadians(lat2);
    const Δλ = convertUnits.degreesToRadians(lon2 - lon1);
    
    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    
    let bearing = convertUnits.radiansToDegrees(Math.atan2(y, x));
    return (bearing + 360) % 360; // Normalize to 0-360 degrees
  },
  
  // Calculate altitude from barometric pressure
  calculateAltitudeFromPressure: (pressure, seaLevelPressure = 1013.25) => {
    return 44330 * (1 - Math.pow(pressure / seaLevelPressure, 1/5.255));
  },
  
  // Calculate true airspeed from indicated airspeed and altitude
  calculateTrueAirspeed: (indicatedAirspeed, altitude) => {
    const densityRatio = Math.pow(1 - (altitude / 44330), 4.255);
    return indicatedAirspeed / Math.sqrt(densityRatio);
  },
  
  // Calculate wind speed and direction from GPS track and heading
  calculateWind: (groundSpeed, airSpeed, trackAngle, headingAngle) => {
    const trackRad = convertUnits.degreesToRadians(trackAngle);
    const headingRad = convertUnits.degreesToRadians(headingAngle);
    
    const windEast = groundSpeed * Math.sin(trackRad) - airSpeed * Math.sin(headingRad);
    const windNorth = groundSpeed * Math.cos(trackRad) - airSpeed * Math.cos(headingRad);
    
    const windSpeed = Math.sqrt(windEast * windEast + windNorth * windNorth);
    const windDirection = (convertUnits.radiansToDegrees(Math.atan2(windEast, windNorth)) + 360) % 360;
    
    return { speed: windSpeed, direction: windDirection };
  }
};

// PID calculations and analysis
export const pidCalculations = {
  // Calculate PID output
  calculatePID: (error, lastError, integral, kp, ki, kd, dt) => {
    const proportional = kp * error;
    const integralTerm = ki * integral * dt;
    const derivative = kd * (error - lastError) / dt;
    
    return proportional + integralTerm + derivative;
  },
  
  // Estimate optimal PID gains based on system response
  estimatePIDGains: (overshoot, settlingTime, riseTime) => {
    // Ziegler-Nichols-inspired estimation
    const kp = 1.2 / (overshoot * riseTime);
    const ki = kp / (2 * settlingTime);
    const kd = kp * settlingTime / 8;
    
    return { kp, ki, kd };
  },
  
  // Analyze PID performance metrics
  analyzePIDPerformance: (setpoint, response, timeArray) => {
    const steadyStateValue = response[response.length - 1];
    const steadyStateError = Math.abs(setpoint - steadyStateValue);
    
    // Find overshoot
    const maxValue = Math.max(...response);
    const overshoot = ((maxValue - setpoint) / setpoint) * 100;
    
    // Find settling time (within 2% of setpoint)
    const tolerance = 0.02 * setpoint;
    let settlingTime = 0;
    for (let i = response.length - 1; i >= 0; i--) {
      if (Math.abs(response[i] - setpoint) > tolerance) {
        settlingTime = timeArray[i];
        break;
      }
    }
    
    return {
      overshoot: Math.max(0, overshoot),
      settlingTime,
      steadyStateError
    };
  }
};

// Signal processing utilities
export const signalProcessing = {
  // Apply low-pass filter
  lowPassFilter: (input, lastOutput, alpha) => {
    return alpha * input + (1 - alpha) * lastOutput;
  },
  
  // Apply exponential moving average
  exponentialMovingAverage: (newValue, oldAverage, alpha) => {
    return alpha * newValue + (1 - alpha) * oldAverage;
  },
  
  // Calculate moving average
  movingAverage: (values, windowSize) => {
    if (values.length < windowSize) return values.reduce((a, b) => a + b, 0) / values.length;
    
    const window = values.slice(-windowSize);
    return window.reduce((a, b) => a + b, 0) / windowSize;
  },
  
  // Detect oscillations in data
  detectOscillations: (values, threshold = 0.1) => {
    if (values.length < 10) return { oscillating: false, frequency: 0 };
    
    let crossings = 0;
    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    
    for (let i = 1; i < values.length; i++) {
      if ((values[i-1] > mean) !== (values[i] > mean)) {
        crossings++;
      }
    }
    
    const frequency = crossings / (2 * values.length);
    const oscillating = frequency > threshold;
    
    return { oscillating, frequency };
  },
  
  // Calculate RMS (Root Mean Square)
  calculateRMS: (values) => {
    const squaredSum = values.reduce((sum, value) => sum + value * value, 0);
    return Math.sqrt(squaredSum / values.length);
  }
};

// Safety calculations
export const safetyCalculations = {
  // Calculate minimum safe battery voltage for RTH
  calculateRTHVoltage: (distance, averageSpeed, averageCurrent, batteryCapacity) => {
    const timeToHome = distance / averageSpeed; // hours
    const energyNeeded = averageCurrent * timeToHome; // Ah
    const safetyMargin = 0.2; // 20% safety margin
    
    const minEnergyPercent = (energyNeeded * (1 + safetyMargin)) / batteryCapacity;
    
    // Convert to voltage (rough approximation for 3S LiPo)
    const minVoltage = 9.0 + (1 - minEnergyPercent) * 3.6; // 9V = empty, 12.6V = full
    
    return Math.max(10.5, minVoltage); // Never below 10.5V for 3S
  },
  
  // Calculate safe operating envelope
  calculateSafeEnvelope: (windSpeed, batteryLevel, gpsAccuracy, altitude) => {
    const maxWindSpeed = 10; // m/s
    const minBatteryLevel = 30; // percent
    const maxGpsError = 5; // meters
    const maxAltitude = 400; // feet
    
    const windSafety = windSpeed <= maxWindSpeed;
    const batterySafety = batteryLevel >= minBatteryLevel;
    const gpsSafety = gpsAccuracy <= maxGpsError;
    const altitudeSafety = altitude <= maxAltitude;
    
    const overallSafety = windSafety && batterySafety && gpsSafety && altitudeSafety;
    
    return {
      safe: overallSafety,
      factors: {
        wind: windSafety,
        battery: batterySafety,
        gps: gpsSafety,
        altitude: altitudeSafety
      },
      recommendations: {
        wind: windSpeed > maxWindSpeed ? `Reduce wind exposure (${windSpeed.toFixed(1)} > ${maxWindSpeed} m/s)` : null,
        battery: batteryLevel < minBatteryLevel ? `Land soon (${batteryLevel}% < ${minBatteryLevel}%)` : null,
        gps: gpsAccuracy > maxGpsError ? `Wait for better GPS (${gpsAccuracy.toFixed(1)} > ${maxGpsError}m error)` : null,
        altitude: altitude > maxAltitude ? `Descend (${altitude.toFixed(0)} > ${maxAltitude} ft)` : null
      }
    };
  }
};

// Formatting utilities
export const formatters = {
  // Format coordinates
  formatCoordinate: (coord, type = 'lat') => {
    const abs = Math.abs(coord);
    const degrees = Math.floor(abs);
    const minutes = (abs - degrees) * 60;
    const hemisphere = type === 'lat' ? (coord >= 0 ? 'N' : 'S') : (coord >= 0 ? 'E' : 'W');
    
    return `${degrees}°${minutes.toFixed(4)}'${hemisphere}`;
  },
  
  // Format time duration
  formatDuration: (seconds) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    
    if (hours > 0) {
      return `${hours}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
    }
    return `${minutes}:${secs.toString().padStart(2, '0')}`;
  },
  
  // Format distance with appropriate units
  formatDistance: (meters) => {
    if (meters < 1000) {
      return `${meters.toFixed(1)}m`;
    } else if (meters < 10000) {
      return `${(meters / 1000).toFixed(2)}km`;
    } else {
      return `${(meters / 1000).toFixed(1)}km`;
    }
  },
  
  // Format speed with appropriate units
  formatSpeed: (mps) => {
    const kmh = convertUnits.mpsToKmh(mps);
    return `${kmh.toFixed(1)} km/h`;
  },
  
  // Format angles
  formatAngle: (degrees) => {
    return `${degrees.toFixed(1)}°`;
  },
  
  // Format voltage with color coding
  formatVoltage: (voltage, cellCount = 3) => {
    const voltagePerCell = voltage / cellCount;
    let status = 'good';
    
    if (voltagePerCell < 3.3) status = 'critical';
    else if (voltagePerCell < 3.6) status = 'low';
    else if (voltagePerCell < 3.8) status = 'medium';
    
    return {
      text: `${voltage.toFixed(2)}V`,
      status,
      percentage: batteryCalculations.voltageToPercentage(voltage, cellCount)
    };
  }
};

export default {
  convertUnits,
  batteryCalculations,
  flightCalculations,
  pidCalculations,
  signalProcessing,
  safetyCalculations,
  formatters
}; 