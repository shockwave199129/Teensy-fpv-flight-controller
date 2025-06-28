# Utilities

This folder contains utility modules for the FPV Drone Desktop Application.

## Modules

### 📡 `serialHelpers.js`
Functions for handling serial communication with the flight controller.

**Key Functions:**
- `validateCommand(command)` - Validate and sanitize commands
- `parseTelemetryData(rawData)` - Parse incoming telemetry data
- `parseSensorStatus(data)` - Extract sensor status information
- `parseCalibrationStatus(data)` - Parse calibration progress
- `parseSafetyCheck(data)` - Parse safety check results
- `CommandTemplates` - Pre-defined command templates

**Usage:**
```javascript
import { validateCommand, parseTelemetryData, CommandTemplates } from '../utils/serialHelpers';

const command = validateCommand('calibrate gyro');
const telemetry = parseTelemetryData(serialData);
const motorTestCmd = CommandTemplates.MOTOR.TEST(1, 1100);
```

### 🧮 `flightMath.js`
Mathematical functions and calculations for flight-related operations.

**Key Modules:**
- `convertUnits` - Unit conversions (temperature, distance, speed, angles)
- `batteryCalculations` - Battery percentage, flight time estimation
- `flightCalculations` - GPS distance/bearing, altitude calculations
- `pidCalculations` - PID control calculations and analysis
- `signalProcessing` - Filtering, averaging, oscillation detection
- `safetyCalculations` - Safety envelope and RTH calculations
- `formatters` - Display formatting for various values

**Usage:**
```javascript
import { batteryCalculations, flightCalculations, formatters } from '../utils/flightMath';

const batteryPercent = batteryCalculations.voltageToPercentage(11.4, 3);
const distance = flightCalculations.calculateDistance(lat1, lon1, lat2, lon2);
const formattedVoltage = formatters.formatVoltage(11.4);
```

### ⚙️ `constants.js`
Application constants, configuration values, and enums.

**Key Constants:**
- `APP_INFO` - Application metadata
- `SERIAL_CONFIG` - Serial communication settings
- `REFRESH_INTERVALS` - UI update intervals
- `FLIGHT_LIMITS` - Motor, PID, RC, battery limits
- `ESC_PROTOCOLS` - ESC protocol definitions
- `RC_PROTOCOLS` - RC protocol information
- `FLIGHT_MODES` - Flight mode requirements
- `SENSOR_TYPES` - Supported sensor types and addresses
- `STATUS_COLORS` - Color schemes for status indicators
- `MESSAGES` - Warning, error, and success messages
- `DEFAULT_CONFIG` - Default configuration values

**Usage:**
```javascript
import { FLIGHT_LIMITS, STATUS_COLORS, ESC_PROTOCOLS } from '../utils/constants';

const maxMotorValue = FLIGHT_LIMITS.motor.max;
const goodColor = STATUS_COLORS.good;
const dshotInfo = ESC_PROTOCOLS.DSHOT600;
```

### 📦 `index.js`
Central export point for all utilities - use this for clean imports.

**Usage:**
```javascript
// Import everything
import * as Utils from '../utils';

// Import specific modules
import { SerialHelpers, FlightMath, Constants } from '../utils';

// Import specific functions
import { 
  validateCommand, 
  parseTelemetryData,
  batteryCalculations,
  STATUS_COLORS 
} from '../utils';
```

## Integration Examples

### Parsing Telemetry Data
```javascript
import { parseTelemetryData, formatters } from '../utils';

const handleSerialData = (rawData) => {
  const telemetry = parseTelemetryData(rawData);
  if (telemetry) {
    const batteryInfo = formatters.formatVoltage(telemetry.battery_voltage);
    setBatteryStatus(batteryInfo);
  }
};
```

### Sending Commands
```javascript
import { validateCommand, CommandTemplates } from '../utils';

const sendMotorTest = async (motor, value) => {
  try {
    const command = CommandTemplates.MOTOR.TEST(motor, value);
    const validated = validateCommand(command);
    await sendCommand(validated);
  } catch (error) {
    console.error('Invalid command:', error);
  }
};
```

### Calculating Flight Data
```javascript
import { flightCalculations, formatters } from '../utils';

const updateGpsInfo = (homePos, currentPos) => {
  const distance = flightCalculations.calculateDistance(
    homePos.lat, homePos.lon,
    currentPos.lat, currentPos.lon
  );
  const bearing = flightCalculations.calculateBearing(
    homePos.lat, homePos.lon,
    currentPos.lat, currentPos.lon
  );
  
  setDistanceToHome(formatters.formatDistance(distance));
  setBearingToHome(formatters.formatAngle(bearing));
};
```

### Using Constants
```javascript
import { FLIGHT_LIMITS, STATUS_COLORS } from '../utils';

const getBatteryColor = (voltage) => {
  if (voltage > FLIGHT_LIMITS.battery.warnVoltage) {
    return STATUS_COLORS.battery.good;
  } else if (voltage > FLIGHT_LIMITS.battery.criticalVoltage) {
    return STATUS_COLORS.battery.medium;
  } else {
    return STATUS_COLORS.battery.critical;
  }
};
```

## Benefits

✅ **Centralized Logic** - All calculations and parsing in one place  
✅ **Reusability** - Functions can be used across multiple components  
✅ **Consistency** - Standardized formatting and validation  
✅ **Maintainability** - Easy to update and test utility functions  
✅ **Type Safety** - Clear function signatures and return types  
✅ **Performance** - Optimized mathematical calculations  

## Testing

Each utility module should have corresponding unit tests to ensure reliability and accuracy of calculations. 