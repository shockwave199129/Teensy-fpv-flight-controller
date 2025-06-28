# FPV Drone CLI Guide

## Overview

The Command Line Interface (CLI) provides direct access to the flight controller firmware via serial commands. This powerful tool allows advanced users to execute commands directly, debug issues, and access features that may not have dedicated UI components yet.

## Features

### 🖥️ Terminal Interface
- **Real-time communication** with flight controller
- **Command history** with Up/Down arrow navigation
- **Tab autocomplete** for available commands
- **Syntax highlighting** for different response types
- **Auto-scrolling** output terminal

### 📋 Command Categories

#### **Basic Commands**
- `help` - Display comprehensive help with all available commands
- `status` - Show detailed system status
- `arm` - Arm motors (use with extreme caution!)
- `disarm` - Disarm motors
- `reboot` - Restart flight controller

#### **Safety & Status Commands**
- `safety check` - Comprehensive pre-flight safety validation
- `gps status` - GPS system status and satellite count
- `battery status` - Battery connection and voltage monitoring
- `calibration check` - Verify all required calibrations

#### **Sensor Redundancy Commands**
- `redundancy status` - Complete sensor health and failover status
- `sensor health` - Individual sensor health monitoring
- `synthetic data` - View synthetic sensor data and confidence levels
- `emergency mode` - Activate emergency stabilization mode
- `recovery mode` - Reset and recalibrate redundancy system

#### **Calibration Commands**

**Basic Calibrations:**
- `calibrate gyro` - Gyroscope bias calibration (keep drone still)
- `calibrate accel` - 6-position accelerometer calibration
- `calibrate mag` - Magnetometer hard/soft iron calibration
- `calibrate esc` - ESC calibration sequence

**Enhanced Calibrations:**
- `calibrate mag timer` - 90-second timed magnetometer calibration
- `calibrate accel position [position]` - Individual position calibration
  - Positions: `level`, `upside_down`, `right_side`, `left_side`, `nose_down`, `nose_up`
- `calibration wizard` - Step-by-step guided calibration
- `calibration status` - JSON formatted calibration status
- `save calibration` - Save calibrations to EEPROM
- `load calibration` - Load calibrations from EEPROM
- `reset calibration` - Clear all calibration data

#### **Enhanced ESC Configuration**

**Step-by-Step ESC Calibration:**
1. `confirm propellers removed` - **REQUIRED safety step**
2. `esc calibration start` - Begin ESC calibration sequence
3. `esc calibration high` - Set high throttle point
4. `esc calibration low` - Set low throttle point
5. `esc calibration complete` - Finish calibration

**ESC Protocol Configuration:**
- `set esc protocol [type]` - Set ESC protocol
  - Types: `PWM`, `ONESHOT125`, `ONESHOT42`, `MULTISHOT`, `DSHOT150`, `DSHOT300`, `DSHOT600`, `DSHOT1200`
- `set esc direction [1-4] [normal/reverse]` - Set motor direction
- `set esc telemetry [on/off]` - Enable/disable DShot telemetry
- `set esc bidirectional [on/off]` - Enable bidirectional DShot

**DShot Commands:**
- `dshot beep1/beep2/beep3` - Send beep commands to all ESCs
- `dshot save` - Save ESC settings
- `dshot motor [1-4] [command]` - Send command to specific motor

#### **Motor Testing & Configuration**
- `test motor [1-4] [1000-2000]` - Test individual motors (propellers OFF!)
- `check motor directions` - Validate CW/CCW configuration for X-frame
- `fix motor direction [1-4]` - Auto-fix incorrect motor direction

#### **RC Channel Configuration**

**Protocol Setup:**
- `set rc protocol [0-3]` - Set RC protocol (0=PPM, 1=iBUS, 2=SBUS, 3=ELRS)

**Channel Mapping:**
- `set channel [1-16] function [0-18] [normal/reversed]` - Map channel to function
  - Functions: 0=Throttle, 1=Roll, 2=Pitch, 3=Yaw, 4=Arm/Disarm, 5=Flight Mode, 6=RTH, etc.
- `channel_test [1-16]` - Test specific channel response
- `get_rc_data` - Get current RC channel values
- `reset_channel_mapping` - Reset to default mapping

**⚠️ GPS-Dependent Functions:**
Functions 6 (RTH), 8 (Position Hold), and 15 (GPS Rescue) require GPS fix before arming.

#### **Rate Profile Configuration**
- `set rate profile [0-2]` - Switch rate profile (0=Beginner, 1=Sport, 2=Acro)
- `set rate [parameter] [value]` - Update rate parameters
  - Parameters: `maxRollRate`, `maxPitchRate`, `maxYawRate`, `rollExpo`, `pitchExpo`, `yawExpo`

#### **Sensor Detection Commands**
- `detect_sensors` / `scan_sensors` - Full sensor detection scan
- `sensor_status` / `get_sensors` - Display sensor detection report
- `sensor_json` - Get sensor info in JSON format
- `i2c_scan` - Scan I2C bus for devices
- `detect [type]` - Detect specific sensor types
  - Types: `imu`, `mag`, `baro`, `gps`, `sonar`, `flow`, `power`

#### **LED Control**
- `led [color]` - Set LED color (red, green, blue, rainbow)
- `led pattern [0-3]` - Set LED pattern

#### **Configuration Management**
- `save_config` - Save current configuration to EEPROM
- `load_config` - Load configuration from EEPROM
- `reset pid` - Reset PID integrals

## Usage Instructions

### 🔌 Connection Requirements
1. **Connect flight controller** via USB cable
2. **Select correct COM port** in Serial Connection
3. **Verify connection** - CLI will show "Connected" status
4. **Type commands** in the terminal input field

### ⌨️ Keyboard Shortcuts
- **Enter** - Execute command
- **Up/Down Arrows** - Navigate command history
- **Tab** - Autocomplete commands
- **Ctrl+C** - (Future) Interrupt current command

### 🎯 Quick Start Workflow

#### **1. System Check**
```bash
status                    # Check overall system health
calibration check         # Verify calibration status
safety check             # Comprehensive safety validation
```

#### **2. Sensor Detection**
```bash
detect_sensors           # Scan for all connected sensors
sensor_status           # View detection results
i2c_scan               # Check I2C bus devices
```

#### **3. Calibration Sequence**
```bash
calibration check       # See what needs calibration
calibrate gyro         # Keep drone completely still
calibrate accel        # Follow 6-position procedure
calibrate mag timer    # 90-second figure-8 motion
save calibration       # Save to EEPROM
```

#### **4. ESC Configuration**
```bash
confirm propellers removed    # SAFETY FIRST!
set esc protocol DSHOT600    # Set modern protocol
set esc telemetry on         # Enable telemetry
esc calibration start       # Begin ESC calibration
```

#### **5. RC Setup**
```bash
set rc protocol 2           # Set SBUS protocol
channel_test 5              # Test individual channels
set channel 5 function 4 normal  # Map arm switch
get_rc_data                 # Verify all channels
```

## Safety Features

### 🛡️ Built-in Safety Checks
- **GPS requirement enforcement** for GPS-dependent functions
- **Battery connection validation** before allowing dangerous operations
- **Calibration verification** prevents arming uncalibrated systems
- **Propeller removal confirmation** required for motor testing
- **Automatic timeout** for unresponsive commands

### ⚠️ Safety Warnings
- **Always remove propellers** before motor testing
- **Ensure stable mounting** during calibration
- **Verify GPS fix** before enabling GPS functions
- **Monitor battery voltage** during extended testing
- **Use safety check** command before first flight

### 🆘 Emergency Commands
- `disarm` - Immediate motor shutdown
- `emergency mode` - Activate emergency stabilization
- `recovery mode` - Reset sensor redundancy system

## Advanced Features

### 📊 Real-time Monitoring
- **Live telemetry data** responses
- **Sensor health monitoring** with synthetic sensor confidence
- **Battery voltage tracking** with ESC telemetry integration
- **GPS satellite count** and fix quality

### 🔧 Development Tools
- **Command history** for repeated operations
- **Response timestamps** for debugging
- **JSON output** for programmatic access
- **Error handling** with descriptive messages

### 🎛️ Autocomplete System
The CLI includes intelligent autocomplete:
- **Tab completion** for partial commands
- **Command suggestions** when multiple matches exist
- **Category-based organization** for easy discovery
- **Context-aware suggestions** based on current system state

## Troubleshooting

### Common Issues

**No Response to Commands:**
- Check serial connection status
- Verify correct COM port selection
- Try `status` command to test communication
- Restart application if needed

**Command Not Found:**
- Type `help` to see all available commands
- Check command spelling and syntax
- Use Tab autocomplete for assistance

**Calibration Failures:**
- Ensure drone is completely stationary for gyro
- Use stable, level surface for accelerometer
- Move away from metal objects for magnetometer
- Follow exact position instructions

**ESC Issues:**
- Confirm propellers are removed
- Check ESC power and connections
- Verify ESC protocol compatibility
- Listen for ESC startup beeps

### Getting Help
- Type `help` for comprehensive command list
- Use command templates in the UI
- Check status commands for diagnostics
- Refer to user manual for detailed procedures

## Command Reference Quick Cards

### Essential Commands
```bash
help                     # Show all commands
status                   # System overview
safety check             # Pre-flight validation
calibration check        # Calibration status
```

### Calibration Essentials
```bash
calibrate gyro           # Gyroscope calibration
calibrate accel          # Accelerometer 6-position
calibrate mag timer      # 90-second magnetometer
save calibration         # Save to EEPROM
```

### ESC Quick Setup
```bash
set esc protocol DSHOT600
set esc telemetry on
confirm propellers removed
esc calibration start
```

### RC Quick Setup
```bash
set rc protocol 2
set channel 5 function 4 normal
set channel 6 function 5 normal
get_rc_data
```

---

**Remember:** The CLI provides direct access to flight controller hardware. Always prioritize safety and follow proper procedures when using these commands. 