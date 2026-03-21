# FPV Drone Project with Teensy 4.1

A complete FPV drone flight controller firmware and desktop configuration application with **dual IMU sensor fusion**, **dynamic adaptive filtering**, **flight-smoothness tuning**, **beginner-oriented safety defaults**, and **advanced flight modes**.

## Table of contents

- [Quick start — how to use](#quick-start--how-to-use-this-project)
- [Recent updates](#recent-updates-latest)
- [Project overview & key features](#-key-features-code-driven-summary)
- [Hardware & pin mapping](#hardware-requirements)
- [Safety & calibration](#-important-safety-notice-)
- [Installation (firmware & desktop app)](#installation)
- [Using the desktop configurator](#usage)
- [Flight modes](#flight-modes)
- [Serial / CLI commands](#serial-commands)
- [More CLI (safety, calibration, redundancy)](#serial-commands-safety-calibration-redundancy)
- [RC channels & rate profiles](#rc-channel-functions)
- [Safety features](#safety-features)
- [Sensor redundancy](#sensor-redundancy-system)
- [Troubleshooting & development](#troubleshooting)
- [Credits & legal](#credits)
- [Appendix: Advanced tuning](#appendix-advanced-tuning-and-implementation-reference)

---

## Quick start — how to use this project

Follow this order the first time (and after major firmware changes):

| Step | Action |
|------|--------|
| **1** | **Hardware**: Wire Teensy 4.1 per [Pin mapping](#pin-mapping). Connect ESCs (props **off**), RC RX, battery sense, optional GPS/I2C sensors. |
| **2** | **Flash firmware**: Open `firmware/fpv_drone_teensy/fpv_drone_teensy.ino` in **Arduino IDE + Teensyduino**. Board: **Teensy 4.1**, USB: **Serial**. Install libraries (**FastLED**, etc.). **Upload**. |
| **3** | **Serial check**: Open serial monitor at **115200 baud**. You should see boot messages and `help` working. |
| **4** | **Desktop app**: `cd desktop_app && npm install`. Dev: `npm run dev` (starts Vite + Electron). Production UI: `npm run build` then `npm start`. |
| **5** | **Connect app**: In the app, choose the Teensy **COM/tty** port and connect. Pages are gated until connected. |
| **6** | **Calibrate** (required to arm): Gyro → Accel → Mag → ESC (props off) → RC mapping. Use **Calibration Wizard** / **Sensor Calibration** or CLI (`calibrate gyro`, etc.). |
| **7** | **Save**: Use **Save Config to File** in the sidebar (JSON export) and/or firmware `save calibration` / EEPROM save as documented below. |
| **8** | **Tune**: **PID Tuning**, **Motor Config**, **Receiver** rate profiles. Start in **Stabilize**; use **Beginner** profile (rate profile **0**) for first flights. |
| **9** | **Pre-flight**: **Safety Status** page; throttle low; level craft; GPS modes only with fix if required. |
| **10** | **Fly**: Arm via stick sequence (see firmware `constants.h`: throttle low + yaw right, hold ~2 s). Disarm: throttle low + yaw left, hold **~0.75 s** (longer hold reduces accidental disarm). |

**Packaging the desktop app** (optional): `npm run generate-icons` (requires dev deps) then `npm run dist` or `npm run dist-linux` / `dist-win` / `dist-mac`. See `desktop_app/public/BINARY_ASSETS_TODO.md` for asset notes.

---

## Recent updates (latest)

### Firmware — smoothness, stability, beginner safety

- **PID fixes**: Per-axis D-term filtering; adaptive gains computed from **stored base gains** (no runaway drift); **throttle pass-through** on PID output; **per-mode angle limits** (`MAX_ANGLE_*` for stabilize / alt hold / pos hold).
- **Dual IMU path**: When `ENABLE_DUAL_IMU` is on, **mag / baro / GPS / sonar / battery** still update every loop; fused IMU gets **calibration** applied.
- **Filtering**: **Gyro PT1 LPF** (~200 Hz) before dynamic notches; **accel PT1 LPF** (~40 Hz) before Mahony; **Mahony Ki** enabled for gyro bias learning; **RC smoothing** uses profile `rc_smoothing_factor` as a PT1 filter.
- **Motor mix**: **Airmode-style** output shifting, **sqrt thrust linearization**, **slew rate limiting** on motor commands; **battery sag compensation** (`compensate_battery_voltage_sag`) when enabled on the motor feature struct.
- **PID extras**: **TPA** (throttle PID attenuation), **I-term relax** on fast stick moves.
- **Horizon mode**: `FLIGHT_MODE_HORIZON` — blends angle-rate setpoints from stick deflection before the rate loop (self-level near center, more rate-like at full stick).
- **Safety**: Disarm hold **~750 ms**; battery warnings aligned with **`LOW_VOLTAGE_THRESHOLD` / `CRITICAL_VOLTAGE_THRESHOLD`** in `constants.h`; **pre-arm** checks (RC valid, throttle min, level, baro/mag freshness); **crash detection** with auto-disarm; **critical battery** → **RTH** if GPS fix + min sats, else emergency descent path.
- **Defaults**: Compile-time **beginner-oriented** style in `constants.h`; **Beginner rate profile (0)** applies **`BEGINNER_THROTTLE_LIMIT`** (e.g. 70% max throttle).

### Desktop app — configurator features

- **Save / Load config JSON** from the sidebar (Electron file dialog + IPC).
- **New pages**: **Board Config**, **GPS Map** (canvas trail / home / RTH UI), **OSD Layout** editor, **Modes & AUX** (range-style mode slots), **Blackbox** viewer (Chart.js, CSV export).
- **Fixes**: Sensor Redundancy, Dual IMU, Dynamic Filtering, System Info, Motor Config wired to shared `sendCommand` / connection props where applicable.
- **Icons**: `npm run generate-icons` builds PNG/ICO (and ICNS on macOS when tools available).

---

## Project Overview

This project implements a **professional-grade flight controller** for quadcopter drones using the Teensy 4.1 microcontroller, with **cascaded PID**, **sensor fusion**, **redundancy**, and a **Betaflight/iNav-style desktop configurator** (Electron + React + Vite).

## ✨ Key Features (Code-Driven Summary)

**🚁 Firmware (Teensy 4.1):**
- **Universal ESC Protocol Support:** PWM, OneShot125/42, Multishot, DShot150/300/600/1200
- **ESC Firmware Compatibility:** BLHeli_S, BLHeli_32, BlueJay, AM32, ESCape32, Generic
- **DMA-Based Motor Control:** High-speed DShot signal generation and telemetry via DMA
- **Bidirectional DShot & Motor Direction Control:** 3D mode, direction commands, ESC beep, LED control
- **DShot Telemetry:** Real-time RPM, temperature, voltage, current monitoring
- **ESC Firmware Auto-Detection:** Detects and adapts to connected ESC firmware
- **Advanced Motor Features:** Predictive control, RPM-based dynamic filtering, health monitoring, battery compensation, individual scaling
- **Multiple RC Protocols:** PPM, iBUS, SBUS, ExpressLRS (ELRS) with ISR-based parsing for low latency
- **Enhanced Channel Mapping:** 19 functions, 3 rate profiles, expo curves, failsafe, arming logic
- **Dual IMU Sensor Fusion:** Automatic failover, cross-validation, adaptive tuning
- **Sensor Redundancy System:** Health monitoring, synthetic sensor generation (GPS, magnetometer, barometer), adaptive flight characteristics, failover, capability levels
- **Comprehensive Sensor Support:** 9 IMU types, 7 magnetometer types, 5 barometer types, GPS, sonar, optical flow
- **Optical Flow Positioning:** PMW3901, PAA5100, ADNS3080 support, dead-reckoning fallback for GPS
- **Advanced Mahony AHRS Filter:** Quaternion-based attitude estimation, adaptive fusion
- **Extended Kalman Filter (EKF):** Position/velocity fusion from IMU, GPS, optical flow
- **Cascaded PID Control:** Rate + angle loops, adaptive gain scheduling from **base gains**, TPA, I-term relax, feedforward, setpoint weighting, per-axis D-term filtering, harmonic compensation, flight phase detection
- **Dynamic Adaptive Filtering System:** FFT-based gyro analysis, gyro **PT1 LPF** before notches, auto-tuning notch filters, vibration detection, RPM feed-forward notch tuning
- **Motor output chain:** Airmode-style mix headroom, optional thrust linearization, slew limiting, battery sag compensation
- **Advanced Flight Modes:** Acro+, Sport, Cinematic, GPS Rescue, Turtle, Launch/Land Assist, RTH, Headless, Altitude/Position Hold, **Horizon** (angle/rate blend by stick)
- **Safety Functions:** Pre-arm checks (RC, throttle min, level, sensor freshness), crash detection auto-disarm, critical-battery RTH or descent, longer disarm hold, unified battery thresholds, failsafe, emergency stop, LED/audio feedback
- **Calibration System:** 6-position accelerometer, magnetometer (figure-8, timer), ESC, RC, persistent storage with CRC validation
- **Blackbox Logging:** SD card and serial logging of flight data, events, and system status
- **LED Control:** RGB LED patterns, status indication, error/warning feedback, custom patterns
- **Navigation Helpers:** Distance/bearing calculations, heading control
- **Sensor Detection & Diagnostics:** Automatic I2C scan, sensor presence reporting, diagnostics
- **Modular Firmware Architecture:** Classes for flight modes, sensor fusion, filtering, logging, safety, communication, hardware abstraction


**💻 Desktop Application (Electron + React):**
- ✅ **Cross-platform** support (Windows, macOS, Linux)
- ✅ **Real-time Serial Communication** with live telemetry and control
- ✅ **Enhanced Safety Interface**:
  - **Safety Status Dashboard**: Real-time safety monitoring and alerts
  - **Pre-Flight Safety Check**: Comprehensive validation before arming
  - **GPS Safety Monitoring**: GPS requirement enforcement and status
  - **Battery Safety Dashboard**: Dual voltage monitoring and warnings
  - **Real-time Warning System**: Active alerts and safety notifications
- ✅ **Comprehensive Configuration Interface**:
  - **Enhanced Dashboard**: Real-time flight data with safety status
  - **Calibration Wizard**: Step-by-step sensor setup with visual feedback
  - **6-Position Accelerometer Calibration**: Visual interface with progress tracking
  - **Enhanced Sensor Calibration**: Individual position calibration with 90-second magnetometer timer
  - **Enhanced Motor Configuration**: Step-by-step ESC calibration, direction validation, safety checks
  - **RC Receiver Setup**: Advanced channel mapping and rate profiles
  - **Cascaded PID Tuning**: Separate rate and angle loop tuning
  - **Flight Modes Configuration**: GPS-aware mode selection with safety blocking
  - **Dual IMU Monitoring**: Real-time dual IMU status and health
  - **Dynamic Filtering Visualization**: Spectral analysis and filter tuning
  - **Sensor Redundancy Dashboard**: Real-time redundancy monitoring and synthetic sensor display
  - **Board Configuration**: Feature toggles, mixer diagram, board orientation preview, motor assignment table
  - **GPS Map**: Live position, home, trail, distance rings, RTH controls (canvas-based)
  - **OSD Layout**: Drag-and-drop virtual OSD grid, element visibility and sizing
  - **Modes & AUX**: Betaflight-style AUX range bars and arming-prevention flags
  - **Blackbox Logs**: Log listing, Chart.js replay, CSV export
  - **Configuration Save/Load**: Export/import full **JSON** config via Electron dialog (sidebar)
  - **LED Pattern Customization**: RGB LED configuration
  - **System Information**: Comprehensive sensor detection and diagnostics
  - **Icon generation**: `npm run generate-icons` for `electron-builder` assets
- ✅ **Advanced Safety Features**:
  - **Safety Status Page**: Dedicated comprehensive safety monitoring
  - **GPS Requirement Visualization**: Real-time GPS dependency tracking
  - **Battery Safety Cards**: Multi-source voltage monitoring display
  - **Warning Management**: Real-time alert collection and dismissal
  - **Emergency Action Buttons**: Quick access to safety commands
  - **Sensor Redundancy Controls**: Emergency mode and recovery system access
  - **Configuration Save/Load**: Export and import settings

## Hardware Requirements

### Microcontroller
- **Teensy 4.1** (600 MHz ARM Cortex-M7)

### Supported Sensors

#### IMU Sensors (9 Types Supported)
- **MPU6050** - Basic 6-axis IMU, I2C address 0x68
- **MPU9250** - 9-axis IMU with built-in magnetometer, I2C address 0x68
- **ICM20948** - Advanced 9-axis IMU with DMP, I2C address 0x69
- **ICM42688P** - High-performance 6-axis IMU, I2C address 0x68
- **BMI270** - Low-power 6-axis IMU, I2C address 0x68
- **LSM6DSO32** - High-g 6-axis IMU (±32g), I2C address 0x6A/0x6B
- **BMI323** - Ultra-low power IMU, I2C address 0x68
- **ICM20602** - Racing-grade IMU, I2C address 0x68
- **LSM6DS33** - Compact 6-axis IMU, I2C address 0x6A/0x6B

#### Magnetometer Sensors (7 Types Supported)
- **HMC5883L** - Classic 3-axis magnetometer, I2C address 0x1E
- **QMC5883L** - Alternative to HMC5883L, I2C address 0x0D
- **RM3100** - Professional 3-axis magnetometer, I2C address 0x20-0x23
- **MMC5883MA** - Low-noise magnetometer, I2C address 0x30
- **IST8310** - High-precision magnetometer, I2C address 0x0C/0x0E
- **LIS3MDL** - ST 3-axis magnetometer, I2C address 0x1C/0x1E
- **AK8963** - Built into MPU9250, I2C address 0x0C

#### Barometer Sensors (5 Types Supported)
- **BMP280** - Basic pressure sensor, I2C address 0x76/0x77
- **BME280** - Pressure + humidity sensor, I2C address 0x76/0x77
- **BMP388** - High-precision pressure sensor, I2C address 0x76/0x77
- **MS5611** - Aviation-grade barometer, I2C address 0x77
- **LPS22HB** - Waterproof pressure sensor, I2C address 0x5C/0x5D

#### Other Sensors
- **GPS** - Various UART-based GPS modules (NMEA/UBX protocols)
- **HC-SR04** - Ultrasonic distance sensor (sonar)
- **PMW3901/PAA5100/ADNS3080** - Optical flow sensors for position hold
- **Voltage/Current** - Analog sensors for battery monitoring

### Hardware Connections
- **4x ESCs** or 4-in-1 ESC (supports all major protocols)
- **4x Brushless motors** with telemetry support
- **RC Receiver** (PPM/SBUS/iBUS/ELRS)
- **RGB LED Strip** (WS2812 or similar)
- **Battery voltage sensor** and current sensor

## Pin Mapping

| Function             | Teensy 4.1 Pin |
|---------------------|----------------|
| Motor 1 PWM         | 2              |
| Motor 2 PWM         | 3              |
| Motor 3 PWM         | 4              |
| Motor 4 PWM         | 5              |
| RGB LED Strip       | 6              |
| RC Receiver (UART)  | 7 (RX)         |
| GPS (UART)          | 0 (RX), 1 (TX) |
| I2C SDA             | 18             |
| I2C SCL             | 19             |
| Sonar Trigger       | 22             |
| Sonar Echo          | 23             |
| Battery Voltage     | A0             |
| Current Sensor      | A1             |

## ⚠️ IMPORTANT SAFETY NOTICE ⚠️

**This firmware includes mandatory sensor calibration requirements:**
- **The drone CANNOT be armed without proper calibration**
- **All sensors must be calibrated before first flight**
- **Calibration data is automatically validated for safety**

**Always calibrate in this order:**
1. Gyroscope (stationary calibration)
2. Accelerometer (6-position calibration) 
3. Magnetometer (figure-8 motion calibration)
4. ESC (with propellers removed)
5. RC receiver (channel mapping)

**This prevents crashes due to:**
- Incorrect sensor readings
- Uncalibrated control surfaces
- Wrong motor directions
- Invalid RC channel mapping

## Installation

### Firmware Setup

1. **Install Arduino IDE with Teensyduino**
   ```bash
   # Download and install from: https://www.pjrc.com/teensy/teensyduino.html
   ```

2. **Required Libraries**
   - Wire (built-in)
   - FastLED
   - Servo (built-in)
   - IntervalTimer (built-in)
   - EEPROM (built-in)

3. **Compile and Upload**
   ```bash
   # Open firmware/fpv_drone_teensy/fpv_drone_teensy.ino in Arduino IDE
   # Select Tools > Board > Teensy 4.1
   # Select Tools > USB Type > Serial
   # Click Upload
   ```
   - **Note:** Dynamic filtering uses **CMSIS-DSP** (`arm_math.h`). Teensyduino typically provides this; if you see missing `arm_math` errors, enable **Audio** or install **Teensyduino** full package per PJRC docs.

4. **Firmware configuration (compile-time)**  
   Edit `firmware/fpv_drone_teensy/constants.h` and `config.h` for: **frame type** (`FRAME_TYPE`), **dual IMU** (`ENABLE_DUAL_IMU`), **optical flow**, **battery cell count**, **flying style** / aggressiveness, **beginner throttle limit**, voltage scale, etc.

### Desktop Application Setup

1. **Prerequisites**
   ```bash
   # Install Node.js (v16 or later)
   # Download from: https://nodejs.org/
   ```

2. **Install Dependencies**
   ```bash
   cd desktop_app
   npm install
   ```

3. **Development Mode**
   ```bash
   # Start the development server
   npm run dev
   ```

4. **Icons (optional, before packaging)**
   ```bash
   npm run generate-icons
   ```

5. **Build for Production**
   ```bash
   npm run build          # Vite production bundle
   npm run dist           # electron-builder (current OS)
   npm run dist-all       # win + mac + linux (where supported)
   ```

## Enhanced Desktop Application Safety Features

The desktop application now includes comprehensive safety monitoring and control:

### **🛡️ Safety Status Dashboard**
- **Real-time Safety Overview**: Live display of all safety systems status
- **Pre-Flight Safety Check**: Comprehensive validation with detailed feedback
- **GPS Safety Monitoring**: Real-time GPS dependency tracking and warnings
- **Battery Safety Dashboard**: Dual voltage monitoring (sensor + ESC telemetry)
- **Active Warning System**: Real-time alerts with timestamp tracking

### **📊 Enhanced Dashboard**
- **Integrated Safety Cards**: Battery, GPS, and calibration status at a glance
- **Warning Notifications**: Real-time safety alerts and notifications
- **Emergency Actions**: Quick access to safety commands and emergency stop
- **Battery Health Monitoring**: Visual voltage indicators with status levels
- **Sidebar Safety Indicator**: Real-time armed/battery/GPS status in navigation

### **✈️ GPS-Aware Flight Modes**
- **Mode Blocking**: Automatic prevention of GPS modes without fix
- **Visual Requirements**: Clear GPS requirement indicators on mode cards
- **Real-time Validation**: Live GPS status with satellite count
- **RTH Configuration**: GPS-aware return-to-home setup with validation

### **🔋 Advanced Battery Monitoring**
- **Connection Detection**: Visual indication of battery connection status
- **Dual Voltage Sources**: Main sensor + ESC telemetry voltage display
- **Status Levels**: Color-coded GOOD/LOW/CRITICAL voltage indicators
- **Emergency Warnings**: Automatic alerts for low and critical voltage

### **⚠️ Warning Management System**
- **Real-time Collection**: Automatic capture of all warnings and errors
- **Timestamp Tracking**: Time-stamped warning history
- **Auto-dismissal**: Configurable warning timeout and cleanup
- **Visual Indicators**: Color-coded severity levels

## Usage

### Desktop app — main areas (after connect)

| Area | Purpose |
|------|---------|
| **Setup & Calibration** | Safety Status, Calibration Wizard, Sensor Calibration, **Board Config**, **GPS Map**, System Info |
| **Basic Configuration** | Dashboard, Motor Config, RC Receiver, **OSD Layout** |
| **Flight Control** | PID Tuning, Flight Modes, **Modes & AUX** |
| **Advanced Tools** | Dual IMU, Dynamic Filtering, Sensor Redundancy, **Blackbox**, CLI, Advanced Performance |
| **Sidebar** | Serial connect, **Save Config to File** / **Load Config from File** (JSON) |

Telemetry and commands use **USB serial at 115200 baud** unless you change the firmware.

### Initial Setup

1. **Flash Firmware**
   - Connect Teensy 4.1 to computer via USB
   - Upload the firmware using Arduino IDE

2. **Connect Hardware**
   - Wire all sensors according to pin mapping
   - Connect ESCs and motors
   - Connect RC receiver
   - Connect power supply

3. **Launch Configuration App**
   - Run the desktop application
   - Connect to the Teensy via USB serial port
   - **Check Safety Status** - Visit the Safety Status page for comprehensive pre-flight validation

### Using the Enhanced Safety Features

1. **Safety Status Monitoring**
   - Navigate to the **Safety Status** page (first item in Setup & Calibration)
   - Monitor real-time safety indicators: Calibration, Battery, GPS, RC Signal, Sensors
   - Review active warnings and alerts
   - Use the "Refresh Status" button for manual updates

2. **Enhanced Dashboard**
   - View integrated safety cards on the main Dashboard
   - Monitor sidebar safety indicators (Armed/Battery/GPS status)
   - Receive real-time warning notifications
   - Access quick safety action buttons

3. **GPS-Aware Flight Modes**
   - Flight modes requiring GPS show visual indicators
   - GPS-dependent modes are automatically disabled without GPS fix
   - RTH configuration validates GPS availability
   - Real-time GPS status with satellite count

4. **Battery Safety System**
   - Monitor battery connection and voltage status
   - View ESC voltage telemetry when available
   - Receive automatic low/critical voltage warnings
   - Emergency RTH triggered on critical voltage

### **⚠️ MANDATORY CALIBRATION PROCESS ⚠️**

**The flight controller will NOT ARM until all required sensors are calibrated!** This safety feature prevents crashes due to uncalibrated sensors.

#### Method 1: Desktop App Enhanced Calibration (Recommended)

1. **Connect to Flight Controller**
   - Select the correct COM port in the desktop app
   - Connect to the flight controller

2. **Launch Enhanced Calibration Interface**
   - Go to the "Sensor Calibration" tab for detailed calibration
   - Or use the "Calibration Wizard" for guided setup
   - Follow the step-by-step visual instructions

3. **Enhanced Calibration Features**:
   - **Visual 6-Position Accelerometer Calibration** with drone orientation graphics
   - **90-Second Magnetometer Timer** with progress tracking and movement guidance
   - **Automatic Sensor Detection** and verification
   - **Real-time Progress Indicators** with quality assessment

#### Method 2: Manual Calibration via Serial Commands

Connect via serial terminal (115200 baud) and use these commands:

##### **1. Gyroscope Calibration**
```bash
calibrate gyro
```
- **Requirement**: Drone must be completely stationary
- **Duration**: 10-15 seconds
- **Instructions**: Place drone on level surface, do not move during calibration
- **Success**: "Gyro calibration complete - bias saved"

##### **2. Accelerometer Calibration**
```bash
calibrate accel
```
- **Requirement**: 6-position calibration procedure
- **Duration**: 2-3 minutes
- **Instructions**:
  1. Level position (normal flight orientation)
  2. Upside down (flip 180°)
  3. Right side down (90° roll right)
  4. Left side down (90° roll left)
  5. Nose down (90° pitch forward)
  6. Nose up (90° pitch backward)
- **Important**: Hold each position steady for 10-15 seconds when prompted
- **Success**: "Accelerometer calibration complete - offsets saved"

##### **2a. Individual Position Calibration (Enhanced Feature)**
```bash
# Calibrate individual positions for visual feedback
calibrate accel position level           # Position 1
calibrate accel position upside_down     # Position 2
calibrate accel position right_side      # Position 3
calibrate accel position left_side       # Position 4
calibrate accel position nose_down       # Position 5
calibrate accel position nose_up         # Position 6
```
- **Enhanced UI Features**: 
  - Visual position indicators with detailed drone orientation graphics
  - Individual "Calibrate" buttons for each position with status tracking
  - Green checkmarks ✅ when each position is completed
  - Real-time progress tracking (X/6 positions completed)
  - Reset capability to restart calibration
  - Quality assessment for each position
- **Access**: Available in both "Calibration Wizard" and "Sensor Calibration" pages

##### **3. Magnetometer Calibration**
```bash
calibrate mag                # Standard calibration
calibrate mag timer          # Enhanced 90-second timer calibration (NEW)
```
- **Enhanced Timer Calibration Features**:
  - **90-second countdown timer** with minutes:seconds display
  - **Progressive movement instructions**: Horizontal figure-8 → Vertical figure-8 → Final rotations
  - **Real-time progress bar** with percentage completion
  - **Automatic completion detection** with quality assessment
  - **Visual guidance** in desktop application
- **Standard Calibration**:
  - **Requirement**: 3D figure-8 motion
  - **Duration**: 60-90 seconds
  - **Instructions**: Hold drone firmly and move in slow figure-8 patterns in all axes
- **Success**: "Magnetometer calibration complete - hard/soft iron corrected"

##### **4. Enhanced ESC Calibration**
```bash
calibrate esc                    # Standard ESC calibration
esc calibration start           # Enhanced step-by-step calibration (NEW)
esc calibration high            # Proceed to high throttle step
esc calibration low             # Proceed to low throttle step
esc calibration complete        # Complete calibration process
```
- **⚠️ CRITICAL SAFETY WARNINGS**: 
  - **REMOVE ALL PROPELLERS FIRST!**
  - **Mandatory safety confirmation required before any ESC operations**
  - **UI enforces safety checks - dangerous operations disabled until confirmed**
- **Enhanced Step-by-Step Process**:
  - **Safety validation**: Propeller removal confirmation required
  - **Visual progress tracking**: Step indicators (Start → High → Low → Complete)
  - **Master control slider**: Simultaneous control of all 4 motors
  - **Individual motor sliders**: Fine control for each motor
  - **Real-time throttle display**: Shows exact throttle values (1000-2000)
  - **Audio feedback integration**: ESC beep detection and confirmation
- **Motor Direction Validation**:
  - **Automatic direction checking**: Validates CW/CCW configuration for X-frame
  - **Visual indicators**: ✅ green for correct, ❌ red for incorrect directions
  - **One-click fixes**: "Fix Direction" buttons for incorrect motors
  - **Configuration diagram**: Real-time quadcopter layout with rotation indicators
- **Success**: "ESC calibration complete - all motors ready"

##### **5. RC Receiver Calibration**
```bash
# Set your RC protocol first
set rc protocol 1      # iBUS
set rc protocol 2      # SBUS
status                 # should now show RC signal valid when transmitter on
```

#### **6. Verify Calibration Status**
```bash
calibration check
```
- This command shows which calibrations are complete
- All sensors must show "✓ CALIBRATED" before arming is allowed
- **Example output**:
```
CALIBRATION_OK: All sensors calibrated - ready for flight
✓ Gyroscope: Quality 95%
✓ Accelerometer: Quality 89%
✓ Magnetometer: Quality 92%
✓ ESC: All motors calibrated
✓ RC Receiver: 6 channels mapped
```

#### **7. Save Calibration Data**
```bash
save calibration
```
- Stores all calibration data to EEPROM
- Calibration persists through power cycles
- **Always save after successful calibration!**

### Configuration Workflow (After Calibration)

1. **Verify System Ready**
   ```bash
   calibration check    # Ensure all sensors calibrated
   status              # Check overall system health
   ```

2. **Motor Configuration**
   - **Select ESC Protocol** (PWM/OneShot/DShot)
   - Test individual motors (propellers still OFF!)
   - Verify rotation directions
   - **Enable DShot features** (telemetry, bidirectional control)

3. **RC Channel Mapping**
   - Test all channel inputs
   - Configure flight mode switches
   - Set up failsafe behavior

4. **PID Tuning**
   - Start with default values
   - Tune incrementally for stable flight
   - Test in stabilize mode first

5. **Flight Modes**
   - Assign flight modes to RC switches
   - Configure RTH and failsafe options

6. **Pre-Flight Testing**
   - Bench test all functions
   - **Install propellers** (correct rotation!)
   - Perform careful test flights in open area

### **Troubleshooting Calibration Issues**

#### **Gyro Calibration Fails**
- Ensure drone is completely stationary
- Check for vibrations from fans, AC, etc.
- Move away from sources of interference

#### **Accelerometer Calibration Fails**
- Ensure each position is held steady
- Use a flat, level surface for reference
- Check that IMU is firmly mounted

#### **Magnetometer Calibration Fails**
- Move away from metal objects (>2 meters)
- Avoid power lines, electronics, cars
- Ensure smooth, continuous motion
- Try outdoor calibration if indoor fails

#### **ESC Calibration Issues**
- Verify ESC protocol matches firmware setting
- Check ESC power and signal connections
- Listen for ESC startup tones
- Some ESCs may need manual calibration mode

#### **RC Receiver Issues**
- Verify transmitter is bound to receiver
- Check receiver protocol setting
- Ensure receiver has power and signal
- Verify channel mapping matches your transmitter

### **Reset Calibration**
If you need to start over:
```bash
reset calibration    # Clears all calibration data
```
**Warning**: This will require re-calibrating ALL sensors before arming is allowed.

## Flight Modes

### Standard Flight Modes
1. **Manual (Rate Mode)** - Direct gyro rate control
2. **Stabilize** - Auto-level using IMU
3. **Altitude Hold** - Maintain current altitude (tighter max tilt than stabilize — see `MAX_ANGLE_ALTITUDE_HOLD`)
4. **Position Hold** - GPS position lock (stricter max tilt — `MAX_ANGLE_POSITION_HOLD`)
5. **Return to Home** - Automatic return to takeoff point
6. **Headless Mode** - Forward direction locked to home
7. **Horizon** - Blends **angle** and **rate** setpoints from stick deflection: near-center sticks behave like stabilize; large deflections behave more like rate mode. Select via firmware / future CLI; add to desktop **Flight Modes** when serial mode name is exposed.

### Advanced Flight Modes (Phase 2)
1. **Acro+** - Rate mode with automatic recovery at 60° tilt
2. **Sport** - High performance mode with 1.5x rate multiplier
3. **Cinematic** - Ultra-smooth operation with enhanced expo curves
4. **GPS Rescue** - Advanced return-to-home with obstacle awareness
5. **Turtle** - Upside-down recovery mode with limited throttle
6. **Launch Assist** - Automatic takeoff detection and assistance
7. **Landing Assist** - Automatic landing detection and control

## ESC Protocol Support

### Universal ESC Compatibility
The firmware supports **all major ESC protocols** and **ESC firmware types**:

#### **ESC Protocols Supported**
- **PWM** (1000-2000μs) - Standard servo protocol
- **OneShot125** (125-250μs) - 2x faster response  
- **OneShot42** (42-84μs) - 5x faster response
- **Multishot** (5-25μs) - Ultra-fast analog protocol
- **DShot150/300/600/1200** - Digital protocols with telemetry

#### **ESC Firmware Compatibility**
- ✅ **BLHeli_S** - Full support with configuration
- ✅ **BLHeli_32** - Complete integration
- ✅ **BlueJay** - Enhanced open-source firmware support
- ✅ **AM32** - Modern open-source firmware
- ✅ **ESCape32** - High-performance firmware
- ✅ **Generic** - Universal compatibility

#### **DShot Features**
- **Bidirectional DShot** with real-time RPM telemetry
- **Motor direction control** via DShot commands
- **ESC beep commands** for identification
- **LED control commands** for ESC status lights
- **Settings save/load** commands
- **3D mode support** for advanced maneuvers

### ESC Configuration Commands
```bash
# Set ESC protocol
set esc protocol DSHOT600               # Set DShot600 protocol
set esc telemetry on                    # Enable telemetry
set esc bidirectional on                # Enable bidirectional DShot

# Motor direction control
set esc direction 1 reverse             # Reverse motor 1
set esc direction 2 normal              # Normal motor 2 direction

# DShot commands
dshot beep1                             # Send beep to all motors
dshot motor 1 beep2                     # Send beep to motor 1
dshot save                              # Save ESC settings
```

## Serial Commands

The flight controller accepts commands over **USB/UART at 115200 baud** (default). The **authoritative list** is implemented in firmware (`communication.cpp` / related handlers). The desktop app sends the same strings via IPC.

- **In-app reference:** `desktop_app/public/cli-guide.md`  
- **Tip:** Type `help` on the serial console after flashing to see what your build supports.

### Basic Commands
```bash
help                          # Show all commands
status                        # Display system status
arm                          # Arm motors (careful!)
disarm                       # Disarm motors
calibrate esc               # Start ESC calibration
test motor [1-4] [pwm]      # Test individual motor
set pid [axis] [param] [val] # Tune PID parameters
set protocol [type]          # Change RC protocol
reset pid                    # Reset PID integrals
led [color/pattern]          # Control LED patterns
reboot                       # Restart controller
```

### ESC Protocol Commands
```bash
# Set ESC protocol (motor must be disarmed)
set esc protocol PWM                    # Standard PWM (1000-2000μs)
set esc protocol ONESHOT125             # OneShot125 (125-250μs)
set esc protocol ONESHOT42              # OneShot42 (42-84μs)
set esc protocol MULTISHOT              # Multishot (5-25μs)
set esc protocol DSHOT150               # DShot 150kbps
set esc protocol DSHOT300               # DShot 300kbps
set esc protocol DSHOT600               # DShot 600kbps (recommended)
set esc protocol DSHOT1200              # DShot 1200kbps

# Motor direction control
set esc direction [1-4] normal          # Set motor to normal direction
set esc direction [1-4] reverse         # Set motor to reverse direction

# DShot telemetry and features (DShot protocols only)
set esc telemetry on                    # Enable DShot telemetry
set esc telemetry off                   # Disable DShot telemetry
set esc bidirectional on                # Enable bidirectional DShot
set esc bidirectional off               # Disable bidirectional DShot

# DShot commands (DShot protocols only)
dshot beep1                             # Send beep1 to all ESCs
dshot beep2                             # Send beep2 to all ESCs
dshot beep3                             # Send beep3 to all ESCs
dshot save                              # Save ESC settings
dshot motor [1-4] beep1                 # Send beep1 to specific motor
dshot motor [1-4] beep2                 # Send beep2 to specific motor
dshot motor [1-4] led_on                # Turn on ESC LED
dshot motor [1-4] led_off               # Turn off ESC LED
```

### Phase 2 Advanced Commands
```bash
# Dual IMU Management
dual_imu status                     # Show dual IMU health status
dual_imu force_primary              # Force primary IMU usage
dual_imu force_secondary            # Force secondary IMU usage
dual_imu auto                       # Enable automatic failover

# Dynamic Filtering
filtering status                    # Show dynamic filter status
filtering auto on                   # Enable auto-tuning filters
filtering auto off                  # Disable auto-tuning filters
filtering analysis                  # Show spectral analysis

# Optical Flow
optical_flow status                 # Show optical flow data
optical_flow reset                  # Reset position tracking

# Advanced Flight Modes
advanced_mode acro+                 # Activate Acro+ mode
advanced_mode sport                 # Activate Sport mode
advanced_mode cinematic             # Activate Cinematic mode
advanced_mode gps_rescue            # Trigger GPS rescue
advanced_mode turtle                # Activate turtle mode

phase_status                        # Show all Phase 1 & 2 features
```

### Sensor Detection Commands
```bash
# Full system sensor detection
detect_sensors                      # Complete sensor scan (all types)
scan_sensors                        # Alias for detect_sensors

# Get detection results
sensor_status                       # Display sensor detection report
get_sensors                         # Alias for sensor_status
sensor_json                         # Get sensor info in JSON format

# I2C bus diagnostics
i2c_scan                           # Scan I2C bus for all devices

# Specific sensor type detection
detect imu                         # Detect IMU sensors only
detect mag                         # Detect magnetometer sensors only
detect baro                        # Detect barometer sensors only
detect gps                         # Detect GPS sensor
detect sonar                       # Detect sonar sensor
detect flow                        # Detect optical flow sensor
detect power                       # Detect voltage/current sensors
```

### Detection Example Output
```bash
> detect_sensors
Starting sensor detection scan...
[INFO] Scanning I2C bus for devices...
[INFO] Found device at address 0x68: MPU6050 IMU
[INFO] Found device at address 0x1E: HMC5883L Magnetometer  
[INFO] Found device at address 0x77: BMP280 Barometer
[INFO] GPS module detected on Serial1
[INFO] Sonar sensor responding on pins 22/23
Sensor detection complete

> sensor_status
=== Sensor Detection Report ===
IMU Sensors:
  ✓ MPU6050 - DETECTED (Primary IMU)
  ✗ MPU9250 - NOT FOUND
  ✗ ICM20948 - NOT FOUND

Magnetometer:
  ✓ HMC5883L - DETECTED
  ✗ QMC5883L - NOT FOUND

Barometer:
  ✓ BMP280 - DETECTED
  ✗ BME280 - NOT FOUND

Other Sensors:
  ✓ GPS - DETECTED (Serial1)
  ✓ Sonar - DETECTED (HC-SR04)
  ✗ Optical Flow - NOT FOUND
  ✓ Voltage Monitor - DETECTED (A0)
  ✓ Current Sensor - DETECTED (A1)

I2C Devices: 0x68, 0x1E, 0x77
===============================
```

## RC Channel Functions

The firmware supports **19 different channel functions** for flexible RC mapping:

| Function ID | Function Name        | Description                    |
|-------------|---------------------|--------------------------------|
| 0           | THROTTLE            | Main throttle control          |
| 1           | ROLL                | Roll axis control              |
| 2           | PITCH               | Pitch axis control             |
| 3           | YAW                 | Yaw axis control               |
| 4           | ARM_DISARM          | Arm/disarm switch              |
| 5           | FLIGHT_MODE         | Flight mode switch             |
| 6           | RTH                 | Return to home                 |
| 7           | ALTITUDE_HOLD       | Altitude hold mode             |
| 8           | POSITION_HOLD       | Position hold mode             |
| 9           | HEADLESS_MODE       | Headless mode                  |
| 10          | TURTLE_MODE         | Turtle mode activation         |
| 11          | BEEPER              | Beeper activation              |
| 12          | LED_CONTROL         | LED pattern control            |
| 13          | CAMERA_TILT         | Camera gimbal tilt             |
| 14          | RATE_PROFILE        | Rate profile switch            |
| 15          | GPS_RESCUE          | GPS rescue mode                |
| 16          | LAUNCH_ASSIST       | Launch assist mode             |
| 17          | BLACKBOX            | Blackbox logging               |
| 18          | NONE                | Disabled channel               |

### Rate Profiles

The system includes **3 configurable rate profiles**:

#### **Profile 0: Beginner** (default rate profile index `0`)
- Max Roll/Pitch Rate: 200°/s
- Max Yaw Rate: 180°/s
- Expo: 0.0 (linear response)
- RC Smoothing: 0.7 — applied as a **PT1 low-pass** on roll/pitch/yaw/throttle outputs
- **Throttle cap:** `BEGINNER_THROTTLE_LIMIT` in `constants.h` (e.g. **70%** of full stick range) applies only when this profile is active

#### **Profile 1: Sport**
- Max Roll/Pitch Rate: 400°/s
- Max Yaw Rate: 360°/s
- Expo: 0.3 (moderate curves)
- RC Smoothing: 0.3 (moderate smoothing)

#### **Profile 2: Acro/Race**
- Max Roll/Pitch Rate: 800°/s
- Max Yaw Rate: 720°/s
- Expo: 0.5 (aggressive curves)
- RC Smoothing: 0.1 (minimal smoothing)

## Safety Features

- ✅ **Calibration-Based Arming Prevention** - Cannot arm without proper calibration
- ✅ **RC Signal Loss Protection** - Automatic failsafe mode
- ✅ **Low Battery Warning** - Visual and audible alerts
- ✅ **Dual IMU Health Monitoring** - Automatic failover between IMUs
- ✅ **Emergency Stop** - Immediate motor shutdown capability
- ✅ **Arming/Disarming Sequences** - Prevents accidental motor start
- ✅ **Vibration Detection** - Automatic filtering adjustment
- ✅ **GPS Rescue Mode** - Advanced return-to-home with obstacle awareness
- ✅ **Motor Saturation Compensation** - Prevents control authority loss
- ✅ **Watchdog Timers** - System health monitoring
- ✅ **GPS Requirement Enforcement** - Prevents arming without GPS when GPS features are configured
- ✅ **Battery Connection Detection** - Prevents arming without battery connected
- ✅ **Enhanced Battery Monitoring** - Dual voltage monitoring via main sensor and ESC telemetry
- ✅ **Pre-Flight Safety Checks** - RC valid, throttle low, level check, sensor freshness, calibration, GPS rules
- ✅ **Crash detection** - High-g / stall heuristics with timed auto-disarm when armed
- ✅ **Disarm stick hold** - Longer hold required to reduce accidental in-flight disarm
- ✅ **Critical battery actions** - RTH when GPS adequate; otherwise pilot warning / descent path

### 🛡️ Enhanced Safety System (New Features)

#### **GPS Requirement Enforcement**
The firmware now enforces GPS requirements for GPS-dependent features:

**GPS-Dependent Functions:**
- Return to Home (RTH)
- GPS Rescue
- Position Hold

**Safety Enforcement:**
- **Configuration Warning**: When assigning GPS-dependent functions to RC channels, the system warns if GPS is not available
- **Arming Prevention**: Cannot arm if GPS-dependent functions are configured but GPS fix is not available
- **Mode Switch Blocking**: Cannot switch to GPS-dependent flight modes without GPS fix
- **Minimum Requirements**: GPS fix with 6+ satellites required

**Commands:**
```bash
gps status                  # Check GPS system status and configured GPS functions
safety check               # Comprehensive pre-flight safety validation
```

#### **Enhanced Battery Safety**
Advanced battery monitoring with multiple sources:

**Battery Detection:**
- **Connection Verification**: Automatically detects if battery is connected
- **Voltage Range Validation**: Ensures voltage readings are within expected range (1V-30V)
- **Arming Prevention**: Cannot arm without battery connected

**Dual Battery Monitoring:**
- **Main Voltage Sensor**: Traditional analog voltage divider
- **ESC Voltage Telemetry**: More accurate voltage from DShot ESCs (when available)
- **Automatic Selection**: Uses ESC telemetry when available (more accurate)

**Battery status levels** (aligned with `constants.h` for a default **3S** pack):
- **Low warning**: below **`LOW_VOLTAGE_THRESHOLD`** (default **10.5 V** pack)
- **Critical**: below **`CRITICAL_VOLTAGE_THRESHOLD`** (default **9.9 V** pack) — land immediately; firmware may trigger **RTH** if GPS is good, else emergency descent logic

> **Desktop app:** If the **Flight Modes** or dashboard still shows older thresholds (e.g. 11.5 V / 10.5 V), treat the **firmware / `constants.h`** values as source of truth until the UI is updated.

**Commands:**
```bash
battery status              # Check battery connection and voltage status
```

**Emergency Features:**
- **Critical Voltage RTH**: Automatically triggers RTH when battery critically low (if GPS available)
- **Continuous Monitoring**: Real-time battery monitoring while armed
- **Multiple Warnings**: Visual LED alerts and serial warnings

#### **Pre-Flight Safety Check System**

Comprehensive safety validation before allowing arming:

```bash
safety check                # Complete pre-flight safety validation
```

**Checks performed (firmware):**
1. **Calibration** — gyro / accel / system calibration flags
2. **Critical sensors** — IMU health, battery present, etc.
3. **RC** — `signal_valid`, throttle at or below **arm threshold**
4. **Attitude** — roll/pitch within **~10°** of level for arming
5. **Baro / mag** — if marked healthy, data must not be **stale** (timeout vs `last_update`)
6. **GPS** — if any GPS-dependent RC function is configured, fix + minimum satellites (or synthetic GPS confidence) as implemented
7. **General sensor health** — as reported by redundancy / IMU paths

**Safety Check Results:**
- **✓ SAFE TO ARM** - All checks passed, ready for flight
- **✗ NOT SAFE TO ARM** - Lists specific issues requiring attention

## Sensor Redundancy System

The firmware includes a comprehensive **sensor redundancy system** that provides automatic failover and synthetic sensor generation for enhanced flight safety.

### **🛡️ Redundancy Features**

#### **Sensor Priority Classification**
- **CRITICAL**: IMU (required for flight)
- **HIGH**: Battery, RC (required for safe operation) 
- **MEDIUM**: Barometer, Magnetometer (degraded performance without)
- **LOW**: GPS, Sonar, Optical Flow (optional features)

#### **Flight Capability Levels**
- **FLIGHT_FULL_CAPABILITY**: All sensors operational
- **FLIGHT_DEGRADED_GPS**: No GPS - using synthetic position estimation
- **FLIGHT_DEGRADED_MAG**: No magnetometer - using GPS heading or gyro integration
- **FLIGHT_DEGRADED_BARO**: No barometer - using GPS altitude or sonar
- **FLIGHT_MINIMAL**: Only IMU + RC - basic stabilization only
- **FLIGHT_EMERGENCY**: Critical sensor failure - emergency landing mode

#### **Synthetic Sensor Generation**
- **Synthetic GPS**: Dead reckoning using IMU acceleration + magnetometer heading + barometer altitude
- **Synthetic Magnetometer**: GPS course over ground when moving, or gyro integration for heading
- **Synthetic Barometer**: GPS altitude or sonar for low altitude measurements

#### **Adaptive Flight Characteristics**
- **Stability gain multiplier**: 1.0x (full) to 3.0x (emergency)
- **Aggressiveness reduction**: 1.0x (full) to 0.2x (emergency)  
- **Smoothing factor**: 1.0x (full) to 3.0x (emergency)
- **Emergency stabilization mode**: Maximum stability settings

### **🖥️ Desktop Redundancy Dashboard**

The **Sensor Redundancy** page provides real-time monitoring:

- **Flight Capability Status**: Visual display with color-coded capability levels
- **Sensor Health Grid**: Individual sensor status (HEALTHY/SYNTHETIC/DEGRADED/FAILED)
- **Synthetic Sensor Systems**: Active synthetic sensors with confidence percentages
- **Active Alerts**: Real-time sensor failure alerts with timestamps
- **Emergency Controls**: Emergency mode and recovery system access
- **Auto-refresh**: 2-second update interval for live monitoring

### **⚡ Redundancy Commands**
```bash
redundancy status              # Complete sensor redundancy safety report
redundancy status json         # JSON formatted redundancy data for desktop app
sensor health                  # Individual sensor health status  
synthetic data                 # View synthetic sensor data and confidence levels
emergency mode                 # Activate maximum stability emergency mode
recovery mode                  # Reset and recalibrate redundancy system
```

## Serial commands (safety, calibration, redundancy)

Additional CLI groups beyond the [main Serial Commands](#serial-commands) section:

### Enhanced Safety Commands
```bash
# Pre-flight safety validation
safety check                        # Comprehensive safety check before arming
gps status                          # GPS system status and function requirements
battery status                      # Battery connection and voltage monitoring

# GPS-dependent function warnings
set channel 6 function 6 normal     # RTH function - warns if GPS not available
set failsafe rth true               # RTH failsafe - warns if GPS not available
```

### Enhanced Calibration Commands
```bash
# Enhanced magnetometer calibration with timer
calibrate mag timer                  # 90-second timer calibration with progress tracking

# Enhanced ESC calibration
esc calibration start               # Start step-by-step ESC calibration
esc calibration high                # Proceed to high throttle step
esc calibration low                 # Proceed to low throttle step
esc calibration complete            # Complete calibration process

# Motor direction configuration
check motor direction               # Validate motor directions for X-configuration
fix motor direction [1-4]           # Fix incorrect motor direction
propeller safety check             # Confirm propellers removed for safety
```

### Sensor Redundancy Commands
```bash
# Sensor redundancy monitoring
redundancy status                   # Complete sensor redundancy safety report
redundancy status json              # JSON formatted redundancy data for desktop app
sensor health                       # Individual sensor health status
synthetic data                      # View synthetic sensor data and confidence levels

# Emergency and recovery controls
emergency mode                      # Activate maximum stability emergency mode
recovery mode                       # Reset and recalibrate redundancy system
```

## Development

### Adding New Features

1. **Firmware Changes**
   - Modify appropriate header files in `firmware/`
   - Update main loop if needed
   - Test thoroughly on bench

2. **Desktop App Changes**
   - Add new React components in `desktop_app/src/`
   - Update communication protocols
   - Test cross-platform compatibility

### Project Structure

```
Teensy-fpv-flight-controller/
├── firmware/fpv_drone_teensy/     # Teensy 4.1 sketch + modules
│   ├── fpv_drone_teensy.ino      # Main loop, sensors, arm/disarm, EEPROM
│   ├── config.h                  # Pins, structs, flight modes enum
│   ├── constants.h               # Thresholds, PID defaults, feature flags
│   ├── pid_controller.h          # Cascaded PID, TPA, I-relax, horizon blend
│   ├── motor_control.cpp/.h      # ESC protocols, mixing, airmode, sag comp
│   ├── sensors.cpp/.h            # IMU/mag/baro/GPS, fusion, calibration hooks
│   ├── receivers.cpp/.h          # PPM/SBUS/iBUS/ELRS, mapping, rate profiles
│   ├── dynamic_filtering.cpp/.h  # Notch + gyro LPF, FFT path
│   ├── safety_functions.cpp      # Battery, pre-arm, RC loss, RTH helpers
│   ├── communication.cpp/.h      # CLI / serial commands
│   ├── esc_dma.cpp/.h            # DShot DMA (Teensy 4.x)
│   ├── flight_modes.cpp/.h
│   ├── sensor_redundancy.cpp/.h
│   └── ...                       # dual_imu, optical_flow, blackbox, etc.
├── desktop_app/
│   ├── main.js                   # Electron main, serial, IPC, file dialogs
│   ├── preload.js                # contextBridge API
│   ├── scripts/generate-icons.js # PNG/ICO (and ICNS) from SVG
│   ├── public/                   # icon.svg, cli-guide.md, assets
│   ├── src/
│   │   ├── App.jsx               # Routes, nav, save/load config JSON
│   │   ├── components/
│   │   └── pages/                # Dashboard, Safety, Calibration, PID,
│   │                             # MotorConfig, ReceiverConfig, FlightModes,
│   │                             # ModesConfig, Blackbox, GPSMap, OSDConfig,
│   │                             # BoardConfig, SensorRedundancy, CLI, ...
│   └── package.json
├── README.md
└── LICENSE
```

## Troubleshooting

### Common Issues

1. **No Serial Connection**
   - Check USB cable and port selection
   - Verify Teensy is powered and running
   - Try different baud rates

2. **Motors Not Responding**
   - Ensure ESCs are calibrated
   - Check power connections
   - Verify PWM signal integrity

3. **Unstable Flight**
   - Check propeller balance and orientation
   - Verify motor rotation directions
   - Tune PID parameters gradually

4. **GPS Not Working**
   - Ensure clear sky view
   - Check UART connections
   - Wait for satellite acquisition

5. **Calibration Fails**
   - Follow calibration procedure exactly
   - Ensure stable environment
   - Check sensor connections

### Getting Help

- Check the troubleshooting section in the code comments
- Review the serial monitor output for error messages
- Test individual components separately
- Use the desktop app's diagnostic tools

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License. See LICENSE file for details.

## Disclaimer

**⚠️ WARNING ⚠️**

Flying drones can be dangerous. Always:
- Follow local laws and regulations
- Fly in safe, open areas
- Remove propellers during testing
- Use proper safety equipment
- Never fly over people or property
- Keep visual line of sight

The authors are not responsible for any damage or injury caused by the use of this software/hardware.

## Credits

- **Teensy Development**: PJRC (Paul Stoffregen)
- **FastLED Library**: FastLED Community
- **Electron Framework**: GitHub/Microsoft
- **React**: Meta/Facebook
- **ESC Firmware**: BLHeli team, BlueJay developers, AM32 team

## Performance overview

Expectations depend on **airframe, props, PID tune, and sensors**. This stack is designed for:

| Goal | What helps |
|------|------------|
| **Smoother flight** | Gyro LPF + dynamic notches, accel LPF into Mahony, RC PT1 smoothing, motor slew limit |
| **Stable hover** | Cascaded angle/rate PID, baro/GPS when enabled, good vibration isolation |
| **Beginners** | Default beginner-style constants, rate profile **0**, throttle cap, pre-arm checks, Horizon mode (firmware) |
| **Safety** | Calibration gating, redundancy/synthetic fallbacks, battery/RTH logic, crash heuristics |

Avoid treating unbench-tested **percentage improvement** claims as guarantees—always tune and verify on your quad.

---

## Appendix: Advanced tuning and implementation reference

Use this when you outgrow defaults and are editing **C++** (most advanced knobs are still **compile-time** unless exposed in `communication.cpp`).

### Where to look

| Topic | Primary files |
|-------|----------------|
| Rate / angle PID, TPA, I-relax, Horizon blend, adaptive gains from **base_gains** | `pid_controller.h` |
| Motor mix, airmode shift, thrust curve, slew, sag comp | `motor_control.cpp`, `motor_control.h` |
| Gyro notch + PT1 LPF | `dynamic_filtering.cpp` |
| Accel LPF, Mahony, fusion entry | `sensors.cpp` |
| RC expo, profiles, smoothing | `receivers.cpp` |
| Thresholds, defaults, feature flags | `constants.h`, `config.h` |
| CLI truth | `communication.cpp` |

### Adaptive gains (pattern)

Gains are **recomputed from stored `base_gains` each update** so adaptive scaling does not compound drift:

```cpp
float adaptive_multiplier = disturbance_factor * load_factor;
cascaded_gains.rate_roll.kp  = base_gains.rate_roll.kp  * (0.8f + 0.4f * adaptive_multiplier);
cascaded_gains.rate_pitch.kp = base_gains.rate_pitch.kp * (0.8f + 0.4f * adaptive_multiplier);
// Yaw P/I and flight-phase helpers use the same base-gain idea.
```

Flight phase still calls `set_flight_characteristics(aggressiveness, smoothness)` to bias responsiveness vs smoothness.

### Key structs (pointers only)

- **`AdvancedPIDFeatures`** in `pid_controller.h` — adaptive PID, setpoint weighting, feedforward, TPA (`tpa_breakpoint`, `tpa_rate`), I-term relax (`conditional_integration`, `iterm_relax_threshold`, `integral_decay_rate`).
- **`AdvancedMotorFeatures`** in `motor_control.h` — RPM filtering, health, predictive control, **`battery_compensation_enabled`** (sag compensation runs when true).
- **`EnhancedSensorFusion`** / EKF / redundancy — see `sensors.h`, `sensor_redundancy.cpp`, and main loop integration.

### Preset starting points (firmware tuning)

These map to **`set_flight_characteristics(aggressiveness, smoothness)`** and related defaults—not all are user-facing in the desktop app yet.

| Style | Aggressiveness | Smoothness | Notes |
|-------|----------------|------------|--------|
| **Conservative** | ~0.7 | ~1.5 | Good first hover / line-of-sight tests |
| **Sport** | ~1.5 | ~1.0 | Balanced |
| **Aggressive** | ~2.0 | ~0.8 | Snappier; verify no oscillation |

Also use **desktop PID page** and **`set pid …`** CLI where implemented.

### CLI caveat

Commands such as `pid adaptive enable`, `motor rpm_filter enable`, or `fusion enhanced enable` appear in older docs as **examples**. Your build may not implement all of them. Always run **`help`** on the serial console and read **`communication.cpp`** for what is actually wired.

---

**Happy flying** 