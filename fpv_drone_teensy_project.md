# FPV Drone Project with Teensy 4.1 - Developer Documentation

## Overview

This document outlines the design and development plan for building a feature-rich, highly stable FPV (First-Person View) drone using the **Teensy 4.1** development board. The drone includes support for multiple RC receiver protocols, various sensors (IMU, barometer, magnetometer, GPS, sonar), ESCs (single and 4-in-1), USB/UART communication, LED control, and advanced flight modes. It also includes a desktop configuration app using **Electron + React.js**.

---

## Hardware Components

### Microcontroller

- **Teensy 4.1** (600 MHz ARM Cortex-M7, multiple I2C/SPI/UART ports)

### Sensors

- **IMU (MPU6050/MPU9250)** - Accelerometer + Gyroscope (I2C)
- **Magnetometer (HMC5883L)** (I2C)
- **Barometer (BMP280/BME280)** (I2C)
- **GPS (NEO-6M/UBlox)** (UART)
- **Sonar (HC-SR04 or similar)** (Digital or I2C)

### RC Receiver

- Support for:
  - **PPM** (Single digital pin input)
  - **iBUS** (UART)
  - **SBUS** (UART, inverted signal)
  - **ELRS (ExpressLRS)** (UART)

### ESCs and Motors

- **4-in-1 ESC or Individual ESCs**
- Motor control via PWM (Teensy high-resolution timers)
- ESC calibration routine

### Peripherals

- **RGB LED Strip (WS2812 or similar)** (Digital pin + FastLED library)
- **USB or UART** (for PC communication/telemetry)
- **Battery voltage/current sensor** (ADC pins)

---

## Software Architecture

### Firmware Overview

- Developed using **Arduino + Teensyduino**
- Modular architecture
  - Sensor drivers
  - Receiver parsing
  - PID control
  - ESC output
  - LED control
  - USB/UART communication
  - Flight modes

### Main Features

- Motor arming/disarming via RC switch
- ESC calibration procedure
- Orientation estimation using complementary or Kalman filter
- GPS-based return-to-home (RTH)
- Altitude and position hold
- Head and headless flight modes
- Sonar-assisted low-altitude control
- LED feedback (armed/disarmed status, GPS lock, flight mode)
- Smooth learning curve and configuration via serial/USB console

---

## Pin Mapping (Suggested)

| Function             | Teensy 4.1 Pin       |
| -------------------- | -------------------- |
| SDA (I2C0)           | 18                   |
| SCL (I2C0)           | 19                   |
| UART1 RX (GPS)       | 0                    |
| UART1 TX (GPS)       | 1                    |
| UART2 RX (SBUS/iBUS) | 7                    |
| UART2 TX             | 8                    |
| PWM Motor 1          | 2                    |
| PWM Motor 2          | 3                    |
| PWM Motor 3          | 4                    |
| PWM Motor 4          | 5                    |
| RGB LED              | 6                    |
| Sonar TRIG           | 22                   |
| Sonar ECHO           | 23                   |
| Battery Voltage      | A0                   |
| Current Sensor       | A1                   |
| USB/UART Telemetry   | Native USB / Serial3 |

---

## Flight Modes

1. **Manual (Rate Mode)** – Full manual control
2. **Stabilize** – Auto-level using IMU
3. **Altitude Hold** – Maintain current altitude (barometer + sonar)
4. **Position Hold** – Maintain lat/lon (requires GPS lock)
5. **Return to Home** – Fly back to home location (on failsafe or RC switch)
6. **Headless Mode** – Forward direction fixed regardless of yaw

---

## ESC Calibration

- On startup, detect calibration mode via RC stick position
- Output max PWM (e.g., 2000us) for 3 seconds
- Then output min PWM (e.g., 1000us) to complete
- Store calibrated values in EEPROM or config struct

---

## PID Control Loop

- Loop frequency: 500-1000 Hz (via IntervalTimer)
- Inputs: Filtered IMU + GPS + sonar + barometer
- Outputs: Motor PWM (mapped through mixer for quad-X)
- Tunable PID gains per axis (roll, pitch, yaw, alt)

---

## Sensor Fusion

- Complementary filter (initial)
- Upgrade to Kalman filter or Mahony/Madgwick (future)
- Sensor input priority:
  - IMU (fast)
  - Barometer/Sonar (slow but stable)
  - GPS (slowest, used for high-level nav)

---

## RGB LED Feedback

- Use FastLED or Adafruit\_NeoPixel
- LED states:
  - **Red** – Disarmed
  - **Green** – Armed
  - **Blue blink** – GPS lock pending
  - **Purple** – RTH active

---

## USB/UART Communication

- Teensy supports native USB
- Communicate via Serial, Serial1/2/3
- Console protocol:
  - `STATUS` → prints all sensor/RC/ESC values
  - `CALIBRATE ESC` → start ESC calibration
  - `ARM` / `DISARM`
  - `MODE` → change flight mode
  - `SET PID ROLL KP 1.2`

---

## PC Configuration App (Electron + React.js)

### Purpose

A desktop configuration utility for Windows/macOS/Linux built with **Electron + React.js** to manage all flight controller settings over USB or UART.

### Key Features

- **Serial Port Connection**

  - Detect and connect to Teensy
  - Command console with feedback

- **Receiver Settings**

  - Protocol selection: PPM, iBUS, SBUS, ELRS
  - Channel mapping and preview
  - Switch assignment wizard

- **ESC Configuration**

  - Calibration process
  - Direction reversal
  - Individual motor test sliders

- **Flight Mode Configuration**

  - Assign RC switches to flight modes
  - Visual switch position mapping

- **LED Management**

  - Set color/mode based on status
  - Live preview of LED behavior

- **Sensor Calibration**

  - Accelerometer
  - Magnetometer
  - Barometer

- **Advanced Configuration**

  - PID tuning
  - Failsafe options
  - RTH settings
  - Serial telemetry

- **Profile Save/Load**

  - JSON-based export/import of full config

### Stack

- **Frontend**: React.js + TailwindCSS or MUI
- **Backend**: Electron + Node.js + serialport
- **Communication Protocol**: JSON over serial

Example:

```json
{ "command": "SET_PROTOCOL", "value": "SBUS" }
{ "command": "GET_STATUS" }
{ "command": "CALIBRATE_ESC" }
```

### Development Tools

- `electron-builder` for cross-platform builds
- `serialport` for USB comm
- `vite-plugin-electron` or `electron-forge`
- `react-hook-form` or similar for forms

---

## Future Add-ons

- SD logging (via SD slot)
- Telemetry (e.g., via ESP8266 or HC-12)
- OSD integration (via MAX7456 or Betaflight-style chip)
- MAVLink support

---

## Developer Workflow

1. Start with testing all sensors individually
2. Implement sensor fusion and stabilize mode
3. Add receiver and ESC support
4. Build and test PID loop on bench
5. Implement failsafes and GPS-based modes
6. Tune PID values for stable flight
7. Add feedback (LED, telemetry)
8. Build the Electron+React configuration tool
9. Test full system in a safe environment

---

## Safety Considerations

- Always remove props during testing
- Use a low-voltage alarm for battery
- Implement failsafe for RC signal loss
- Always fly in open areas when testing new code

---

## Useful Tools

- Logic analyzer (to debug SBUS/iBUS)
- Oscilloscope (for PWM/ESC debug)
- FTDI adapter (for UART communication)
- Telemetry monitor (e.g., Mission Planner if MAVLink later)

---

## Licensing & Collaboration

- MIT License for open-source contribution
- Encourage modular development for easy fork/debug

---

## Contact & Contributions

- Project Maintainer: [Your Name / GitHub]
- Contributions via PR welcome

---

This document will grow with the project. Next milestone: Basic stabilization and motor test, followed by development of the PC configuration tool.

