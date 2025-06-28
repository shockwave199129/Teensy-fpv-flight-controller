# FPV Drone Desktop Application - User Manual

## Overview
This application provides comprehensive configuration and monitoring for FPV drone flight controllers using Teensy 4.1 hardware.

## Key Features
- Real-time telemetry monitoring
- Step-by-step calibration wizards
- Enhanced safety systems
- Motor and ESC configuration
- Flight mode management
- Sensor redundancy monitoring

## Getting Started

### 1. Connection
1. Connect your Teensy 4.1 flight controller via USB
2. Launch the application
3. Select the correct COM port
4. Click "Connect"

### 2. Initial Setup
1. Visit the Safety Status page first
2. Complete all calibrations using the Calibration Wizard
3. Configure motors and ESCs
4. Set up RC receiver channels
5. Configure flight modes

## Safety Requirements

### Mandatory Calibrations
The flight controller will NOT arm without:
- Gyroscope calibration
- Accelerometer calibration (6 positions)
- Magnetometer calibration
- ESC calibration
- RC receiver setup

### Pre-Flight Checks
Always verify:
- All sensors calibrated
- Battery connected and voltage adequate
- GPS fix (if GPS functions used)
- RC signal valid
- No active warnings

## Calibration Procedures

### Accelerometer 6-Position Calibration
1. Position 1: Level (normal flight orientation)
2. Position 2: Upside down (180° flip)
3. Position 3: Right side down (90° roll right)
4. Position 4: Left side down (90° roll left)
5. Position 5: Nose down (90° pitch forward)
6. Position 6: Nose up (90° pitch backward)

Hold each position steady for 10-15 seconds when prompted.

### Motor Configuration
**⚠️ REMOVE PROPELLERS FIRST!**

1. Select ESC protocol (PWM, OneShot, DShot)
2. Follow step-by-step ESC calibration
3. Verify motor directions using the diagram
4. Test individual motors at low throttle

## Troubleshooting

### Connection Issues
- Check USB cable and drivers
- Verify correct COM port
- Restart application if needed

### Calibration Problems
- Ensure stable, vibration-free environment
- Keep away from metal objects for magnetometer
- Use flat surface for accelerometer positions

### Motor Issues
- Verify ESC power and connections
- Check motor direction configuration
- Ensure proper propeller installation

## Support
For technical support and documentation, visit the project repository or contact support.

---
© 2024 FPV Drone Controller Project 