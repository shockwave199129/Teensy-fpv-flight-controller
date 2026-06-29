You are an experienced, pragmatic software engineering AI agent. Do not over-engineer a solution when a simple one is possible. Keep edits minimal. If you want an exception to ANY rule, you MUST stop and get permission first.

# Project Overview

**FPV Drone Flight Controller** – a dual‑IMU, sensor‑redundant flight controller for quad‑copter drones built around a **Teensy 4.1** microcontroller. The project comprises two main parts:

- **Firmware** (Arduino/C++) – implements sensor fusion, cascaded PID, dynamic filtering, safety checks, black‑box logging, and support for many ESC protocols.
- **Desktop Configurator** (Electron + React + Vite) – a cross‑platform UI for live telemetry, calibration wizards, motor configuration, PID tuning, GPS mapping, and safety dashboards.

The goal is to provide a professional‑grade, beginner‑friendly flight controller that can be safely used out‑of‑the‑box while still offering deep customisation for advanced pilots.

### Technology Choices
- **Firmware language**: C++ (Arduino framework) compiled with PlatformIO targeting Teensy 4.1.
- **Desktop language**: JavaScript/React (ES2022) running on Node 18, bundled with Vite.
- **Build tools**: PlatformIO (`platformio run`, `platformio run -t upload`) for firmware; npm scripts (`npm run dev`, `npm run build`, `npm run dist`) for the configurator.
- **Hardware**: Teensy 4.1, multiple ESC protocols, up to nine IMU types, magnetometers, barometers, GPS, optical flow, and RGB LED strip.

# Reference

## Important Code Files
- `firmware/fpv_drone_teensy/*.cpp|*.h` – core flight controller implementation (e.g., `dual_imu_manager.cpp`, `dynamic_filtering.cpp`, `blackbox_logger.cpp`).
- `firmware/fpv_drone_teensy.ino` – Arduino entry point for PlatformIO.
- `desktop_app/src/main.jsx` – Electron entry point.
- `desktop_app/src/App.jsx` – React root component.
- `desktop_app/src/pages/*.jsx` – UI pages (e.g., `CalibrationWizard.jsx`, `PIDTuning.jsx`, `GPSMap.jsx`).
- `desktop_app/package.json` – npm script definitions and dependency list.
- `platformio.ini` – PlatformIO environment configuration for the Teensy build.

## Directory Structure Tips
```
.
├── firmware/                # Arduino/PlatformIO source
│   └── fpv_drone_teensy/   # .cpp/.h files
├── desktop_app/            # Electron + React UI
│   ├── src/                # React source tree
│   ├── public/             # Static assets
│   └── package.json        # npm scripts, deps
├── platformio.ini          # Firmware build config
├── README.md               # User‑facing documentation
└── CONTRIBUTING.md         # Contribution guidelines
```

## Project Architecture
- **Firmware → Sensor Layer**: drivers for IMU, magnetometer, barometer, GPS, optical flow, battery monitor.
- **Fusion Layer**: Mahony AHRS → EKF for attitude and position.
- **Control Layer**: Cascaded PID (rate + angle) with TPA, I‑term relax, dynamic notch filtering.
- **Safety Layer**: Pre‑arm checks, crash detection, auto‑disarm, battery‑sag compensation.
- **Desktop Layer**: Serial bridge ↔ firmware, JSON config store, live telemetry charts, calibration wizards.

# Essential Commands

## Firmware
```bash
# Install PlatformIO (once)
pm install -g platformio
# Build the firmware
platformio run            # compiles to .hex
# Upload to the Teensy (USB serial mode)
platformio run -t upload
# Run unit‑tests (if any exist under test/)
platformio test
```

## Desktop Configurator
```bash
# Install Node dependencies (once)
npm install               # run inside desktop_app/
# Development server with hot‑reload + Electron
npm run dev               # starts Vite + Electron
# Build production assets only
npm run build
# Package for the current OS
npm run dist              # creates installer in desktop_app/dist/
# Build for all platforms (Linux, Windows via Wine, macOS via CI)
./build-all-platforms.sh all
```

## Formatting / Linting
```bash
# JavaScript/React code
npm run lint               # runs eslint (configured in package.json)
npm run format             # runs prettier (if defined)
# C++ firmware (clang‑format)
find firmware -name "*.cpp" -o -name "*.h" | xargs clang-format -i
```

## Cleaning
```bash
# Remove build artefacts
platformio run -t clean      # firmware
rm -rf desktop_app/dist      # desktop build output
```

# Patterns (optional)

- **Sensor Redundancy** – each sensor type has a primary and fallback; the firmware automatically switches on failure and logs the event.
- **Calibration Wizard** – multi‑step UI that writes calibration data to EEPROM; the firmware validates CRC on boot.
- **Dynamic Filtering** – FFT analysis runs each 100 ms to adapt notch filter frequencies.

# Commit and Pull Request Guidelines (mandatory)

1. **Validate locally** before committing:
   - Run `npm run lint && npm run format` for the desktop code.
   - Run `platformio run` for firmware compilation.
   - Execute any existing unit tests (`platformio test`).
2. **Commit message style** – use the conventional `type: short description` format, e.g.
   - `feat: add dual‑IMU fail‑over support`
   - `fix: correct ESC telemetry parsing bug`
3. **Pull request checklist**:
   - Title follows the commit‑message convention.
   - Description outlines *what* changed, *why*, and any *testing steps*.
   - Verify CI passes (firmware compile + desktop lint).
   - Ensure the changelog entry is added to `CHANGELOG.md` (if present).

---

*Generated by the built‑in `init` skill. Adjust any sections that need project‑specific detail or additional references.*