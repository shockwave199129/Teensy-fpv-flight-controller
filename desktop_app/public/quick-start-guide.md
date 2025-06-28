# FPV Drone Controller - Quick Start Guide

## ⚡ Get Flying in 6 Steps

### Step 1: Connect Hardware
- Connect Teensy 4.1 flight controller via USB
- Ensure firmware is installed and running
- Launch the FPV Drone Desktop Application

### Step 2: Establish Connection
- Select your flight controller's COM port
- Click "Connect" button
- Wait for successful connection (green status)

### Step 3: Safety Check
- Navigate to **Safety Status** page
- Review all safety indicators
- Address any red warnings before proceeding

### Step 4: Complete Calibrations
- Open **Calibration Wizard**
- Follow all calibration steps in order:
  1. **Gyroscope** - Keep drone stationary (15 seconds)
  2. **Accelerometer** - 6 positions (2-3 minutes)
  3. **Magnetometer** - Figure-8 movements (90 seconds)
  4. **ESC** - ⚠️ Remove propellers first!
  5. **RC Receiver** - Channel mapping

### Step 5: Configure Motors
**⚠️ SAFETY: Propellers must be removed!**
- Go to **Motor Config** page
- Select ESC protocol (DShot600 recommended)
- Complete ESC calibration wizard
- Verify motor directions using the diagram
- Test each motor individually at low throttle

### Step 6: Set Up RC and Flight Modes
- Navigate to **RC Receiver** page
- Set protocol (SBUS, PPM, iBUS, ELRS)
- Map channels to functions
- Go to **Flight Modes** page
- Assign modes to RC switches
- Configure failsafe settings

## ✅ Pre-Flight Checklist

Before every flight, verify:
- [ ] All calibrations complete (green checkmarks)
- [ ] Battery voltage > 11.5V
- [ ] RC signal strong and responsive
- [ ] GPS lock if using GPS modes (6+ satellites)
- [ ] Propellers installed correctly (CW/CCW)
- [ ] Flight area is safe and legal

## 🚨 Emergency Procedures

### If Something Goes Wrong:
1. **Emergency Stop** - Use red button in Dashboard (motors stop immediately)
2. **Low Battery** - Land immediately when warning appears
3. **GPS Loss** - Switch to manual mode, fly back manually
4. **Connection Lost** - Flight controller continues with last commands

## 📞 Need Help?

### Common Issues:
- **Can't connect?** Check USB cable and COM port
- **Calibration fails?** Ensure stable environment, no vibrations
- **Motors wrong direction?** Use "Fix Direction" buttons
- **Won't arm?** Check Safety Status page for specific issues

### Resources:
- Full User Manual (user-manual.pdf)
- Video tutorials online
- Community support forum
- Technical documentation

---

**⚠️ REMEMBER: Safety first! Never skip calibrations or ignore warnings.**

*Quick Start Guide v1.0 - © 2024 FPV Drone Controller Project* 