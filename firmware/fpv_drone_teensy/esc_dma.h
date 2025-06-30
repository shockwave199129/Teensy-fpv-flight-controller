#pragma once
// DMA-driven DShot implementation for Teensy 4.x (FlexPWM + DMA)
// Supports DShot150/300/600/1200 and bidirectional telemetry.
// Only minimal register programming is included; fine-tune timing as needed.

#include <Arduino.h>
#include "config.h"

namespace ESC_DMA {

// Initialise FlexPWM + DMA for the selected DShot protocol.
void init(EscProtocol protocol, const uint8_t* motorPins, uint8_t motorCount = 4);

// Queue throttle values (0-2047) for all motors; call at control-loop rate.
void sendThrottle(const uint16_t* values, bool requestTelemetry = false);

// Send a single DShot command (e.g. beep) to one motor.
void sendCommand(uint8_t motorIndex, uint16_t command);

// Reads latest telemetry packet. Voltage in mV, current in mA (if available).
bool getTelemetry(uint8_t motorIndex, uint16_t& eRPM, uint16_t& voltage, uint8_t& temperature, uint16_t &current);

// Enable or disable telemetry bit in next frame sequence (call from main loop)
void requestTelemetry(bool enable = true);

// Read raw 12-bit payload from the last telemetry packet. Useful for ESC_INFO and other
// non-standard telemetry frames. Returns true if a fresh, CRC-checked packet was received.
bool getRawPacket(uint8_t motorIndex, uint16_t &payload12);

} // namespace ESC_DMA 