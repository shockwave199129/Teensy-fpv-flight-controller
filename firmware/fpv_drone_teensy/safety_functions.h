#ifndef SAFETY_FUNCTIONS_H
#define SAFETY_FUNCTIONS_H

#include <Arduino.h>

// Function declarations for safety functions
/**
 * @brief Safety‑related checks and feedback routines.
 *
 * This header declares functions that verify sensor health, perform
 * pre‑arm checks, handle automatic failsafes, and run calibration
 * procedures. All audio/LED feedback is provided through the
 * BeepPattern helper and LEDControl.
 */bool critical_sensor_check_with_feedback();
bool enhanced_safety_check_for_arming_with_feedback();
void check_automatic_battery_failsafe();
void handle_rc_signal_loss();

// Calibration function declarations
bool perform_gyro_calibration();
bool perform_accelerometer_calibration();
void save_calibration_to_eeprom();

// Battery detection function declaration
bool detect_battery_connection();

#endif // SAFETY_FUNCTIONS_H
