#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "config.h"
// Forward declarations
class PIDController;
class MotorControl;
class FlightModes;
class LEDControl;
class DualIMUManager;
class DynamicFilteringSystem;
class OpticalFlowSensor;
class AdvancedFlightModes;


class Communication {
private:
  String command_buffer;
  unsigned long last_telemetry_send;
  
  void process_command(String command);
  void send_telemetry();
  void print_help();
  void print_status();
  
public:
  void init();
  void process_commands();
  void send_data(String data);
};

// External references to global objects
extern SensorData sensor_data;
extern RcData rc_data;
extern FlightState flight_state;
extern PIDController pid_controller;
extern MotorControl motor_control;
extern FlightModes flight_modes;
extern LEDControl led_control;
extern bool armed;

// External references to Phase 2 global objects
extern DualIMUManager dual_imu_manager;
extern DynamicFilteringSystem dynamic_filtering;
extern OpticalFlowSensor optical_flow;
extern AdvancedFlightModes advanced_flight_modes;

#endif // COMMUNICATION_H

