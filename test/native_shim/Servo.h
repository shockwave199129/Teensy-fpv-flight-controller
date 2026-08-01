#pragma once
// Minimal native stand-in for the Arduino Servo library - just enough for
// MotorControl (motor_control.h) to declare `Servo motors[MAX_MOTORS];` and
// compile under the native test environment. No real PWM output.

class Servo {
public:
  void attach(int) {}
  void attach(int, int, int) {}
  void detach() {}
  void write(int) {}
  void writeMicroseconds(int) {}
  int read() const { return 0; }
  bool attached() const { return false; }
};
