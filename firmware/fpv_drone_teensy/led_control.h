#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "config.h"
#include <FastLED.h>

class LEDControl {
private:
  CRGB leds[NUM_LEDS];
  LedStatus current_status;
  unsigned long last_update;
  int animation_step;
  
public:
  void init();
  void update();
  void set_status(LedStatus status);
  void update_health_status(SensorData& sensor_data, FlightState& flight_state);
  void show_custom_pattern(CRGB color, int pattern);
  void flash_error(); // Emergency error flash
};

#endif // LED_CONTROL_H
