#include "led_control.h"
#include "Arduino.h"

void LEDControl::init() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
  
  current_status = LED_DISARMED;
  last_update = 0;
  animation_step = 0;
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println("LED control initialized");
}

void LEDControl::update() {
  unsigned long current_time = millis();
  
  if (current_time - last_update < 50) { // Update at 20Hz
    return;
  }
  
  last_update = current_time;
  animation_step++;
  
  switch (current_status) {
    case LED_DISARMED:
      // Solid red
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      break;
      
    case LED_ARMED:
      // Solid green
      fill_solid(leds, NUM_LEDS, CRGB::Green);
      break;
      
    case LED_GPS_LOCK:
      // Blinking blue
      if (animation_step % 20 < 10) {
        fill_solid(leds, NUM_LEDS, CRGB::Blue);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
      }
      break;
      
    case LED_RTH_ACTIVE:
      // Purple pulse
      {
        int brightness = sin8(animation_step * 8);
        CRGB color = CRGB::Purple;
        color.nscale8(brightness);
        fill_solid(leds, NUM_LEDS, color);
      }
      break;
      
    case LED_LOW_BATTERY:
      // Fast red blink
      if (animation_step % 6 < 3) {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
      }
      break;
      
    case LED_ERROR:
      // Alternating red/white strobe
      if (animation_step % 4 < 2) {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::White);
      }
      break;
  }
  
  FastLED.show();
}

void LEDControl::set_status(LedStatus status) {
  if (current_status != status) {
    current_status = status;
    animation_step = 0; // Reset animation
    
    Serial.print("LED status changed to: ");
    switch (status) {
      case LED_DISARMED:
        Serial.println("DISARMED (Red)");
        break;
      case LED_ARMED:
        Serial.println("ARMED (Green)");
        break;
      case LED_GPS_LOCK:
        Serial.println("GPS LOCK (Blinking Blue)");
        break;
      case LED_RTH_ACTIVE:
        Serial.println("RTH ACTIVE (Purple)");
        break;
      case LED_LOW_BATTERY:
        Serial.println("LOW BATTERY (Fast Red Blink)");
        break;
      case LED_ERROR:
        Serial.println("ERROR (Red/White Strobe)");
        break;
    }
  }
}

void LEDControl::update_health_status(SensorData& sensor_data, FlightState& flight_state) {
  // Check for GPS lock status
  if (sensor_data.gps.fix && sensor_data.gps.satellites >= 6) {
    if (current_status == LED_DISARMED || current_status == LED_GPS_LOCK) {
      // Don't override armed state or error states
      if (!flight_state.armed) {
        set_status(LED_GPS_LOCK);
      }
    }
  }
  
  // Check for RTH mode
  if (flight_state.flight_mode == FLIGHT_MODE_RETURN_TO_HOME) {
    set_status(LED_RTH_ACTIVE);
  }
  
  // Check for low battery
  if (sensor_data.battery_voltage < LOW_VOLTAGE_THRESHOLD) {
    set_status(LED_LOW_BATTERY);
  }
  
  // Check for sensor errors
  if (!sensor_data.imu.healthy) {
    set_status(LED_ERROR);
  }
}

void LEDControl::show_custom_pattern(CRGB color, int pattern) {
  switch (pattern) {
    case 0: // Rainbow
      fill_rainbow(leds, NUM_LEDS, animation_step * 2, 20);
      break;
      
    case 1: // Chase
      {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        int pos = animation_step % NUM_LEDS;
        leds[pos] = color;
        if (pos > 0) leds[pos - 1] = color.nscale8(128);
        if (pos < NUM_LEDS - 1) leds[pos + 1] = color.nscale8(128);
      }
      break;
      
    case 2: // Breathe
      {
        int brightness = sin8(animation_step * 4);
        CRGB scaled_color = color;
        scaled_color.nscale8(brightness);
        fill_solid(leds, NUM_LEDS, scaled_color);
      }
      break;
      
    case 3: // Sparkle
      {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        for (int i = 0; i < 3; i++) {
          int pos = random(NUM_LEDS);
          leds[pos] = color;
        }
      }
      break;
      
    default:
      fill_solid(leds, NUM_LEDS, color);
      break;
  }
  
  FastLED.show();
}

void LEDControl::flash_error() {
  // Emergency error flash - immediate override of current pattern
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(100);
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  delay(100);
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(100);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
} 