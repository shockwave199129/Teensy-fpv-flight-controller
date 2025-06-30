#include "flight_modes.h"
#include "Arduino.h"
#include "sensor_redundancy.h"
#include "constants.h"
#include <math.h>

extern SensorRedundancySystem sensor_redundancy;

void FlightModes::init() {
  current_mode = FLIGHT_MODE_STABILIZE;
  previous_mode = FLIGHT_MODE_STABILIZE;
  mode_change_time = 0;
  home_set = false;
  
  Serial.println("Flight modes initialized - default: STABILIZE");
}

void FlightModes::update(RcData& rc_data, SensorData& sensor_data) {
  // Set home position if GPS fix is available and not already set
  if (!home_set && sensor_data.gps.fix && sensor_data.gps.satellites >= 6) {
    set_home_position(sensor_data.gps.latitude, sensor_data.gps.longitude, sensor_data.gps.altitude);
  }
  
  // Handle mode-specific logic
  switch (current_mode) {
    case FLIGHT_MODE_MANUAL:
      // No additional processing needed - direct rate control
      break;
      
    case FLIGHT_MODE_STABILIZE:
      // No additional processing needed - angle stabilization
      break;
      
    case FLIGHT_MODE_ALTITUDE_HOLD:
      // Altitude hold logic is handled in PID controller
      break;
      
    case FLIGHT_MODE_POSITION_HOLD:
      {
        // Check if GPS is available or if we have synthetic GPS
        /* capability info not currently used */
        (void)sensor_redundancy.get_flight_capability();
        SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
        
        bool position_available = false;
        float current_lat, current_lon;
        
        if (sensor_data.gps.fix) {
          // Use real GPS
          current_lat = sensor_data.gps.latitude;
          current_lon = sensor_data.gps.longitude;
          position_available = true;
          Serial.println("Position hold: Using GPS position");
        } else if (synthetic.synthetic_gps_valid && synthetic.gps_confidence > 0.3) {
          // Use synthetic GPS
          current_lat = synthetic.synthetic_lat;
          current_lon = synthetic.synthetic_lon;
          position_available = true;
          Serial.println("Position hold: Using synthetic GPS position");
        }
        
        if (position_available) {
          static bool target_set = false;
          static float target_lat = 0.0f, target_lon = 0.0f;
          if (!target_set) { target_lat = current_lat; target_lon = current_lon; target_set = true; }

          //---------------------------------------------
          // Basic Position-Hold implementation
          //---------------------------------------------
          // Constants (tune in config.h if desired)
          const float PH_KP          = 0.02f;  // deg per meter
          const float PH_MAX_ANGLE   = 15.0f;  // deg
          const float DEADBAND_M     = 1.0f;   // neutral radius

          // Compute N-E displacement from target (flat-Earth approx)
          float dLat = (target_lat - current_lat) * 111320.0f; // meters
          float dLon = (target_lon - current_lon) * 111320.0f * cos(current_lat * PI / 180.0f);

          // Apply dead-band
          if (fabsf(dLat) < DEADBAND_M) dLat = 0.0f;
          if (fabsf(dLon) < DEADBAND_M) dLon = 0.0f;

          // Convert to desired pitch/roll angles (earth frame -> body frame)
          float desired_pitch_deg =   constrain(-PH_KP * dLat, -PH_MAX_ANGLE, PH_MAX_ANGLE); // north error => nose down (negative pitch)
          float desired_roll_deg  =   constrain( PH_KP * dLon, -PH_MAX_ANGLE, PH_MAX_ANGLE); // east  error => roll right (positive)

          // Map to RC pwm (1500 center, ±500 = ±max angle)
          rc_data.pitch = 1500 + (int)(desired_pitch_deg / PH_MAX_ANGLE * 500.0f);
          rc_data.roll  = 1500 + (int)(desired_roll_deg  / PH_MAX_ANGLE * 500.0f);

          // Altitude hold already active; yaw left unchanged
        } else {
          Serial.println("Position hold: No position source available - switching to altitude hold");
          current_mode = FLIGHT_MODE_ALTITUDE_HOLD; // Fallback
        }
      }
      break;
      
    case FLIGHT_MODE_RETURN_TO_HOME:
      if (home_set && sensor_data.gps.fix) {
        return_to_home(sensor_data, rc_data);
      } else {
        Serial.println("RTH: No home position or GPS fix - switching to stabilize");
        current_mode = FLIGHT_MODE_STABILIZE;
      }
      break;
      
    case FLIGHT_MODE_HEADLESS:
      // Headless mode logic
      {
        static bool offset_set = false;
        static float heading_offset_deg = 0.0f;

        if (!offset_set) {
          heading_offset_deg = sensor_data.yaw; // lock when first entered
          offset_set = true;
          Serial.print("Headless mode activated. Heading offset locked at ");
          Serial.println(heading_offset_deg);
        }

        // Transform current RC inputs (roll/pitch) to earth-frame
        float cosH = cosf((sensor_data.yaw - heading_offset_deg) * DEG_TO_RAD);
        float sinH = sinf((sensor_data.yaw - heading_offset_deg) * DEG_TO_RAD);

        // Convert pwm to normalized (-1..1)
        float x = (rc_data.roll  - 1500) / 500.0f;  // right = +1
        float y = (rc_data.pitch - 1500) / 500.0f;  // nose up = +1 (but forward is negative pitch in our mapping)

        // Rotate stick vector
        float xE =  x * cosH - y * sinH;
        float yE =  x * sinH + y * cosH;

        // Map back to pwm
        rc_data.roll  = 1500 + (int)(xE * 500.0f);
        rc_data.pitch = 1500 + (int)(yE * 500.0f);
      }
      break;
  }
}

void FlightModes::check_mode_switch(RcData& rc_data, SensorData& sensor_data) {
  FlightMode new_mode = current_mode;
  
  // Use AUX1 channel for mode switching
  if (rc_data.aux1 < 1300) {
    new_mode = FLIGHT_MODE_MANUAL;
  } else if (rc_data.aux1 < 1500) {
    new_mode = FLIGHT_MODE_STABILIZE;
  } else if (rc_data.aux1 < 1700) {
    new_mode = FLIGHT_MODE_ALTITUDE_HOLD;
  } else {
    new_mode = FLIGHT_MODE_POSITION_HOLD;
  }
  
  // Check for RTH on AUX2
  if (rc_data.aux2 > 1700) {
    new_mode = FLIGHT_MODE_RETURN_TO_HOME;
  }
  
  // Check for headless mode on AUX3
  if (rc_data.aux3 > 1700) {
    new_mode = FLIGHT_MODE_HEADLESS;
  }
  
  // GPS REQUIREMENT CHECKS - Enhanced to include synthetic GPS
  bool dedicated_gps_available = (sensor_data.gps.healthy && sensor_data.gps.fix && sensor_data.gps.satellites >= 6);
  SyntheticSensorData synthetic = sensor_redundancy.get_synthetic_data();
  bool synthetic_gps_available = (synthetic.synthetic_gps_valid && synthetic.gps_confidence > 0.3);
  bool gps_functionality_available = dedicated_gps_available || synthetic_gps_available;
  
  // Check if new mode requires GPS functionality
  if (new_mode == FLIGHT_MODE_POSITION_HOLD || new_mode == FLIGHT_MODE_RETURN_TO_HOME) {
    if (!gps_functionality_available) {
      // Block GPS-dependent mode switch
      Serial.print("Mode switch BLOCKED: ");
      switch (new_mode) {
        case FLIGHT_MODE_POSITION_HOLD:
          Serial.println("Position Hold requires GPS functionality");
          break;
        case FLIGHT_MODE_RETURN_TO_HOME:
          Serial.println("Return to Home requires GPS functionality");
          break;
        case FLIGHT_MODE_HEADLESS:
          Serial.println("Headless mode requires GPS functionality (heading reference)");
          break;
        case FLIGHT_MODE_MANUAL:
        case FLIGHT_MODE_STABILIZE:
        case FLIGHT_MODE_ALTITUDE_HOLD:
          Serial.println("Selected mode does not require GPS");
          break;
        default:
          Serial.println("Unknown mode requires GPS functionality");
          break;
      }
      
      Serial.println("GPS functionality requires either:");
      Serial.println("  - Dedicated GPS with 6+ satellites, OR");
      Serial.println("  - Synthetic GPS with >30% confidence");
      
      Serial.print("Current status: ");
      if (!sensor_data.gps.healthy) {
        Serial.print("No dedicated GPS detected");
      } else if (!sensor_data.gps.fix) {
        Serial.print("Dedicated GPS no fix (");
        Serial.print(sensor_data.gps.satellites);
        Serial.print(" satellites)");
      } else {
        Serial.print("Dedicated GPS insufficient satellites (");
        Serial.print(sensor_data.gps.satellites);
        Serial.print(" < 6)");
      }
      
      if (synthetic.synthetic_gps_valid) {
        Serial.print(", Synthetic GPS available (");
        Serial.print(synthetic.gps_confidence * 100, 1);
        Serial.print("% confidence - ");
        if (synthetic.gps_confidence > 0.3) {
          Serial.println("sufficient)");
        } else {
          Serial.println("insufficient)");
        }
      } else {
        Serial.println(", No synthetic GPS available");
      }
      
      Serial.println("Remaining in current flight mode");
      return; // Don't change mode
    } else {
      // GPS functionality available - log source
      if (dedicated_gps_available) {
        Serial.println("GPS mode available: Using dedicated GPS");
      } else if (synthetic_gps_available) {
        Serial.print("GPS mode available: Using synthetic GPS (");
        Serial.print(synthetic.gps_confidence * 100, 1);
        Serial.println("% confidence)");
      }
    }
  }
  
  // Special check for RTH - also needs home position set
  if (new_mode == FLIGHT_MODE_RETURN_TO_HOME) {
    if (!home_set) {
      Serial.println("Mode switch BLOCKED: Return to Home requires home position to be set");
      Serial.println("Home position will be set automatically when GPS fix is acquired");
      Serial.println("Remaining in current flight mode");
      return; // Don't change mode
    }
  }
  
  // Change mode if different and all checks passed
  if (new_mode != current_mode) {
    previous_mode = current_mode;
    current_mode = new_mode;
    mode_change_time = millis();
    
    Serial.print("Flight mode changed to: ");
    switch (current_mode) {
      case FLIGHT_MODE_MANUAL:
        Serial.println("MANUAL");
        break;
      case FLIGHT_MODE_STABILIZE:
        Serial.println("STABILIZE");
        break;
      case FLIGHT_MODE_ALTITUDE_HOLD:
        Serial.println("ALTITUDE HOLD");
        break;
      case FLIGHT_MODE_POSITION_HOLD:
        Serial.println("POSITION HOLD");
        break;
      case FLIGHT_MODE_RETURN_TO_HOME:
        Serial.println("RETURN TO HOME");
        break;
      case FLIGHT_MODE_HEADLESS:
        Serial.println("HEADLESS MODE");
        break;
      default:
        Serial.println("UNKNOWN MODE");
        break;
    }
  }
}

// STUB IMPLEMENTATIONS for functions not fully visible in the source file
void FlightModes::set_home_position(float lat, float lon, float alt) {
  home_latitude = lat;
  home_longitude = lon;
  home_altitude = alt;
  home_set = true;
  Serial.println("Home position set.");
}

void FlightModes::return_to_home(SensorData& sensor_data, RcData& rc_data) {
  if (!home_set) {
    Serial.println("RTH aborted: no home position");
    return;
  }

  // Constants
  const float DESCENT_ALTITUDE = home_altitude + 2.0f;  // target altitude 2 m above home
  const float LAND_RADIUS_M = 1.5f;                     // switch to land if within 1.5 m

  // Calculate 2-D distance to home (rough flat-earth)
  float dLat = (home_latitude  - sensor_data.gps.latitude)  * 111320.0f; // meters per deg lat
  float dLon = (home_longitude - sensor_data.gps.longitude) * 111320.0f * cos(sensor_data.gps.latitude * PI/180.0f);
  float distance2D = sqrtf(dLat*dLat + dLon*dLon);

  // Bearing to home (0=N)
  float bearing = atan2f(dLon, dLat) * 180.0f / PI;
  if (bearing < 0) bearing += 360.0f;

  // Desired yaw towards home
  float yaw_error = bearing - sensor_data.yaw;
  while (yaw_error > 180.0f) yaw_error -= 360.0f;
  while (yaw_error < -180.0f) yaw_error += 360.0f;

  // PI controller for yaw (very lightweight)
  static float yaw_i = 0.0f;
  float yaw_p = 0.01f * yaw_error;
  yaw_i += 0.0001f * yaw_error;
  float yaw_output = constrain(yaw_p + yaw_i, -0.3f, 0.3f); // -0.3 .. 0.3 rad/s

  // Throttle to maintain / descend to target altitude
  float alt_error = DESCENT_ALTITUDE - sensor_data.baro.altitude;
  float throttle_output = constrain(0.5f + 0.05f * alt_error, 0.3f, 0.7f);

  // Pitch forward to move toward home until close
  float pitch_output = constrain( -0.02f * distance2D , -0.2f, 0.0f); // negative pitch (forward)

  // When close, start landing by reducing throttle slowly
  static bool landing_phase = false;
  if (distance2D < LAND_RADIUS_M) landing_phase = true;
  if (landing_phase) throttle_output = max(0.3f, throttle_output - 0.001f);

  // Feed into rc_data legacy structure (1000-2000 range)
  rc_data.roll     = 1500; // keep level
  rc_data.pitch    = 1500 + (int)(pitch_output * 500.0f);
  rc_data.yaw      = 1500 + (int)(yaw_output * 500.0f);
  rc_data.throttle = (int)(throttle_output * 1000.0f) + 1000;

  // Safety: disarm when landed (simple criterion current altitude below home and throttle low)
  if (landing_phase && sensor_data.baro.altitude < home_altitude + 0.2f && throttle_output <= 0.32f) {
    Serial.println("RTH: Landing complete, switching to DISARMED");
    // Additional disarm logic handled elsewhere (motor_control.disarm())
    current_mode = FLIGHT_MODE_STABILIZE;
    landing_phase = false;
  }
} 