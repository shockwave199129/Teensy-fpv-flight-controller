#ifndef FLIGHT_MODES_H
#define FLIGHT_MODES_H

#include "config.h"

class FlightModes {
private:
  FlightMode current_mode;
  FlightMode previous_mode;
  unsigned long mode_change_time;
  
  // Home position for RTH
  float home_latitude;
  float home_longitude;
  float home_altitude;
  bool home_set;
  
public:
  void init();
  void update(RcData& rc_data, SensorData& sensor_data);
  void check_mode_switch(RcData& rc_data, SensorData& sensor_data);
  FlightMode get_current_mode() { return current_mode; }
  void set_home_position(float lat, float lon, float alt);
  void return_to_home(SensorData& sensor_data, RcData& rc_data);
};

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
      // Check if GPS is available or if we have synthetic GPS
      extern SensorRedundancySystem sensor_redundancy;
      FlightCapability capability = sensor_redundancy.get_flight_capability();
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
        // Position hold logic with available position source
        static float target_lat = 0, target_lon = 0;
        static bool position_set = false;
        
        if (!position_set) {
          target_lat = current_lat;
          target_lon = current_lon;
          position_set = true;
          Serial.println("Position hold target set");
        }
        
        // Calculate position error (simplified)
        float lat_error = target_lat - current_lat;
        float lon_error = target_lon - current_lon;
        
        // Convert to distance (very simplified)
        float distance_error = sqrt(lat_error * lat_error + lon_error * lon_error) * 111000; // rough meters
        
        if (distance_error > 5.0) { // More than 5m off
          Serial.println("Position hold: correcting position");
          // Position correction would be implemented here
        }
      } else {
        Serial.println("Position hold: No position source available - switching to altitude hold");
        current_mode = FLIGHT_MODE_ALTITUDE_HOLD; // Fallback to altitude hold instead of stabilize
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
      // Headless mode logic - reference direction is locked to home
      if (home_set) {
        // Calculate heading offset from home
        float heading_offset = atan2(sensor_data.gps.longitude - home_longitude,
                                   sensor_data.gps.latitude - home_latitude) * 180.0 / PI;
        
        // Adjust control inputs based on heading offset
        // This would modify RC inputs before they go to PID controller
        Serial.print("Headless mode: heading offset = ");
        Serial.println(heading_offset);
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
        Serial.println("GPS-based position hold active");
        break;
      case FLIGHT_MODE_RETURN_TO_HOME:
        Serial.println("RETURN TO HOME");
        Serial.print("Returning to home: ");
        Serial.print(home_latitude, 6);
        Serial.print(", ");
        Serial.println(home_longitude, 6);
        break;
      case FLIGHT_MODE_HEADLESS:
        Serial.println("HEADLESS");
        break;
    }
  }
}

void FlightModes::set_home_position(float lat, float lon, float alt) {
  home_latitude = lat;
  home_longitude = lon;
  home_altitude = alt;
  home_set = true;
  
  Serial.println("Home position set:");
  Serial.print("  Latitude: ");
  Serial.println(lat, 6);
  Serial.print("  Longitude: ");
  Serial.println(lon, 6);
  Serial.print("  Altitude: ");
  Serial.println(alt);
}

void FlightModes::return_to_home(SensorData& sensor_data, RcData& rc_data) {
  static enum RTHState {
    RTH_CLIMB,
    RTH_NAVIGATE,
    RTH_DESCEND,
    RTH_LAND
  } rth_state = RTH_CLIMB;
  
  static unsigned long rth_start_time = 0;
  
  if (rth_start_time == 0) {
    rth_start_time = millis();
    rth_state = RTH_CLIMB;
    Serial.println("RTH: Starting return to home sequence");
  }
  
  float distance_to_home = sqrt(pow((sensor_data.gps.latitude - home_latitude) * 111000, 2) +
                               pow((sensor_data.gps.longitude - home_longitude) * 111000, 2));
  
  Serial.print("RTH: Distance to home: ");
  Serial.print(distance_to_home);
  Serial.println("m");
  
  switch (rth_state) {
    case RTH_CLIMB:
      if (sensor_data.altitude_fused < home_altitude + 20) { // Climb 20m above home
        Serial.println("RTH: Climbing to safe altitude");
        // Climbing logic would be handled by altitude controller
      } else {
        rth_state = RTH_NAVIGATE;
        Serial.println("RTH: Safe altitude reached, navigating home");
      }
      break;
      
    case RTH_NAVIGATE:
      if (distance_to_home > 5.0) { // More than 5m from home
        Serial.println("RTH: Navigating towards home");
        // Navigation logic would calculate bearing and adjust position
        
        float bearing = atan2(home_longitude - sensor_data.gps.longitude,
                             home_latitude - sensor_data.gps.latitude) * 180.0 / PI;
        
        Serial.print("RTH: Bearing to home: ");
        Serial.println(bearing);
        
        // Position correction would be applied here
      } else {
        rth_state = RTH_DESCEND;
        Serial.println("RTH: Reached home position, beginning descent");
      }
      break;
      
    case RTH_DESCEND:
      if (sensor_data.altitude_fused > home_altitude + 2) {
        Serial.println("RTH: Descending to landing altitude");
        // Descent logic
      } else {
        rth_state = RTH_LAND;
        Serial.println("RTH: Beginning landing sequence");
      }
      break;
      
    case RTH_LAND:
      if (sensor_data.sonar.distance > 10) { // More than 10cm above ground
        Serial.println("RTH: Landing...");
        // Landing logic - slow descent
      } else {
        Serial.println("RTH: Landing complete - disarming");
        // Would trigger disarm sequence
        rth_start_time = 0; // Reset for next RTH
        current_mode = FLIGHT_MODE_STABILIZE;
      }
      break;
  }
  
  // Emergency abort RTH if pilot takes control
  if (rc_data.throttle > 1600 || abs(rc_data.roll - 1500) > 200 || 
      abs(rc_data.pitch - 1500) > 200 || abs(rc_data.yaw - 1500) > 200) {
    Serial.println("RTH: Pilot override detected - aborting RTH");
    rth_start_time = 0;
    current_mode = FLIGHT_MODE_STABILIZE;
  }
}

#endif // FLIGHT_MODES_H
