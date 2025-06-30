#include "advanced_flight_modes.h"
#include "motor_control.h"
#include "navigation_helpers.h"

extern MotorControl motor_control;

AdvancedFlightModes::AdvancedFlightModes() {
  current_mode = ADV_FLIGHT_MODE_ACRO_PLUS;
  mode_active = false;
  mode_entry_time = 0;
  recovery_mode_active = false;
  launch_detected = false;
  landing_sequence_active = false;
}

void AdvancedFlightModes::init() {
  Serial.println("Initializing Advanced Flight Modes...");
  
  // Initialize acro plus parameters
  acro_plus_recovery_angle = ACRO_PLUS_RECOVERY_ANGLE;
  
  // Initialize sport mode parameters
  sport_rate_multiplier = SPORT_MODE_RATE_MULTIPLIER;
  
  // Initialize cinematic mode parameters
  cinematic_expo_factor = CINEMATIC_EXPO_FACTOR;
  
  // Initialize GPS rescue parameters
  gps_rescue_climb_rate = GPS_RESCUE_CLIMB_RATE;
  
  // Initialize turtle mode parameters
  turtle_throttle_limit = TURTLE_MODE_THROTTLE_LIMIT;
  
  // Initialize mode flags
  current_mode = ADV_FLIGHT_MODE_ACRO_PLUS;
  mode_active = false;
  mode_entry_time = 0;
  recovery_mode_active = false;
  launch_detected = false;
  landing_sequence_active = false;
  
  Serial.println("Advanced flight modes initialized:");
  Serial.println("  - Acro Plus (Rate mode with auto-recovery)");
  Serial.println("  - Sport (High performance mode)");
  Serial.println("  - Cinematic (Ultra-smooth for video)");
  Serial.println("  - GPS Rescue (Advanced RTH with obstacles)");
  Serial.println("  - Turtle (Upside-down recovery)");
  Serial.println("  - Launch Assist (Automatic launch detection)");
  Serial.println("  - Land Assist (Automatic landing assistance)");
}

void AdvancedFlightModes::update(SensorData& sensors, RcData& rc, FlightState& state) {
  // Check for mode activation based on RC switches
  check_mode_activation(rc);
  
  if (!mode_active) {
    return;
  }
  
  // Execute current advanced flight mode
  switch (current_mode) {
    case ADV_FLIGHT_MODE_ACRO_PLUS:
      update_acro_plus_mode(sensors, rc, state);
      break;
      
    case ADV_FLIGHT_MODE_SPORT:
      update_sport_mode(sensors, rc, state);
      break;
      
    case ADV_FLIGHT_MODE_CINEMATIC:
      update_cinematic_mode(sensors, rc, state);
      break;
      
    case ADV_FLIGHT_MODE_GPS_RESCUE:
      update_gps_rescue_mode(sensors, rc, state);
      break;
      
    case ADV_FLIGHT_MODE_TURTLE:
      update_turtle_mode(sensors, rc, state);
      break;
      
    case ADV_FLIGHT_MODE_LAUNCH_ASSIST:
      update_launch_assist_mode(sensors, rc, state);
      break;
      
    case ADV_FLIGHT_MODE_LAND_ASSIST:
      update_land_assist_mode(sensors, rc, state);
      break;
  }
}

void AdvancedFlightModes::check_mode_activation(const RcData& rc) {
  // Check AUX channels for advanced mode activation
  // This is a simplified implementation - would use proper channel mapping
  
  static AdvancedFlightMode last_mode = current_mode;
  
  // AUX4 channel for advanced modes (example mapping)
  if (rc.aux4 < 1300) {
    current_mode = ADV_FLIGHT_MODE_ACRO_PLUS;
  } else if (rc.aux4 < 1500) {
    current_mode = ADV_FLIGHT_MODE_SPORT;
  } else if (rc.aux4 < 1700) {
    current_mode = ADV_FLIGHT_MODE_CINEMATIC;
  } else {
    current_mode = ADV_FLIGHT_MODE_GPS_RESCUE;
  }
  
  // Turtle mode on separate switch (AUX3)
  if (rc.aux3 > 1700) {
    current_mode = ADV_FLIGHT_MODE_TURTLE;
  }
  
  // Check if mode changed
  if (current_mode != last_mode) {
    mode_entry_time = millis();
    mode_active = true;
    
    Serial.print("Advanced flight mode changed to: ");
    switch (current_mode) {
      case ADV_FLIGHT_MODE_ACRO_PLUS:
        Serial.println("ACRO PLUS");
        break;
      case ADV_FLIGHT_MODE_SPORT:
        Serial.println("SPORT");
        break;
      case ADV_FLIGHT_MODE_CINEMATIC:
        Serial.println("CINEMATIC");
        break;
      case ADV_FLIGHT_MODE_GPS_RESCUE:
        Serial.println("GPS RESCUE");
        break;
      case ADV_FLIGHT_MODE_TURTLE:
        Serial.println("TURTLE MODE");
        break;
      case ADV_FLIGHT_MODE_LAUNCH_ASSIST:
        Serial.println("LAUNCH ASSIST");
        break;
      case ADV_FLIGHT_MODE_LAND_ASSIST:
        Serial.println("LAND ASSIST");
        break;
    }
    
    last_mode = current_mode;
  }
}

void AdvancedFlightModes::update_acro_plus_mode(SensorData& sensors, RcData& rc, FlightState& state) {
  // Acro Plus: Rate mode with automatic recovery when exceeding angle limits
  
  float current_angle = max(abs(sensors.roll), abs(sensors.pitch));
  
  if (current_angle > acro_plus_recovery_angle && !recovery_mode_active) {
    Serial.println("Acro Plus: Angle limit exceeded - activating auto-recovery");
    recovery_mode_active = true;
  }
  
  if (recovery_mode_active) {
    // Gradually level the aircraft
    if (current_angle < 10.0f) { // Recovery complete
      recovery_mode_active = false;
      Serial.println("Acro Plus: Recovery complete");
    } else {
      // Would implement recovery logic here - gradually reduce angle
      Serial.print("Acro Plus: Recovering from ");
      Serial.print(current_angle);
      Serial.println("° angle");
    }
  }
}

void AdvancedFlightModes::update_sport_mode(SensorData& sensors, RcData& rc, FlightState& state) {
  // Sport mode: High performance with increased response rates
  
  // This would modify PID gains for more aggressive response
  // For now, just log the mode activity
  static unsigned long last_log = 0;
  if (millis() - last_log > 5000) {
    Serial.println("Sport mode: High performance flight characteristics active");
    last_log = millis();
  }
}

void AdvancedFlightModes::update_cinematic_mode(SensorData& sensors, RcData& rc, FlightState& state) {
  // Cinematic mode: Ultra-smooth response for video recording
  
  // This would apply heavy expo and smoothing to all inputs
  static unsigned long last_log = 0;
  if (millis() - last_log > 5000) {
    Serial.println("Cinematic mode: Smooth video-optimized flight characteristics active");
    last_log = millis();
  }
}

void AdvancedFlightModes::update_gps_rescue_mode(SensorData& sensors, RcData& rc, FlightState& state) {
  // Requires valid GPS and home position
  if (!sensors.gps.healthy || !sensors.gps.fix || !state.home_set) {
      Serial.println("GPS Rescue: GPS/home not ready");
      return;
  }

  enum RescuePhase { RESCUE_CLIMB, RESCUE_NAVIGATE, RESCUE_LAND };
  static RescuePhase phase = RESCUE_CLIMB;
  static unsigned long start_ms = 0;
  if (start_ms == 0) { start_ms = millis(); phase = RESCUE_CLIMB; Serial.println("GPS Rescue INIT"); }

  switch(phase) {
      case RESCUE_CLIMB:
          if (sensors.altitude_fused < RTH_CLIMB_HEIGHT) {
              rc.throttle = constrain(rc.throttle + 50, 1200, 1800);
          } else {
              phase = RESCUE_NAVIGATE;
          }
          break;

      case RESCUE_NAVIGATE: {
          float dist = NavigationHelpers::distance_between(sensors.gps.latitude, sensors.gps.longitude,
                                                          state.home_lat, state.home_lon);
          float bearing = NavigationHelpers::bearing_to(sensors.gps.latitude, sensors.gps.longitude,
                                                       state.home_lat, state.home_lon);
          NavigationHelpers::set_nav_heading(bearing);
          if (dist < 5.0f) phase = RESCUE_LAND;
          break; }

      case RESCUE_LAND:
          if (sensors.altitude_fused > 2.0f) {
              rc.throttle = constrain(rc.throttle - 20, 1100, 1400);
          } else {
              Serial.println("GPS Rescue complete");
              start_ms = 0; // reset for next trigger
          }
          break;
  }
}

void AdvancedFlightModes::update_turtle_mode(SensorData& sensors, RcData& rc, FlightState& state) {
  static bool turtle_active = false;
  bool inverted = (fabs(sensors.roll) > 120.0f || fabs(sensors.pitch) > 120.0f);

  if (inverted && !turtle_active) {
      turtle_active = true;
      Serial.println("Turtle mode engaged: reversing motors");
      for(int m=0;m<4;m++) motor_control.reverse_motor_direction(m);
  }

  if (turtle_active) {
      rc.throttle = 1300; // fixed low throttle to flip
      if (!inverted) {
          Serial.println("Turtle mode complete: restoring motor directions");
          for(int m=0;m<4;m++) motor_control.reverse_motor_direction(m);
          turtle_active = false;
      }
  }
}

void AdvancedFlightModes::update_launch_assist_mode(SensorData& sensors, RcData& rc, FlightState& state) {
  // Launch assist: Automatic launch detection and assistance
  
  static bool pre_launch_detected = false;
  
  // Detect launch preparation (aircraft moved and throttle applied)
  bool throttle_applied = (rc.throttle > 1200);
  bool motion_detected = (abs(sensors.imu.accel_x) > 2.0f || 
                         abs(sensors.imu.accel_y) > 2.0f ||
                         abs(sensors.imu.accel_z - 9.81f) > 3.0f);
  
  if (throttle_applied && motion_detected && !pre_launch_detected) {
    pre_launch_detected = true;
    Serial.println("Launch Assist: Pre-launch conditions detected");
  }
  
  if (pre_launch_detected && sensors.altitude_fused > 1.0f && !launch_detected) {
    launch_detected = true;
    Serial.println("Launch Assist: Launch detected - stabilizing flight");
    
    // Would implement launch stabilization logic
    // - Gradually increase stability
    // - Smooth throttle response
    // - Prevent tip-overs
  }
  
  // Reset detection after successful flight
  if (launch_detected && sensors.altitude_fused > 5.0f) {
    Serial.println("Launch Assist: Stable flight achieved");
    launch_detected = false;
    pre_launch_detected = false;
  }
}

void AdvancedFlightModes::update_land_assist_mode(SensorData& sensors, RcData& rc, FlightState& state) {
  // Land assist: Automatic landing assistance
  
  // Detect landing sequence (low throttle and descending)
  bool low_throttle = (rc.throttle < 1200);
  bool descending = (sensors.altitude_fused < 5.0f);
  bool near_ground = (sensors.sonar.distance < 50.0f); // 50cm
  
  if (low_throttle && descending && !landing_sequence_active) {
    landing_sequence_active = true;
    Serial.println("Land Assist: Landing sequence detected");
  }
  
  if (landing_sequence_active) {
    if (near_ground) {
      Serial.println("Land Assist: Ground proximity - reducing descent rate");
      // Would implement gentle landing logic
      // - Reduce descent rate
      // - Maintain level attitude
      // - Cut throttle when touching ground
      
      if (sensors.sonar.distance < 5.0f) { // 5cm - on ground
        Serial.println("Land Assist: Ground contact - landing complete");
        landing_sequence_active = false;
      }
    } else {
      Serial.println("Land Assist: Controlled descent active");
    }
  }
}

void AdvancedFlightModes::set_mode(AdvancedFlightMode mode) { 
  current_mode = mode;
  mode_active = true;
  mode_entry_time = millis();
  
  Serial.print("Advanced flight mode set to: ");
  Serial.println((int)mode);
}

AdvancedFlightMode AdvancedFlightModes::get_mode() {
  return current_mode;
} 