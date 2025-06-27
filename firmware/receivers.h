#ifndef RECEIVERS_H
#define RECEIVERS_H

#include "config.h"

// Global variables (extern declarations)
extern RcProtocol current_protocol;
extern RcConfig rc_config;
extern EnhancedRcData enhanced_rc_data;
extern RateProfile rate_profiles[3];
extern FunctionSwitches function_switches;

// Default channel mappings (standard setup)
void init_default_channel_mapping() {
  // Clear all mappings first
  for (int i = 0; i < 16; i++) {
    rc_config.channels[i].channel_number = 0;
    rc_config.channels[i].function = CHAN_FUNC_NONE;
    rc_config.channels[i].reversed = false;
    rc_config.channels[i].min_value = 1000;
    rc_config.channels[i].center_value = 1500;
    rc_config.channels[i].max_value = 2000;
    rc_config.channels[i].deadband = 10;
    rc_config.channels[i].is_switch = false;
    rc_config.channels[i].switch_positions = 2;
    rc_config.channels[i].switch_thresholds[0] = 1300;
    rc_config.channels[i].switch_thresholds[1] = 1700;
  }
  
  // Standard AETR mapping (Aileron, Elevator, Throttle, Rudder)
  // Channel 1: Roll (Aileron)
  rc_config.channels[0].channel_number = 1;
  rc_config.channels[0].function = CHAN_FUNC_ROLL;
  
  // Channel 2: Pitch (Elevator)
  rc_config.channels[1].channel_number = 2;
  rc_config.channels[1].function = CHAN_FUNC_PITCH;
  rc_config.channels[1].reversed = true; // Usually reversed for pitch
  
  // Channel 3: Throttle
  rc_config.channels[2].channel_number = 3;
  rc_config.channels[2].function = CHAN_FUNC_THROTTLE;
  rc_config.channels[2].min_value = 1000;
  rc_config.channels[2].center_value = 1000; // Throttle has no center
  rc_config.channels[2].max_value = 2000;
  
  // Channel 4: Yaw (Rudder)
  rc_config.channels[3].channel_number = 4;
  rc_config.channels[3].function = CHAN_FUNC_YAW;
  
  // Channel 5: Arm/Disarm Switch
  rc_config.channels[4].channel_number = 5;
  rc_config.channels[4].function = CHAN_FUNC_ARM_DISARM;
  rc_config.channels[4].is_switch = true;
  rc_config.channels[4].switch_positions = 2;
  
  // Channel 6: Flight Mode Switch
  rc_config.channels[5].channel_number = 6;
  rc_config.channels[5].function = CHAN_FUNC_FLIGHT_MODE;
  rc_config.channels[5].is_switch = true;
  rc_config.channels[5].switch_positions = 3;
  
  // Channel 7: Auxiliary functions
  rc_config.channels[6].channel_number = 7;
  rc_config.channels[6].function = CHAN_FUNC_RTH;
  rc_config.channels[6].is_switch = true;
  rc_config.channels[6].switch_positions = 2;
  
  // Channel 8: Auxiliary functions
  rc_config.channels[7].channel_number = 8;
  rc_config.channels[7].function = CHAN_FUNC_BEEPER;
  rc_config.channels[7].is_switch = true;
  rc_config.channels[7].switch_positions = 2;
  
  // RC Config defaults
  rc_config.protocol = RC_PROTOCOL_PPM;
  rc_config.channel_count = 8;
  rc_config.failsafe_enabled = true;
  rc_config.failsafe_throttle = 1000;
  rc_config.failsafe_mode = FLIGHT_MODE_STABILIZE;
  rc_config.rth_on_failsafe = false;
  rc_config.stick_arming_enabled = true;
  rc_config.switch_arming_enabled = true;
  rc_config.arm_channel = 5;
  rc_config.arm_sequence_time_ms = 2000;
  rc_config.current_rate_profile = 0;
  rc_config.rate_profile_channel = 0;
}

// Initialize default rate profiles
void init_default_rate_profiles() {
  // Profile 0: Beginner
  strcpy(rate_profiles[0].name, "Beginner");
  rate_profiles[0].max_roll_rate = 200.0;
  rate_profiles[0].max_pitch_rate = 200.0;
  rate_profiles[0].max_yaw_rate = 180.0;
  rate_profiles[0].roll_expo = 0.0;
  rate_profiles[0].pitch_expo = 0.0;
  rate_profiles[0].yaw_expo = 0.0;
  rate_profiles[0].rc_smoothing_factor = 0.7;
  rate_profiles[0].throttle_expo = 0.0;
  rate_profiles[0].throttle_mid = 0.5;
  
  // Profile 1: Sport
  strcpy(rate_profiles[1].name, "Sport");
  rate_profiles[1].max_roll_rate = 400.0;
  rate_profiles[1].max_pitch_rate = 400.0;
  rate_profiles[1].max_yaw_rate = 360.0;
  rate_profiles[1].roll_expo = 0.3;
  rate_profiles[1].pitch_expo = 0.3;
  rate_profiles[1].yaw_expo = 0.2;
  rate_profiles[1].rc_smoothing_factor = 0.3;
  rate_profiles[1].throttle_expo = 0.2;
  rate_profiles[1].throttle_mid = 0.5;
  
  // Profile 2: Acro/Race
  strcpy(rate_profiles[2].name, "Acro");
  rate_profiles[2].max_roll_rate = 800.0;
  rate_profiles[2].max_pitch_rate = 800.0;
  rate_profiles[2].max_yaw_rate = 720.0;
  rate_profiles[2].roll_expo = 0.5;
  rate_profiles[2].pitch_expo = 0.5;
  rate_profiles[2].yaw_expo = 0.4;
  rate_profiles[2].rc_smoothing_factor = 0.1;
  rate_profiles[2].throttle_expo = 0.3;
  rate_profiles[2].throttle_mid = 0.4;
}

// Function prototypes
void set_receiver_protocol(RcProtocol protocol);
bool read_rc_data(EnhancedRcData* data);
bool read_ppm_data(uint16_t* raw_channels);
bool read_sbus_data(uint16_t* raw_channels);
bool read_ibus_data(uint16_t* raw_channels);
void process_channel_mapping(uint16_t* raw_channels, EnhancedRcData* data);
float apply_expo_curve(float input, float expo);
float map_channel_value(ChannelMapping* mapping, uint16_t raw_value);
SwitchPosition get_switch_position(ChannelMapping* mapping, uint16_t raw_value);
void update_flight_mode_from_channels(EnhancedRcData* data);
void check_failsafe(EnhancedRcData* data);
bool validate_channel_assignment(uint8_t channel, ChannelFunction function);
void set_channel_mapping(uint8_t channel, ChannelFunction function, bool reversed);

void set_receiver_protocol(RcProtocol protocol) {
  current_protocol = protocol;
  rc_config.protocol = protocol;
  
  switch (protocol) {
    case RC_PROTOCOL_PPM:
      Serial.println("RC Protocol set to PPM");
      break;
    case RC_PROTOCOL_SBUS:
      Serial.println("RC Protocol set to SBUS");
      Serial2.begin(100000, SERIAL_8E2); // SBUS specific settings
      break;
    case RC_PROTOCOL_IBUS:
      Serial.println("RC Protocol set to iBUS");
      Serial2.begin(115200);
      break;
    case RC_PROTOCOL_ELRS:
      Serial.println("RC Protocol set to ELRS");
      Serial2.begin(420000); // ELRS baud rate
      break;
  }
}

bool read_rc_data(EnhancedRcData* data) {
  uint16_t raw_channels[16] = {0};
  bool success = false;
  
  switch (current_protocol) {
    case RC_PROTOCOL_PPM:
      success = read_ppm_data(raw_channels);
      break;
    case RC_PROTOCOL_SBUS:
      success = read_sbus_data(raw_channels);
      break;
    case RC_PROTOCOL_IBUS:
      success = read_ibus_data(raw_channels);
      break;
    case RC_PROTOCOL_ELRS:
      success = read_sbus_data(raw_channels); // ELRS uses SBUS protocol
      break;
    default:
      return false;
  }
  
  if (success) {
    // Copy raw channel data
    for (int i = 0; i < 16; i++) {
      data->raw_channels[i] = raw_channels[i];
    }
    data->channel_count = rc_config.channel_count;
    data->last_update = millis();
    
    // Process channel mapping
    process_channel_mapping(raw_channels, data);
    
    // Check for failsafe conditions
    check_failsafe(data);
    
    return true;
  }
  
  return false;
}

bool read_ppm_data(uint16_t* raw_channels) {
  // PPM implementation (simplified for example)
  // In real implementation, use interrupt-based PPM reading
  
  // Simulate PPM data
  raw_channels[0] = 1500; // Roll
  raw_channels[1] = 1500; // Pitch  
  raw_channels[2] = 1000; // Throttle
  raw_channels[3] = 1500; // Yaw
  raw_channels[4] = 1000; // Arm switch
  raw_channels[5] = 1500; // Mode switch
  raw_channels[6] = 1000; // Aux1
  raw_channels[7] = 1000; // Aux2
  
  return true;
}

bool read_sbus_data(uint16_t* raw_channels) {
  static uint8_t sbus_buffer[25];
  static int buffer_index = 0;
  
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    
    if (buffer_index == 0 && byte != 0x0F) {
      continue; // Wait for start byte
    }
    
    sbus_buffer[buffer_index] = byte;
    buffer_index++;
    
    if (buffer_index >= 25) {
      buffer_index = 0;
      
      // Check end byte
      if (sbus_buffer[24] != 0x00) {
        continue;
      }
      
      // Parse SBUS data
      uint16_t channels[16];
      
      channels[0]  = ((sbus_buffer[1]    | sbus_buffer[2]<<8)                 & 0x07FF);
      channels[1]  = ((sbus_buffer[2]>>3 | sbus_buffer[3]<<5)                 & 0x07FF);
      channels[2]  = ((sbus_buffer[3]>>6 | sbus_buffer[4]<<2 | sbus_buffer[5]<<10) & 0x07FF);
      channels[3]  = ((sbus_buffer[5]>>1 | sbus_buffer[6]<<7)                 & 0x07FF);
      channels[4]  = ((sbus_buffer[6]>>4 | sbus_buffer[7]<<4)                 & 0x07FF);
      channels[5]  = ((sbus_buffer[7]>>7 | sbus_buffer[8]<<1 | sbus_buffer[9]<<9)  & 0x07FF);
      channels[6]  = ((sbus_buffer[9]>>2 | sbus_buffer[10]<<6)                & 0x07FF);
      channels[7]  = ((sbus_buffer[10]>>5| sbus_buffer[11]<<3)                & 0x07FF);
      channels[8]  = ((sbus_buffer[11]>>8 | sbus_buffer[12]<<8)               & 0x07FF);
      channels[9]  = ((sbus_buffer[12]>>3 | sbus_buffer[13]<<5)               & 0x07FF);
      channels[10] = ((sbus_buffer[13]>>6 | sbus_buffer[14]<<2 | sbus_buffer[15]<<10) & 0x07FF);
      channels[11] = ((sbus_buffer[15]>>1 | sbus_buffer[16]<<7)               & 0x07FF);
      channels[12] = ((sbus_buffer[16]>>4 | sbus_buffer[17]<<4)               & 0x07FF);
      channels[13] = ((sbus_buffer[17]>>7 | sbus_buffer[18]<<1 | sbus_buffer[19]<<9)  & 0x07FF);
      channels[14] = ((sbus_buffer[19]>>2 | sbus_buffer[20]<<6)               & 0x07FF);
      channels[15] = ((sbus_buffer[20]>>5 | sbus_buffer[21]<<3)               & 0x07FF);
      
      // Convert SBUS values (172-1811) to standard PWM (1000-2000)
      for (int i = 0; i < 16; i++) {
        raw_channels[i] = map(channels[i], 172, 1811, 1000, 2000);
      }
      
      return true;
    }
  }
  
  return false;
}

bool read_ibus_data(uint16_t* raw_channels) {
  static uint8_t ibus_buffer[32];
  static int buffer_index = 0;
  
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    
    if (buffer_index == 0 && byte != 0x20) {
      continue; // Wait for iBUS header
    }
    
    if (buffer_index == 1 && byte != 0x40) {
      buffer_index = 0;
      continue; // Wait for correct second byte
    }
    
    ibus_buffer[buffer_index] = byte;
    buffer_index++;
    
    if (buffer_index >= 32) {
      buffer_index = 0;
      
      // Calculate checksum
      uint16_t checksum = 0xFFFF;
      for (int i = 0; i < 30; i++) {
        checksum -= ibus_buffer[i];
      }
      
      uint16_t received_checksum = ibus_buffer[30] | (ibus_buffer[31] << 8);
      
      if (checksum != received_checksum) {
        continue; // Invalid checksum
      }
      
      // Parse iBUS data
      for (int i = 0; i < 14; i++) {
        raw_channels[i] = ibus_buffer[2 + i*2] | (ibus_buffer[3 + i*2] << 8);
      }
      
      return true;
    }
  }
  
  return false;
}

void process_channel_mapping(uint16_t* raw_channels, EnhancedRcData* data) {
  RateProfile* current_profile = &rate_profiles[rc_config.current_rate_profile];
  
  // Initialize all values
  data->throttle = 0.0;
  data->roll = 0.0;
  data->pitch = 0.0;
  data->yaw = 0.0;
  data->armed = false;
  data->flight_mode = FLIGHT_MODE_STABILIZE;
  data->rth_activated = false;
  data->altitude_hold_enabled = false;
  data->position_hold_enabled = false;
  data->headless_mode = false;
  data->turtle_mode = false;
  data->beeper_activated = false;
  
  // Process each configured channel
  for (int i = 0; i < rc_config.channel_count; i++) {
    ChannelMapping* mapping = &rc_config.channels[i];
    
    if (mapping->channel_number == 0 || mapping->function == CHAN_FUNC_NONE) {
      continue;
    }
    
    uint16_t raw_value = raw_channels[mapping->channel_number - 1];
    
    // Check channel health
    data->channel_health[i] = (raw_value >= 900 && raw_value <= 2100);
    
    switch (mapping->function) {
      case CHAN_FUNC_THROTTLE: {
        float throttle_raw = map_channel_value(mapping, raw_value);
        data->throttle = apply_expo_curve(throttle_raw, current_profile->throttle_expo);
        // Apply throttle curve
        if (data->throttle < current_profile->throttle_mid) {
          data->throttle = data->throttle * current_profile->throttle_mid / current_profile->throttle_mid;
        } else {
          data->throttle = current_profile->throttle_mid + 
                          (data->throttle - current_profile->throttle_mid) * 
                          (1.0 - current_profile->throttle_mid) / (1.0 - current_profile->throttle_mid);
        }
        break;
      }
      
      case CHAN_FUNC_ROLL: {
        float roll_raw = map_channel_value(mapping, raw_value);
        data->roll = apply_expo_curve(roll_raw, current_profile->roll_expo);
        // Apply rate limiting
        data->roll *= current_profile->max_roll_rate / 500.0; // Normalize to ±1.0 at max rate
        break;
      }
      
      case CHAN_FUNC_PITCH: {
        float pitch_raw = map_channel_value(mapping, raw_value);
        data->pitch = apply_expo_curve(pitch_raw, current_profile->pitch_expo);
        data->pitch *= current_profile->max_pitch_rate / 500.0;
        break;
      }
      
      case CHAN_FUNC_YAW: {
        float yaw_raw = map_channel_value(mapping, raw_value);
        data->yaw = apply_expo_curve(yaw_raw, current_profile->yaw_expo);
        data->yaw *= current_profile->max_yaw_rate / 500.0;
        break;
      }
      
      case CHAN_FUNC_ARM_DISARM: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        data->armed = (pos == SWITCH_HIGH);
        break;
      }
      
      case CHAN_FUNC_FLIGHT_MODE: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        switch (pos) {
          case SWITCH_LOW:
            data->flight_mode = FLIGHT_MODE_MANUAL;
            break;
          case SWITCH_MID:
            data->flight_mode = FLIGHT_MODE_STABILIZE;
            break;
          case SWITCH_HIGH:
            data->flight_mode = FLIGHT_MODE_ALTITUDE_HOLD;
            break;
        }
        break;
      }
      
      case CHAN_FUNC_RTH: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        data->rth_activated = (pos == SWITCH_HIGH);
        break;
      }
      
      case CHAN_FUNC_ALTITUDE_HOLD: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        data->altitude_hold_enabled = (pos == SWITCH_HIGH);
        break;
      }
      
      case CHAN_FUNC_POSITION_HOLD: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        data->position_hold_enabled = (pos == SWITCH_HIGH);
        break;
      }
      
      case CHAN_FUNC_HEADLESS_MODE: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        data->headless_mode = (pos == SWITCH_HIGH);
        break;
      }
      
      case CHAN_FUNC_TURTLE_MODE: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        data->turtle_mode = (pos == SWITCH_HIGH);
        break;
      }
      
      case CHAN_FUNC_BEEPER: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        data->beeper_activated = (pos == SWITCH_HIGH);
        break;
      }
      
      case CHAN_FUNC_RATE_PROFILE: {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        switch (pos) {
          case SWITCH_LOW:
            rc_config.current_rate_profile = 0;
            break;
          case SWITCH_MID:
            rc_config.current_rate_profile = 1;
            break;
          case SWITCH_HIGH:
            rc_config.current_rate_profile = 2;
            break;
        }
        break;
      }
      
      default:
        // Handle other functions as needed
        break;
    }
  }
  
  data->signal_valid = true;
}

float apply_expo_curve(float input, float expo) {
  if (expo == 0.0) {
    return input;
  }
  
  float abs_input = abs(input);
  float result = (1.0 - expo) * input + expo * input * abs_input;
  return result;
}

float map_channel_value(ChannelMapping* mapping, uint16_t raw_value) {
  // Constrain to valid range
  raw_value = constrain(raw_value, mapping->min_value, mapping->max_value);
  
  float result;
  
  if (mapping->function == CHAN_FUNC_THROTTLE) {
    // Throttle: 0.0 to 1.0
    result = (float)(raw_value - mapping->min_value) / 
             (float)(mapping->max_value - mapping->min_value);
  } else {
    // Other controls: -1.0 to 1.0
    result = ((float)(raw_value - mapping->center_value) / 
              (float)(mapping->max_value - mapping->center_value));
    
    // Apply deadband
    if (abs(result) < (float)mapping->deadband / (float)(mapping->max_value - mapping->center_value)) {
      result = 0.0;
    }
  }
  
  // Apply reversal if needed
  if (mapping->reversed) {
    if (mapping->function == CHAN_FUNC_THROTTLE) {
      result = 1.0 - result;
    } else {
      result = -result;
    }
  }
  
  return result;
}

SwitchPosition get_switch_position(ChannelMapping* mapping, uint16_t raw_value) {
  if (raw_value < mapping->switch_thresholds[0]) {
    return SWITCH_LOW;
  } else if (mapping->switch_positions == 3 && raw_value < mapping->switch_thresholds[1]) {
    return SWITCH_MID;
  } else {
    return SWITCH_HIGH;
  }
}

void check_failsafe(EnhancedRcData* data) {
  unsigned long current_time = millis();
  
  // Check signal timeout
  if (current_time - data->last_update > RC_TIMEOUT_MS) {
    data->failsafe_active = true;
    data->signal_valid = false;
    
    if (rc_config.failsafe_enabled) {
      data->throttle = (float)rc_config.failsafe_throttle / 2000.0;
      data->roll = 0.0;
      data->pitch = 0.0;
      data->yaw = 0.0;
      data->flight_mode = rc_config.failsafe_mode;
      
      if (rc_config.rth_on_failsafe) {
        data->rth_activated = true;
      }
    }
  } else {
    data->failsafe_active = false;
  }
}

bool validate_channel_assignment(uint8_t channel, ChannelFunction function) {
  // Check if channel is already assigned to a critical function
  for (int i = 0; i < rc_config.channel_count; i++) {
    if (rc_config.channels[i].channel_number == channel && 
        rc_config.channels[i].function != function) {
      // Check if it's a critical function that can't be reassigned
      if (rc_config.channels[i].function == CHAN_FUNC_THROTTLE ||
          rc_config.channels[i].function == CHAN_FUNC_ROLL ||
          rc_config.channels[i].function == CHAN_FUNC_PITCH ||
          rc_config.channels[i].function == CHAN_FUNC_YAW) {
        return false; // Critical functions can't be reassigned
      }
    }
  }
  return true;
}

void set_channel_mapping(uint8_t channel, ChannelFunction function, bool reversed) {
  if (channel == 0 || channel > 16) {
    return; // Invalid channel
  }
  
  if (!validate_channel_assignment(channel, function)) {
    Serial.println("ERROR: Cannot assign channel - already used by critical function");
    return;
  }
  
  // Find the mapping slot for this function or create new one
  int mapping_index = -1;
  for (int i = 0; i < 16; i++) {
    if (rc_config.channels[i].function == function) {
      mapping_index = i;
      break;
    }
    if (rc_config.channels[i].function == CHAN_FUNC_NONE && mapping_index == -1) {
      mapping_index = i;
    }
  }
  
  if (mapping_index == -1) {
    Serial.println("ERROR: No available mapping slots");
    return;
  }
  
  // Configure the mapping
  rc_config.channels[mapping_index].channel_number = channel;
  rc_config.channels[mapping_index].function = function;
  rc_config.channels[mapping_index].reversed = reversed;
  
  // Set defaults based on function type
  if (function == CHAN_FUNC_ARM_DISARM || 
      function == CHAN_FUNC_FLIGHT_MODE ||
      function == CHAN_FUNC_RTH ||
      function == CHAN_FUNC_BEEPER) {
    rc_config.channels[mapping_index].is_switch = true;
    rc_config.channels[mapping_index].switch_positions = (function == CHAN_FUNC_FLIGHT_MODE) ? 3 : 2;
  }
  
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" mapped to function ");
  Serial.println(function);
}

#endif // RECEIVERS_H
