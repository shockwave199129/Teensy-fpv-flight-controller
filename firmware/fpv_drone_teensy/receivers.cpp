#include "receivers.h"
#include <Arduino.h>
#include <string.h>
#include "hal_timer_pwm.h" // (not needed but include for consistency)

// Global variable definitions
RcProtocol current_protocol = RC_PROTOCOL_PPM;
RcConfig rc_config;
EnhancedRcData enhanced_rc_data;
RateProfile rate_profiles[3];
FunctionSwitches function_switches;

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

// ------------------- NEW ISR-BASED RC PARSER -------------------
// We switch from polling Serial2 to interrupt-driven parsing so the control
// loop no longer spends precious time in serial byte handling.  Teensy's
// HardwareSerial already fires an IRQ whenever a new byte arrives; we attach a
// handler that decodes SBUS / iBUS / CRSF frames on the fly.

static volatile uint16_t isr_raw_channels[16];
static volatile bool     isr_frame_ready = false;
static volatile uint8_t  isr_channel_count = 0;
static volatile uint8_t  isr_failsafe_flags = 0; // protocol-specific (SBUS bits etc.)
static unsigned long     isr_last_frame_time = 0;

// Forward declarations
static void rc_serial_isr();
static void process_sbus_byte(uint8_t b);
static void process_ibus_byte(uint8_t b);
static void process_crsf_byte(uint8_t b);

// -------------------  Serial interrupt registration  -------------------
void init_receiver_isr(RcProtocol proto)
{
  switch(proto) {
    case RC_PROTOCOL_SBUS:
      Serial2.begin(100000); // 100 kBd inverted
      break;
    case RC_PROTOCOL_IBUS:
      Serial2.begin(115200);
      break;
    case RC_PROTOCOL_ELRS:
      Serial2.begin(420000);
      break;
    default:
      return;
  }
  // Teensy 4.x HardwareSerial does not expose attachInterrupt().
  // Instead we rely on Arduino's serialEvent2() mechanism which
  // is invoked from yield() whenever new bytes are available.
  // The static rc_serial_isr() will be called from that hook.
}

static void rc_serial_isr()
{
  while(Serial2.available()) {
    uint8_t b = Serial2.read();
    switch(current_protocol) {
      case RC_PROTOCOL_SBUS: process_sbus_byte(b); break;
      case RC_PROTOCOL_IBUS: process_ibus_byte(b); break;
      case RC_PROTOCOL_ELRS: process_crsf_byte(b); break;
      default: /* ignore */ break;
    }
  }
}

// Arduino/Teensy runtime calls this whenever Serial2 has received data.
void serialEvent2() {
  rc_serial_isr();
}

// -------------------  SBUS byte-wise decoder  -------------------
static void process_sbus_byte(uint8_t byte)
{
  constexpr uint8_t FRAME_SIZE = 25;
  static uint8_t sbusFrame[FRAME_SIZE];
  static uint8_t pos = 0;

  if (pos == 0) {
    if (byte != 0x0F) return; // wait for start byte
  }
  sbusFrame[pos++] = byte;
  if (pos == FRAME_SIZE) {
    pos = 0;
    if (sbusFrame[24] != 0x00) return; // invalid end byte

    uint32_t buf = 0; uint8_t bufBits = 0; uint8_t chIdx = 0;
    for(int i=1;i<23;i++) {
      buf |= ((uint32_t)sbusFrame[i]) << bufBits;
      bufBits += 8;
      while (bufBits >= 11 && chIdx < 16) {
        isr_raw_channels[chIdx++] = (buf & 0x7FF) + 880;
        buf >>= 11; bufBits -= 11;
      }
    }
    isr_channel_count = chIdx;
    isr_failsafe_flags = sbusFrame[23];
    isr_frame_ready = true;
    isr_last_frame_time = millis();
  }
}

// -------------------  iBUS decoder  -------------------
static void process_ibus_byte(uint8_t byte)
{
  constexpr uint8_t FRAME_SIZE = 32;
  static uint8_t ibusFrame[FRAME_SIZE];
  static uint8_t pos = 0;
  static uint32_t lastByteTime = 0;

  uint32_t now = millis();
  if (now - lastByteTime > 10) pos = 0;
  lastByteTime = now;

  if (pos == 0 && byte != 0x20) return; // header byte
  ibusFrame[pos++] = byte;
  if (pos == FRAME_SIZE) {
    // checksum
    uint16_t checksum = 0xFFFF;
    for(int i=0;i<FRAME_SIZE-2;i++) checksum -= ibusFrame[i];
    uint16_t rxChecksum = ibusFrame[FRAME_SIZE-2] | (ibusFrame[FRAME_SIZE-1]<<8);
    if (checksum == rxChecksum) {
      for(int ch=0; ch<14; ch++) {
        isr_raw_channels[ch] = ibusFrame[2+ch*2] | (ibusFrame[3+ch*2]<<8);
      }
      isr_channel_count = 14;
      isr_frame_ready = true;
      isr_last_frame_time = now;
    }
    pos = 0;
  }
}

// -------------------  CRSF byte-wise decoder (ELRS)  -------------------
static void process_crsf_byte(uint8_t byte)
{
  static uint8_t frame[64];
  static uint8_t idx = 0;
  static uint8_t frameLen = 0;

  if (idx == 0 && byte != 0xC8) return; // sync byte
  frame[idx++] = byte;
  if (idx == 2) {
    frameLen = byte;
    if (frameLen > sizeof(frame)) { idx = 0; return; }
  }
  if (idx > 2 && idx == frameLen + 2) {
    // We have full frame (addr+len+payload+crc)
    uint8_t type = frame[2];
    if (type == 0x16) { // RC channels packed
      const uint8_t* d = &frame[3];
      uint32_t buf = 0; uint8_t bits=0; uint8_t ch=0;
      while (ch < 16) {
        while(bits < 11) { buf |= ((uint32_t)(*d++)) << bits; bits += 8; }
        isr_raw_channels[ch++] = (buf & 0x7FF) + 880; buf >>= 11; bits -= 11;
      }
      isr_channel_count = 16;
      isr_frame_ready = true;
      isr_last_frame_time = millis();
    }
    idx = 0;
  }
}

// -------------------  Updated protocol init  -------------------
// Modify set_receiver_protocol to start ISR based parser instead of polling
void set_receiver_protocol(RcProtocol protocol) {
  current_protocol = protocol;
  rc_config.protocol = protocol;
  
  switch (protocol) {
    case RC_PROTOCOL_PPM:
      Serial.println("RC Protocol set to PPM");
      break;
    case RC_PROTOCOL_SBUS:
      Serial.println("RC Protocol set to SBUS (ISR)");
      init_receiver_isr(RC_PROTOCOL_SBUS);
      break;
    case RC_PROTOCOL_IBUS:
      Serial.println("RC Protocol set to iBUS (ISR)");
      init_receiver_isr(RC_PROTOCOL_IBUS);
      break;
    case RC_PROTOCOL_ELRS:
      Serial.println("RC Protocol set to ELRS (ISR)");
      init_receiver_isr(RC_PROTOCOL_ELRS);
      break;
  }
}

bool read_rc_data(EnhancedRcData* data) {
  uint16_t raw_channels[16] = {0};
  bool success = false;
  data->failsafe = false;
  
  if (current_protocol == RC_PROTOCOL_PPM) {
    // Legacy PPM remains polling using pulseIn or GPIO capture elsewhere
    success = read_ppm_data(raw_channels);
  } else {
    // Interrupt-driven protocols: check if a new frame is available
    noInterrupts();
    if (isr_frame_ready) {
      for(uint8_t i=0;i<isr_channel_count;i++) raw_channels[i]=isr_raw_channels[i];
      isr_frame_ready = false;
      success = true;
    }
    interrupts();
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
    
    // ---------- Channel-count & protocol validity checks ----------
    data->signal_valid = true;

    if (current_protocol == RC_PROTOCOL_PPM) {
        // Assume PPM always provides correct channel count configured earlier
    } else {
        if (isr_channel_count < rc_config.channel_count) {
            data->signal_valid = false; // not enough channels received
        }

        // SBUS specific flags (failsafe / frame lost)
        if (current_protocol == RC_PROTOCOL_SBUS) {
            bool frameLost  = isr_failsafe_flags & 0x04;
            bool failsafeF  = isr_failsafe_flags & 0x08;
            if (frameLost)  data->signal_valid = false;
            if (failsafeF || frameLost) data->failsafe = true;
        }
    }

    // ----------- Check for failsafe/timeouts etc. ---------------
    check_failsafe(data);
    
    // Mirror to legacy field
    data->failsafe_active = data->failsafe;
    
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

// Legacy polling implementations are now obsolete because parsing is done in
// the ISR.  We keep stub functions so other compilation units still link.

bool read_sbus_data(uint16_t* raw_channels) {
  if (current_protocol != RC_PROTOCOL_SBUS) return false;
  bool success = false;
  noInterrupts();
  if (isr_frame_ready) {
    for (uint8_t i = 0; i < isr_channel_count; i++) raw_channels[i] = isr_raw_channels[i];
    isr_frame_ready = false; // mark consumed
    success = true;
  }
  interrupts();
  return success;
}

bool read_ibus_data(uint16_t* raw_channels) {
  if (current_protocol != RC_PROTOCOL_IBUS) return false;
  bool success = false;
  noInterrupts();
  if (isr_frame_ready) {
    for (uint8_t i = 0; i < isr_channel_count; i++) raw_channels[i] = isr_raw_channels[i];
    isr_frame_ready = false;
    success = true;
  }
  interrupts();
  return success;
}

bool read_crsf_data(uint16_t* raw_channels) {
  if (current_protocol != RC_PROTOCOL_ELRS) return false; // CRSF = ELRS in this firmware
  bool success = false;
  noInterrupts();
  if (isr_frame_ready) {
    for (uint8_t i = 0; i < isr_channel_count; i++) raw_channels[i] = isr_raw_channels[i];
    isr_frame_ready = false;
    success = true;
  }
  interrupts();
  return success;
}

void process_channel_mapping(uint16_t* raw_channels, EnhancedRcData* data) {
  // Reset processed values
  data->roll = 0.0;
  data->pitch = 0.0;
  data->yaw = 0.0;
  data->throttle = 0.0;
  
  for (int i = 0; i < rc_config.channel_count; i++) {
    ChannelMapping* mapping = &rc_config.channels[i];
    if (mapping->channel_number > 0) {
      uint16_t raw_value = raw_channels[mapping->channel_number - 1];
      
      if (mapping->is_switch) {
        SwitchPosition pos = get_switch_position(mapping, raw_value);
        switch (mapping->function) {
          case CHAN_FUNC_ARM_DISARM:
            function_switches.arm_disarm = pos;
            break;
          case CHAN_FUNC_FLIGHT_MODE:
            function_switches.flight_mode = pos;
            break;
          case CHAN_FUNC_BEEPER:
            function_switches.beeper = pos;
            break;
          case CHAN_FUNC_RTH:
            function_switches.rth = pos;
            break;
          default:
            break;
        }
      } else {
        float mapped_value = map_channel_value(mapping, raw_value);
        
        switch (mapping->function) {
          case CHAN_FUNC_ROLL:
            data->roll = mapped_value;
            break;
          case CHAN_FUNC_PITCH:
            data->pitch = mapped_value;
            break;
          case CHAN_FUNC_YAW:
            data->yaw = mapped_value;
            break;
          case CHAN_FUNC_THROTTLE:
            data->throttle = mapped_value;
            break;
          default:
            break;
        }
      }
    }
  }
  
  // Apply rates and expo from the current profile
  RateProfile* profile = &rate_profiles[rc_config.current_rate_profile];
  data->roll = apply_expo_curve(data->roll, profile->roll_expo) * profile->max_roll_rate;
  data->pitch = apply_expo_curve(data->pitch, profile->pitch_expo) * profile->max_pitch_rate;
  data->yaw = apply_expo_curve(data->yaw, profile->yaw_expo) * profile->max_yaw_rate;
  data->throttle = apply_expo_curve(data->throttle, profile->throttle_expo);
}

float apply_expo_curve(float input, float expo) {
  if (expo == 0) return input;
  return (1 - expo) * input + expo * input * input * input;
}

float map_channel_value(ChannelMapping* mapping, uint16_t raw_value) {
  float value = (float)raw_value;
  
  // Apply reversal if needed
  if (mapping->reversed) {
    value = (float)(mapping->max_value + mapping->min_value) - value;
  }
  
  // Normalize to -1.0 to 1.0 or 0.0 to 1.0
  if (mapping->function == CHAN_FUNC_THROTTLE) {
    return (value - mapping->min_value) / (mapping->max_value - mapping->min_value);
  } else {
    float centered = value - mapping->center_value;
    if (abs(centered) < mapping->deadband) return 0.0;
    
    float range_up = mapping->max_value - mapping->center_value;
    float range_down = mapping->center_value - mapping->min_value;
    
    if (centered > 0) {
      return centered / range_up;
    } else {
      return centered / range_down;
    }
  }
}

SwitchPosition get_switch_position(ChannelMapping* mapping, uint16_t raw_value) {
  if (mapping->switch_positions == 2) {
    return (raw_value > mapping->switch_thresholds[0]) ? SWITCH_POS_HIGH : SWITCH_POS_LOW;
  } else if (mapping->switch_positions == 3) {
    if (raw_value < mapping->switch_thresholds[0]) return SWITCH_POS_LOW;
    if (raw_value > mapping->switch_thresholds[1]) return SWITCH_POS_HIGH;
    return SWITCH_POS_MIDDLE;
  }
  return SWITCH_POS_LOW;
}

void check_failsafe(EnhancedRcData* data) {
  // preserve any prior protocol-level failsafe flag state
  if (!rc_config.failsafe_enabled) return;
  
  // Basic failsafe: if no update for a while
  if (millis() - data->last_update > 500) { // 500ms timeout
    data->failsafe = true;
  }

  // Invalid signal flag from read_rc_data
  if (!data->signal_valid) {
    data->failsafe = true;
  }

  // Channel-count mismatch check for ISR-based protocols
  if (current_protocol != RC_PROTOCOL_PPM && isr_channel_count < rc_config.channel_count) {
    data->failsafe = true;
  }
  
  if (data->failsafe) {
    data->failsafe_active = true;
    data->throttle = (float)(rc_config.failsafe_throttle - 1000) / 1000.0;
    data->roll = 0.0;
    data->pitch = 0.0;
    data->yaw = 0.0;
    // Set current flight mode to failsafe mode
  }
}

bool validate_channel_assignment(uint8_t channel, ChannelFunction function) {
  if (channel < 1 || channel > 16) return false;
  
  // Check if this function is already assigned to another channel
  for (int i = 0; i < 16; i++) {
    if (rc_config.channels[i].function == function && rc_config.channels[i].channel_number != channel) {
      return false; // Already assigned
    }
  }
  
  // Check if this channel is already used for another function
  for (int i = 0; i < 16; i++) {
    if (rc_config.channels[i].channel_number == channel && rc_config.channels[i].function != function) {
      return false; // Channel already in use
    }
  }
  
  return true;
}

void update_flight_mode_from_channels(EnhancedRcData* data) {
    // Example logic using a 3-position switch for flight modes
    // This is highly dependent on the user's RC transmitter setup
    
    switch (function_switches.flight_mode) {
        case SWITCH_POS_LOW:
            // current_flight_mode = FLIGHT_MODE_STABILIZE;
            break;
        case SWITCH_POS_MIDDLE:
            // current_flight_mode = FLIGHT_MODE_HORIZON;
            break;
        case SWITCH_POS_HIGH:
            // current_flight_mode = FLIGHT_MODE_ACRO;
            break;
    }

    // Handle RTH switch
    if (function_switches.rth == SWITCH_POS_HIGH) {
        // current_flight_mode = FLIGHT_MODE_RTH;
    }

    // Arming logic
    bool is_armed = false;
    if (rc_config.switch_arming_enabled) {
        if (function_switches.arm_disarm == SWITCH_POS_HIGH) {
            is_armed = true;
        }
    }

    if (rc_config.stick_arming_enabled) {
        // Stick arming logic (e.g., throttle down, yaw right)
        if (data->throttle < 0.05 && data->yaw > 0.9) {
            // Add a timer to prevent accidental arming
            static unsigned long arm_stick_timer = 0;
            if (arm_stick_timer == 0) {
                arm_stick_timer = millis();
            } else if (millis() - arm_stick_timer > rc_config.arm_sequence_time_ms) {
                is_armed = true;
            }
        } else {
            // arm_stick_timer = 0;
        }
    }

    // Update global armed state visible to safety functions/blackbox
    extern bool drone_is_armed;
    drone_is_armed = is_armed;
}


void set_channel_mapping(uint8_t channel, ChannelFunction function, bool reversed) {
  if (!validate_channel_assignment(channel, function)) {
    Serial.println("Invalid channel assignment.");
    return;
  }
  
  // Find the channel mapping to update
  for (int i = 0; i < 16; i++) {
    if (rc_config.channels[i].channel_number == channel) {
      rc_config.channels[i].function = function;
      rc_config.channels[i].reversed = reversed;
      
      // If it's a primary flight control, ensure it's not set as a switch
      if (function == CHAN_FUNC_ROLL || function == CHAN_FUNC_PITCH ||
          function == CHAN_FUNC_YAW || function == CHAN_FUNC_THROTTLE) {
        rc_config.channels[i].is_switch = false;
      }
      
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.print(" mapped to function ");
      Serial.println(function);
      return;
    }
  }
}