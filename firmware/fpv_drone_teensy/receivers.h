#ifndef RECEIVERS_H
#define RECEIVERS_H

#include "config.h"

// Global variables (extern declarations)
extern RcProtocol current_protocol;
extern RcConfig rc_config;
extern EnhancedRcData enhanced_rc_data;
extern RateProfile rate_profiles[3];
extern FunctionSwitches function_switches;

// Function prototypes
void init_default_channel_mapping();
void init_default_rate_profiles();
void set_receiver_protocol(RcProtocol protocol);
bool read_rc_data(EnhancedRcData* data);
bool read_ppm_data(uint16_t* raw_channels);
bool read_sbus_data(uint16_t* raw_channels);
bool read_ibus_data(uint16_t* raw_channels);
bool read_crsf_data(uint16_t* raw_channels);
void process_channel_mapping(uint16_t* raw_channels, EnhancedRcData* data);
float apply_expo_curve(float input, float expo);
float map_channel_value(ChannelMapping* mapping, uint16_t raw_value);
SwitchPosition get_switch_position(ChannelMapping* mapping, uint16_t raw_value);
void check_failsafe(EnhancedRcData* data);
bool validate_channel_assignment(uint8_t channel, ChannelFunction function);
void update_flight_mode_from_channels(EnhancedRcData* data);
void set_channel_mapping(uint8_t channel, ChannelFunction function, bool reversed);

#endif // RECEIVERS_H
