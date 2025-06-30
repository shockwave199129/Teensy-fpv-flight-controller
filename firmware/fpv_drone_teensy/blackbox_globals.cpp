#include "blackbox_logger.h"
#include "config.h"

// Global instances referenced across modules
BlackboxLogger blackbox;

// Provide weak default instances; main program can override
SensorData sensor_data_global __attribute__((weak));
RcData rc_data_global __attribute__((weak));

bool drone_is_armed __attribute__((weak)) = false; 