#include "hal_timer_pwm.h"

#if !defined(__IMXRT1062__) && !defined(CORE_TEENSY)
#error "hal_timer_pwm.cpp targets Teensy 4.x (IMXRT1062). Other MCUs are not supported."
#endif

// The current implementation is header-only.  This translation unit is
// provided so that future, more sophisticated MCU-specific code can live here
// without changing build scripts.  Right now it merely compiles the header.