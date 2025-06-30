#ifndef HAL_TIMER_PWM_H
#define HAL_TIMER_PWM_H

#include <Arduino.h>
#include "config.h"

// Low-level timer / PWM abstraction layer so that upper modules (ESC driver,
// motor control, etc.) no longer touch MCU registers directly.  For the first
// iteration we expose only the pieces currently needed by the existing source
// code: 1) A nanosecond delay helper for bit-banged protocols and 2) helper
// functions that translate EscProtocol into FlexPWM MOD / duty-cycle tick
// counts for DShot output.  The API is intentionally minimal and can be
// extended later for other timer usages (input-capture, SBUS DMA triggers…).

namespace HalTimerPWM {

// ---------------------------------------------------------------------------
//  Utility: coarse nanosecond delay (busy-wait)
// ---------------------------------------------------------------------------
//  On Teensy 4.x we do not have a cycle-accurate NOP loop in Arduino, but the
//  core provides delayNanoseconds().  Unfortunately that function is not part
//  of every Arduino core so we roll our own fallback that degrades gracefully
//  to delayMicroseconds() when the requested time exceeds ~1 µs.
// ---------------------------------------------------------------------------
inline void delayNS(uint32_t ns)
{
#if defined(delayNanoseconds)
    delayNanoseconds(ns);
#else
    // For very small delays the loop below is still >100 ns so we bound at 1 µs.
    if (ns <= 1000) {
        // minimum achievable resolution with delayMicroseconds()
        delayMicroseconds(1);
    } else {
        delayMicroseconds(static_cast<unsigned int>(ns / 1000));
    }
#endif
}

// ---------------------------------------------------------------------------
//  Helper: compute FlexPWM MOD value (tick count per bit) for a given DShot
//  protocol.  We *approximate* the timer clock so that MOD ends up in a
//  sensible range (tens of ticks).  For precise timing the implementation will
//  be refined later once we finalise the HAL clock tree configuration.
// ---------------------------------------------------------------------------
inline uint16_t dshot_mod_ticks(EscProtocol proto)
{
    // Assume FlexPWM clock = 150 MHz and each tick = 10 ns -> MOD = bitTime / 10.
    switch (proto) {
        case ESC_PROTOCOL_DSHOT150:  return 6667 / 10;  // 150 kHz (≈67 ticks)
        case ESC_PROTOCOL_DSHOT300:  return 3333 / 10;  // 300 kHz (≈33 ticks)
        case ESC_PROTOCOL_DSHOT600:  return 1667 / 10;  // 600 kHz (≈17 ticks)
        case ESC_PROTOCOL_DSHOT1200: return  833 / 10;  // 1.2 MHz (≈8 ticks)
        default:                    return 1667 / 10;  // default to 600 kHz
    }
}

// Duty-cycle helper for logical 0 / 1.
inline uint16_t dshot_hi_ticks0(EscProtocol proto) { return dshot_mod_ticks(proto) * 37 / 100; }
inline uint16_t dshot_hi_ticks1(EscProtocol proto) { return dshot_mod_ticks(proto) * 74 / 100; }

} // namespace HalTimerPWM

#endif // HAL_TIMER_PWM_H 