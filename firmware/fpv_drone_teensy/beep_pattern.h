#pragma once

#include <Arduino.h>

/**
 * @brief Helper for non‑blocking beep patterns on the Teensy.
 *
 * The class provides a static scheduler that can be fed a sequence of tones
 * (frequency & duration) together with inter‑tone gaps.  `playPattern` stores the
 * sequence and starts the first tone immediately.  `update` must be called from
 * the main loop (or any regular processing loop) to advance the pattern and to
 * stop the tone when its duration expires.
 *
 * Example usage:
 * ```cpp
 * const uint16_t freqs[] = {2000, 0, 2000, 0};
 * const uint16_t dur[]   = {100, 0, 100, 0};
 * const uint16_t gaps[]  = {100, 50, 100, 0};
 * BeepPattern::playPattern(freqs, dur, gaps, 2);
 * ```
 * The `update` call is typically placed at the end of the main control loop.
 */
class BeepPattern {
public:
  /**
   * @brief Schedule a beep pattern.
   *
   * @param freqs   Array of frequencies (Hz). A frequency of 0 means silence
   *                (used for gaps). The array length must be >= `count * 2` –
   *                each repeat consists of a tone and the following gap.
   * @param durations Array of tone durations (ms). Elements corresponding to a
   *                silence entry should be 0.
   * @param gaps    Array of gap durations (ms) after each tone.
   * @param count   Number of repetitions (i.e., number of tones).
   */
  static void playPattern(const uint16_t *freqs,
                          const uint16_t *durations,
                          const uint16_t *gaps,
                          size_t count);

  /**
   * @brief Call from the main loop to progress the scheduled pattern.
   */
  static void update();

private:
  static const uint8_t MAX_STEPS = 20; // safety limit
  static uint16_t _freqs[MAX_STEPS];
  static uint16_t _durations[MAX_STEPS];
  static uint16_t _gaps[MAX_STEPS];
  static uint8_t _stepCount;
  static uint8_t _currentStep;
  static unsigned long _stepStart;
  static bool _active;
};
