#include "beep_pattern.h"
#include <Arduino.h>
#include "beep_config.h"
#ifndef BEEPER_PIN
#define BEEPER_PIN 8 // Default beeper pin (should match safety_functions.cpp)
#endif

// Static member definitions
uint16_t BeepPattern::_freqs[BeepPattern::MAX_STEPS] = {0};
uint16_t BeepPattern::_durations[BeepPattern::MAX_STEPS] = {0};
uint16_t BeepPattern::_gaps[BeepPattern::MAX_STEPS] = {0};
uint8_t BeepPattern::_stepCount = 0;
uint8_t BeepPattern::_currentStep = 0;
unsigned long BeepPattern::_stepStart = 0;
bool BeepPattern::_active = false;
bool enable_beep_feedback = true;

void BeepPattern::playPattern(const uint16_t *freqs,
                              const uint16_t *durations,
                              const uint16_t *gaps,
                              size_t count) {
  // If beeps are disabled, do nothing
  if (!enable_beep_feedback) { _active = false; return; }
  // Clamp to safe maximum
  _stepCount = (count * 2 > MAX_STEPS) ? MAX_STEPS / 2 : count;
  for (uint8_t i = 0; i < _stepCount * 2; ++i) {
    _freqs[i] = freqs[i];
    _durations[i] = durations[i];
    _gaps[i] = gaps[i];
  }
  _currentStep = 0;
  _active = true;
  // Start first tone if frequency > 0
  if (_freqs[0] > 0) {
    tone(BEEPER_PIN, _freqs[0], _durations[0]);
  }
  _stepStart = millis();
}

void BeepPattern::update() {
  if (!_active) return;
  unsigned long now = millis();
  // Determine elapsed time for current tone
  if (now - _stepStart >= _durations[_currentStep]) {
    // Tone finished – stop it
    noTone(BEEPER_PIN);
    // Apply gap before next step
    unsigned long gap = _gaps[_currentStep];
    if (gap > 0) {
      if (now - _stepStart < _durations[_currentStep] + gap) {
        // still in gap, wait
        return;
      }
    }
    // Advance to next step
    ++_currentStep;
    if (_currentStep >= _stepCount * 2) {
      // Pattern completed
      _active = false;
      return;
    }
    // Start next tone if not silence
    if (_freqs[_currentStep] > 0) {
      tone(BEEPER_PIN, _freqs[_currentStep], _durations[_currentStep]);
    }
    _stepStart = now;
  }
}
