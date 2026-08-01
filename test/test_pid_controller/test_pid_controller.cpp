// Real coverage for PIDController (replaces the old TEST_ASSERT_TRUE(true)
// placeholder). Exercises the two bugs from FIX_PLAN.md F2:
//   1. SET PID must only touch the target axis's own term, not bleed in
//      rate_roll's other terms.
//   2. A tuned gain must survive the next adaptive-gain recompute, since
//      that recompute scales from base_gains, not cascaded_gains.
#include <unity.h>
#include "config.h"

// pid_controller.h's update_angle_loop() reaches for a global
// `extern SensorRedundancySystem sensor_redundancy;` and calls
// get_adaptive_tuning() on it, so SensorRedundancySystem must already be a
// complete type wherever pid_controller.h is included (true of every real
// .cpp/.ino in this firmware, which all include sensor_redundancy.h first).
// The real class pulls in Wire/SPI/String plumbing unrelated to what's under
// test here, so provide a minimal double with just the interface
// pid_controller.h actually calls.
class SensorRedundancySystem {
public:
  FlightTuning get_adaptive_tuning() {
    FlightTuning t;
    memset(&t, 0, sizeof(t));
    t.stability_gain_multiplier = 1.0f;
    return t;
  }
};
SensorRedundancySystem sensor_redundancy;

#include "pid_controller.h"

// apply_harmonic_compensation() (compiled unconditionally as part of the
// PIDController class body, even though it early-returns unless
// harmonic_compensation_enabled) reaches for a global
// `extern MotorControl motor_control;` - needs a real definition to link.
MotorControl motor_control;

static PIDController* pid = nullptr;

void setUp(void) {
  test_set_millis(0);
  pid = new PIDController();
  pid->init();
}

void tearDown(void) {
  delete pid;
  pid = nullptr;
}

void test_set_rate_pid_term_only_changes_target_term(void) {
  CascadedPIDGains before = pid->get_cascaded_gains();

  pid->set_rate_pid_term(1 /* PITCH */, 'I', 0.5f);

  CascadedPIDGains after = pid->get_cascaded_gains();

  // The requested term changed...
  TEST_ASSERT_EQUAL_FLOAT(0.5f, after.rate_pitch.ki);
  // ...but pitch's own P and D must be untouched.
  TEST_ASSERT_EQUAL_FLOAT(before.rate_pitch.kp, after.rate_pitch.kp);
  TEST_ASSERT_EQUAL_FLOAT(before.rate_pitch.kd, after.rate_pitch.kd);
  // Roll and yaw must be completely untouched (the original bug always
  // sourced the "other two terms" from rate_roll, corrupting it).
  TEST_ASSERT_EQUAL_FLOAT(before.rate_roll.kp, after.rate_roll.kp);
  TEST_ASSERT_EQUAL_FLOAT(before.rate_roll.ki, after.rate_roll.ki);
  TEST_ASSERT_EQUAL_FLOAT(before.rate_roll.kd, after.rate_roll.kd);
  TEST_ASSERT_EQUAL_FLOAT(before.rate_yaw.kp, after.rate_yaw.kp);
  TEST_ASSERT_EQUAL_FLOAT(before.rate_yaw.ki, after.rate_yaw.ki);
  TEST_ASSERT_EQUAL_FLOAT(before.rate_yaw.kd, after.rate_yaw.kd);
}

void test_set_rate_pid_term_rejects_invalid_axis_and_term(void) {
  CascadedPIDGains before = pid->get_cascaded_gains();

  pid->set_rate_pid_term(99, 'P', 7.0f);   // invalid axis
  pid->set_rate_pid_term(0, 'X', 7.0f);    // invalid term

  CascadedPIDGains after = pid->get_cascaded_gains();
  TEST_ASSERT_EQUAL_FLOAT(before.rate_roll.kp, after.rate_roll.kp);
  TEST_ASSERT_EQUAL_FLOAT(before.rate_pitch.kp, after.rate_pitch.kp);
}

// Drives one PID update cycle so update_adaptive_gains() (which recomputes
// cascaded_gains.rate_*.kp/ki from base_gains every cycle) actually runs.
static void run_one_update_cycle(PIDController& controller) {
  SensorData sensor_data;
  memset(&sensor_data, 0, sizeof(sensor_data));

  RcData rc_data;
  memset(&rc_data, 0, sizeof(rc_data));
  rc_data.throttle = 1500;
  rc_data.roll = 1500;
  rc_data.pitch = 1500;
  rc_data.yaw = 1500;
  rc_data.signal_valid = true;

  FlightState flight_state;
  memset(&flight_state, 0, sizeof(flight_state));
  flight_state.flight_mode = FLIGHT_MODE_MANUAL;

  test_set_millis(1);
  controller.update(sensor_data, rc_data, flight_state);
}

void test_tuned_rate_gain_survives_adaptive_recompute(void) {
  pid->set_rate_pid_term(1 /* PITCH */, 'P', 3.3f);
  TEST_ASSERT_EQUAL_FLOAT(3.3f, pid->get_cascaded_gains().rate_pitch.kp);

  // update_adaptive_gains() (invoked every update() cycle) recomputes
  // cascaded_gains.rate_*.kp by scaling base_gains, not cascaded_gains, by a
  // flight-condition-derived multiplier - so cascaded_gains itself is
  // expected to move. What must NOT happen is base_gains reverting to the
  // factory default: before the F2 fix, set_rate_pid_gains()/
  // set_rate_pid_term() never touched base_gains, so every future adaptive
  // recompute silently scaled from the old default instead of the tuned
  // value, permanently discarding the CLI change.
  run_one_update_cycle(*pid);
  run_one_update_cycle(*pid);

  TEST_ASSERT_EQUAL_FLOAT(3.3f, pid->get_base_gains().rate_pitch.kp);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_set_rate_pid_term_only_changes_target_term);
  RUN_TEST(test_set_rate_pid_term_rejects_invalid_axis_and_term);
  RUN_TEST(test_tuned_rate_gain_survives_adaptive_recompute);
  return UNITY_END();
}
