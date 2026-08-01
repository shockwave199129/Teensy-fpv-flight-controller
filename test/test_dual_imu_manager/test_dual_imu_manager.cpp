#include <unity.h>
#include "dual_imu_manager.h"
#include "sensors.h"

void setUp(void) {}
void tearDown(void) {}

void test_dummy(void) {
  // Basic sanity check – always true
  TEST_ASSERT_TRUE(true);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_dummy);
  return UNITY_END();
}
