#include <unity.h>
#include "safety_functions.h"

void setUp(void) {}
void tearDown(void) {}

void test_dummy(void) {
  TEST_ASSERT_TRUE(true);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_dummy);
  return UNITY_END();
}
