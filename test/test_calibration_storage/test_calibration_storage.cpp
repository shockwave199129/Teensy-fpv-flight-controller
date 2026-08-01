// Real coverage for calibration_storage.cpp (replaces the old
// TEST_ASSERT_TRUE(true) placeholder). Exercises the exact bug from
// FIX_PLAN.md F1: calibration data must actually survive a reboot, i.e. a
// save() followed by a fresh load() into a zeroed struct must recover the
// same values - and a corrupted/foreign blob must be rejected, not misread.
#include <unity.h>
#include "config.h"
#include "calibration_storage.h"

// Pull in the real implementation under test (native build has no separate
// compilation step wired up for it, so we compile it by inclusion).
#include "../../firmware/fpv_drone_teensy/calibration_storage.cpp"

void setUp(void) { EEPROM.test_clear(); }
void tearDown(void) {}

static CalibrationData make_sample_calibration() {
  CalibrationData d;
  memset(&d, 0, sizeof(d));
  d.gyro_calibrated = true;
  d.gyro_offsets[0] = 1.5f;
  d.gyro_offsets[1] = -2.25f;
  d.gyro_offsets[2] = 0.125f;
  d.accel_calibrated = true;
  d.accel_offsets[0] = 0.01f;
  d.accel_offsets[1] = 0.02f;
  d.accel_offsets[2] = 9.81f;
  d.mag_calibrated = true;
  d.mag_declination = 4.5f;
  d.system_calibrated = true;
  d.calibration_timestamp = 12345;
  return d;
}

void test_calibration_survives_round_trip(void) {
  CalibrationData original = make_sample_calibration();
  TEST_ASSERT_TRUE(CalibrationStorage::save(original));

  // Simulate a reboot: a fresh, zero-initialized struct with no memory of
  // the values above.
  CalibrationData loaded;
  memset(&loaded, 0, sizeof(loaded));

  TEST_ASSERT_TRUE(CalibrationStorage::load(loaded));
  TEST_ASSERT_TRUE(loaded.gyro_calibrated);
  TEST_ASSERT_TRUE(loaded.accel_calibrated);
  TEST_ASSERT_TRUE(loaded.mag_calibrated);
  TEST_ASSERT_EQUAL_FLOAT(1.5f, loaded.gyro_offsets[0]);
  TEST_ASSERT_EQUAL_FLOAT(-2.25f, loaded.gyro_offsets[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.125f, loaded.gyro_offsets[2]);
  TEST_ASSERT_EQUAL_FLOAT(9.81f, loaded.accel_offsets[2]);
  TEST_ASSERT_EQUAL_FLOAT(4.5f, loaded.mag_declination);
  TEST_ASSERT_EQUAL_UINT32(12345, loaded.calibration_timestamp);
}

void test_load_rejects_blank_eeprom(void) {
  // Nothing saved yet - a fresh board / erased chip.
  CalibrationData loaded;
  memset(&loaded, 0xAA, sizeof(loaded)); // non-zero canary
  TEST_ASSERT_FALSE(CalibrationStorage::load(loaded));
}

void test_load_rejects_corrupted_crc(void) {
  CalibrationData original = make_sample_calibration();
  TEST_ASSERT_TRUE(CalibrationStorage::save(original));

  // Flip a byte inside the stored payload (past the 2-byte magic + 2-byte
  // version header) without updating the CRC, simulating bit-rot / a torn
  // write.
  uint8_t corrupted = EEPROM.read(CALIBRATION_EEPROM_BASE + 10);
  EEPROM.write(CALIBRATION_EEPROM_BASE + 10, corrupted ^ 0xFF);

  CalibrationData loaded;
  memset(&loaded, 0, sizeof(loaded));
  TEST_ASSERT_FALSE(CalibrationStorage::load(loaded));
}

void test_load_rejects_foreign_blob_at_same_address(void) {
  // Simulates the exact F1 collision: some other subsystem (e.g. the old
  // save_config()/load_config() format) wrote its own magic number over
  // byte 0 of the calibration region. CalibrationStorage must reject it
  // instead of misinterpreting it as valid calibration data.
  uint32_t foreign_signature = 0x12345678;
  for (size_t i = 0; i < sizeof(foreign_signature); i++) {
    EEPROM.write(CALIBRATION_EEPROM_BASE + (int)i, ((uint8_t*)&foreign_signature)[i]);
  }

  CalibrationData loaded;
  memset(&loaded, 0, sizeof(loaded));
  TEST_ASSERT_FALSE(CalibrationStorage::load(loaded));
}

void test_calibration_and_config_regions_do_not_overlap(void) {
  // The whole point of F1: the two EEPROM memory regions must be disjoint.
  TEST_ASSERT_TRUE(CONFIG_EEPROM_BASE >= CALIBRATION_EEPROM_BASE);
  TEST_ASSERT_TRUE((CONFIG_EEPROM_BASE - CALIBRATION_EEPROM_BASE) >= (int)sizeof(CalibrationData));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_calibration_survives_round_trip);
  RUN_TEST(test_load_rejects_blank_eeprom);
  RUN_TEST(test_load_rejects_corrupted_crc);
  RUN_TEST(test_load_rejects_foreign_blob_at_same_address);
  RUN_TEST(test_calibration_and_config_regions_do_not_overlap);
  return UNITY_END();
}
