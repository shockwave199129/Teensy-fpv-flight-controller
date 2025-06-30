#ifndef DUAL_IMU_MANAGER_H
#define DUAL_IMU_MANAGER_H

#include "config.h"

class DualIMUManager {
private:
  bool primary_imu_initialized;
  bool secondary_imu_initialized;
  bool using_primary_imu;
  bool validation_enabled;
  unsigned long last_validation_time;
  uint8_t cross_check_failures;
  IMUData fused_data;
  
  void validate_imu_data(const IMUData& primary, const IMUData& secondary);
  bool read_imu_data_secondary(IMUData* data);
  
public:
  DualIMUManager();
  void init();
  void update();
  IMUData get_fused_data();
  bool is_healthy();
  bool using_primary();
};

#endif // DUAL_IMU_MANAGER_H 