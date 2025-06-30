#include "config.h"
#include "dual_imu_manager.h"
#include "dynamic_filtering.h"
#include "optical_flow.h"
#include "advanced_flight_modes.h"

// External references to global instances from main .ino file
extern DualIMUManager dual_imu_manager;
extern DynamicFilteringSystem dynamic_filtering;
extern OpticalFlowSensor optical_flow;
extern AdvancedFlightModes advanced_flight_modes;

void init_dual_imu_system() {
  Serial.println("Initializing Phase 2 Systems...");
  
  dual_imu_manager.init();
  dynamic_filtering.init();
  
  #ifdef ENABLE_OPTICAL_FLOW
  optical_flow.init(OPTICAL_FLOW_TYPE_PMW3901); // Use PMW3901 as default
  #endif
  
  advanced_flight_modes.init();
  
  Serial.println("Phase 2 systems initialized successfully");
} 