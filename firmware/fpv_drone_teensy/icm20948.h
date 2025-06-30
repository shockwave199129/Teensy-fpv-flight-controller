#pragma once

#include <Arduino.h>
#include "config.h"

// Simple ICM20948 gyro/accel/mag driver (SPI) – minimal subset

#define ICM20948_CS_PIN 10

bool icm20948_init();
bool icm20948_read(IMUData* data);
bool icm20948_self_test();
void icm20948_calibrate_bias(float* gyro_bias_xyz); 