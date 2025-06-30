#include "constants.h"
#include <Wire.h>
#include "optical_flow.h"
#include <SPI.h>

OpticalFlowSensor::OpticalFlowSensor() {
  initialized = false;
  sensor_type = OPTICAL_FLOW_TYPE_PMW3901;
  flow_data.data_valid = false;
  flow_data.last_update = 0;
}

void OpticalFlowSensor::init(int type) {
  sensor_type = type;
  
  Serial.print("Initializing Optical Flow Sensor (Type: ");
  Serial.print(type);
  Serial.println(")...");
  
  switch (sensor_type) {
    case OPTICAL_FLOW_TYPE_PMW3901:
      initialized = init_pmw3901();
      break;
    case OPTICAL_FLOW_TYPE_PAA5100:
      initialized = init_paa5100();
      break;
    case OPTICAL_FLOW_TYPE_ADNS3080:
      initialized = init_adns3080();
      break;
    default:
      Serial.println("Unknown optical flow sensor type");
      initialized = false;
      break;
  }
  
  if (initialized) {
    // Initialize flow data structure
    flow_data.velocity_x = 0.0f;
    flow_data.velocity_y = 0.0f;
    flow_data.displacement_x = 0.0f;
    flow_data.displacement_y = 0.0f;
    flow_data.surface_quality = 0;
    flow_data.data_valid = false;
    flow_data.last_update = millis();
    
    Serial.println("Optical flow sensor initialized successfully");
  } else {
    Serial.println("ERROR: Optical flow sensor initialization failed");
  }
}

void OpticalFlowSensor::update() {
  if (!initialized) return;
  
  static unsigned long last_update_time = 0;
  unsigned long current_time = millis();
  
  // Update at 100Hz
  if (current_time - last_update_time < 10) {
    return;
  }
  
  float dt = (current_time - last_update_time) / 1000.0f;
  last_update_time = current_time;
  
  // Read sensor data based on type
  bool read_success = false;
  
  switch (sensor_type) {
    case OPTICAL_FLOW_TYPE_PMW3901:
      read_success = read_pmw3901_data();
      break;
    case OPTICAL_FLOW_TYPE_PAA5100:
      read_success = read_paa5100_data();
      break;
    case OPTICAL_FLOW_TYPE_ADNS3080:
      read_success = read_adns3080_data();
      break;
  }
  
  if (read_success) {
    // Integrate velocity to get displacement
    flow_data.displacement_x += flow_data.velocity_x * dt;
    flow_data.displacement_y += flow_data.velocity_y * dt;
    
    flow_data.data_valid = true;
    flow_data.last_update = current_time;
  } else {
    flow_data.data_valid = false;
  }
}

bool OpticalFlowSensor::init_pmw3901() {
  // Initialize SPI for PMW3901
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  
  // Reset sensor
  write_register(0x3A, 0x5A);
  delay(50);
  
  // Read product ID
  uint8_t product_id = read_register(0x00);
  if (product_id != 0x49) {
    Serial.print("ERROR: PMW3901 product ID mismatch: 0x");
    Serial.println(product_id, HEX);
    SPI.endTransaction();
    return false;
  }
  
  // Initialize sensor settings
  write_register(0x7F, 0x00);
  write_register(0x61, 0xAD);
  write_register(0x7F, 0x03);
  write_register(0x40, 0x00);
  write_register(0x7F, 0x05);
  write_register(0x41, 0xB3);
  write_register(0x43, 0xF1);
  write_register(0x45, 0x14);
  write_register(0x5B, 0x32);
  write_register(0x5F, 0x34);
  write_register(0x7B, 0x08);
  write_register(0x7F, 0x06);
  write_register(0x44, 0x1B);
  write_register(0x40, 0xBF);
  write_register(0x4E, 0x3F);
  write_register(0x7F, 0x08);
  write_register(0x65, 0x20);
  write_register(0x6A, 0x18);
  write_register(0x7F, 0x09);
  write_register(0x4F, 0xAF);
  write_register(0x5F, 0x40);
  write_register(0x48, 0x80);
  write_register(0x49, 0x80);
  write_register(0x57, 0x77);
  write_register(0x60, 0x78);
  write_register(0x61, 0x78);
  write_register(0x62, 0x08);
  write_register(0x63, 0x50);
  write_register(0x7F, 0x0A);
  write_register(0x45, 0x60);
  write_register(0x7F, 0x00);
  write_register(0x4D, 0x11);
  write_register(0x55, 0x80);
  write_register(0x74, 0x1F);
  write_register(0x75, 0x1F);
  write_register(0x4A, 0x78);
  write_register(0x4B, 0x78);
  write_register(0x44, 0x08);
  write_register(0x45, 0x50);
  write_register(0x64, 0xFF);
  write_register(0x65, 0x1F);
  write_register(0x7F, 0x14);
  write_register(0x65, 0x67);
  write_register(0x66, 0x08);
  write_register(0x63, 0x70);
  write_register(0x7F, 0x15);
  write_register(0x48, 0x48);
  write_register(0x7F, 0x07);
  write_register(0x41, 0x0D);
  write_register(0x43, 0x14);
  write_register(0x4B, 0x0E);
  write_register(0x45, 0x0F);
  write_register(0x44, 0x42);
  write_register(0x4C, 0x80);
  write_register(0x7F, 0x10);
  write_register(0x5B, 0x02);
  write_register(0x7F, 0x07);
  write_register(0x40, 0x41);
  write_register(0x70, 0x00);
  
  delay(10);
  write_register(0x32, 0x44);
  write_register(0x7F, 0x07);
  write_register(0x40, 0x40);
  write_register(0x7F, 0x06);
  write_register(0x62, 0xF0);
  write_register(0x63, 0x00);
  write_register(0x7F, 0x0D);
  write_register(0x48, 0xC0);
  write_register(0x6F, 0xD5);
  write_register(0x7F, 0x00);
  write_register(0x5B, 0xA0);
  write_register(0x4E, 0xA8);
  write_register(0x5A, 0x50);
  write_register(0x40, 0x80);
  
  SPI.endTransaction();
  return true;
}

bool OpticalFlowSensor::init_paa5100() {
  // PAA5100 is I2C based
  Wire.beginTransmission(0x50); // PAA5100 default address
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: PAA5100 not found on I2C");
    return false;
  }
  
  // Initialize PAA5100 (simplified)
  Serial.println("PAA5100 initialization complete");
  return true;
}

bool OpticalFlowSensor::init_adns3080() {
  // ADNS3080 SPI initialization
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  
  // Reset and configure ADNS3080
  write_register(0x3A, 0x5A);
  delay(50);
  
  // Read product ID
  uint8_t product_id = read_register(0x00);
  if (product_id != 0x17) {
    Serial.print("ERROR: ADNS3080 product ID mismatch: 0x");
    Serial.println(product_id, HEX);
    SPI.endTransaction();
    return false;
  }
  
  SPI.endTransaction();
  Serial.println("ADNS3080 initialization complete");
  return true;
}

bool OpticalFlowSensor::read_pmw3901_data() {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  
  // Read motion register
  uint8_t motion = read_register(0x02);
  
  if (motion & 0x80) { // Motion detected
    // Read delta X and Y
    int16_t delta_x = (int16_t)read_register(0x03);
    int16_t delta_y = (int16_t)read_register(0x04);
    
    // Read surface quality
    flow_data.surface_quality = read_register(0x05);
    
    // Convert to velocity (simplified conversion)
    // In practice, this would depend on height above ground
    float scale = 0.1f; // Scale factor
    flow_data.velocity_x = delta_x * scale;
    flow_data.velocity_y = delta_y * scale;
    
    SPI.endTransaction();
    return (flow_data.surface_quality > OPTICAL_FLOW_MIN_SURFACE_QUALITY);
  }
  
  SPI.endTransaction();
  return false;
}

bool OpticalFlowSensor::read_paa5100_data() {
  // Read PAA5100 over I2C (simplified implementation)
  Wire.beginTransmission(0x50);
  Wire.write(0x02); // Motion register
  Wire.endTransmission();
  
  Wire.requestFrom(0x50, 1);
  if (Wire.available()) {
    uint8_t motion = Wire.read();
    
    if (motion & 0x80) {
      // Read X and Y motion data
      // This is simplified - actual implementation would read multiple registers
      flow_data.velocity_x = 0.0f; // Placeholder
      flow_data.velocity_y = 0.0f; // Placeholder
      flow_data.surface_quality = 100; // Placeholder
      return true;
    }
  }
  
  return false;
}

bool OpticalFlowSensor::read_adns3080_data() {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  
  // Read motion register
  uint8_t motion = read_register(0x02);
  
  if (motion & 0x80) { // Motion detected
    // Read delta X and Y
    int8_t delta_x = (int8_t)read_register(0x03);
    int8_t delta_y = (int8_t)read_register(0x04);
    
    // Read SQUAL (surface quality)
    flow_data.surface_quality = read_register(0x05);
    
    // Convert to velocity
    float scale = 0.05f;
    flow_data.velocity_x = delta_x * scale;
    flow_data.velocity_y = delta_y * scale;
    
    SPI.endTransaction();
    return (flow_data.surface_quality > 30); // Minimum quality threshold
  }
  
  SPI.endTransaction();
  return false;
}

uint8_t OpticalFlowSensor::read_register(uint8_t reg) {
  digitalWrite(SS, LOW);
  SPI.transfer(reg & 0x7F); // Read bit (MSB = 0)
  delayMicroseconds(100);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(SS, HIGH);
  delayMicroseconds(100);
  return data;
}

void OpticalFlowSensor::write_register(uint8_t reg, uint8_t value) {
  digitalWrite(SS, LOW);
  SPI.transfer(reg | 0x80); // Write bit (MSB = 1)
  SPI.transfer(value);
  digitalWrite(SS, HIGH);
  delayMicroseconds(100);
}

OpticalFlowData OpticalFlowSensor::get_data() {
  return flow_data;
}

bool OpticalFlowSensor::is_healthy() {
  return initialized && flow_data.data_valid && (millis() - flow_data.last_update < 100);
} 