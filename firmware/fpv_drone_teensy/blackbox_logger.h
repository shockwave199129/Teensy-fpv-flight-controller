#pragma once
#include "config.h"
#include "sensor_redundancy.h"
#include "motor_control.h"
#include "pid_controller.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <cstring>

#ifndef BLACKBOX_SD_CS_PIN
#define BLACKBOX_SD_CS_PIN 10
#endif

#define BLACKBOX_MAX_LOG_FILES 20

// Very lightweight blackbox logger that streams a compact binary frame over
// Serial3 (2 MBaud by default). The frame contains a sync byte, timestamp,
// attitude, rates, RC inputs and motor commands. Use desktop app to decode.
class BlackboxLogger {
public:
  void begin(uint32_t baud = 2000000) {
    start_time_us = micros();

    // If USB serial connected (PC), don't log to SD to avoid unnecessary writes
    if (Serial && Serial.dtr()) {
      enabled = false;
      sd_ok = false;
      enabled = false; // disable logging if SD not present
      return;
    }

    sd_ok = SD.begin(BLACKBOX_SD_CS_PIN);
    if(sd_ok){
      // Generate unique filename with timestamp counter (seconds since boot)
      uint32_t sec = (millis() / 1000UL);
      char fname[20];
      snprintf(fname, sizeof(fname), "bb_%lu.bin", (unsigned long)sec);
      logFile = SD.open(fname, FILE_WRITE);
      if(!logFile){
        sd_ok = false;
      }
    }
    if(!sd_ok){
      enabled = false;
      return;
    }
    Serial3.begin(baud); // optional serial stream for live view
    Serial3.write('\xA5');
    logFile.write('\xA5');

    enforceLogLimit(); // ensure storage limits
  }

  void logFrame(const SensorData &sensors,
                const RcData &rc,
                const PIDOutput &pid,
                const MotorOutput &motors) {
    if (!enabled || !sd_ok) return;

    uint32_t t = micros() - start_time_us;
    write32(t);
    write16((int16_t)(sensors.roll * 100));
    write16((int16_t)(sensors.pitch * 100));
    write16((int16_t)(sensors.yaw * 100));
    write16((int16_t)(sensors.imu.gyro_x * 10));
    write16((int16_t)(sensors.imu.gyro_y * 10));
    write16((int16_t)(sensors.imu.gyro_z * 10));
    write16((uint16_t)rc.throttle);
    write16((int16_t)pid.roll);
    write16((int16_t)pid.pitch);
    write16((int16_t)pid.yaw);
    write16((uint16_t)motors.motor1);
    write16((uint16_t)motors.motor2);
    write16((uint16_t)motors.motor3);
    write16((uint16_t)motors.motor4);
  }

  void setEnabled(bool e) { enabled = e; }
  bool isEnabled() const { return enabled; }

  // Delete all log files
  void clearLogs(){
    File root = SD.open("/");
    if(!root) return;
    File f = root.openNextFile();
    while(f){
      if(!f.isDirectory() && startsWithBB(f.name())){
        SD.remove(f.name());
      }
      f.close();
      f = root.openNextFile();
    }
    root.close();
  }

private:
  uint32_t start_time_us = 0;
  bool enabled = true;
  bool sd_ok = false;
  File logFile;

  inline void write16(int16_t v) {
    Serial3.write((uint8_t*)&v,2);
    if(sd_ok) logFile.write((uint8_t*)&v,2);
  }
  inline void write32(uint32_t v) {
    Serial3.write((uint8_t*)&v,4);
    if(sd_ok) logFile.write((uint8_t*)&v,4);
  }

  bool startsWithBB(const char *name){
    return name[0]=='b' && name[1]=='b' && name[2]=='_';
  }

  void enforceLogLimit(){
    // Count log files, if exceed max delete oldest by filename.
    struct Entry{char name[32];};
    Entry oldest={""}; int count=0;
    File root = SD.open("/");
    if(!root) return;
    File f = root.openNextFile();
    while(f){
      if(!f.isDirectory() && startsWithBB(f.name())){
        count++;
        if(strlen(oldest.name)==0 || strcmp(f.name(),oldest.name)<0){
          strncpy(oldest.name,f.name(),31);
          oldest.name[31]='\0';
        }
      }
      f.close();
      f = root.openNextFile();
    }
    root.close();
    if(count>BLACKBOX_MAX_LOG_FILES && strlen(oldest.name)){
      SD.remove(oldest.name);
    }
  }
}; 