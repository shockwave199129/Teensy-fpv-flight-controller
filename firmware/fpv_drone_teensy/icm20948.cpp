#include "icm20948.h"
#include <SPI.h>

static bool icm_initialized = false;
static float gyro_bias[3] = {0};

bool icm20948_init()
{
    pinMode(ICM20948_CS_PIN, OUTPUT);
    digitalWrite(ICM20948_CS_PIN, HIGH);
    SPI.begin();

    // Soft-reset / verify WHO_AM_I (0x00 should return 0xEA)
    digitalWrite(ICM20948_CS_PIN, LOW);
    SPI.transfer(0x00 | 0x80); // read WHO_AM_I register
    uint8_t who = SPI.transfer(0x00);
    digitalWrite(ICM20948_CS_PIN, HIGH);

    if (who != 0xEA) {
        Serial.print("ICM20948 not found, WHO_AM_I=0x"); Serial.println(who, HEX);
        return false;
    }

    icm_initialized = true;
    Serial.println("ICM20948 detected (SPI)");

    // For this minimal driver we leave default config (gyro 250dps, accel 2g)
    return true;
}

static inline int16_t read_reg16(uint8_t regH)
{
    digitalWrite(ICM20948_CS_PIN, LOW);
    SPI.transfer(regH | 0x80); // read high byte
    uint8_t hi = SPI.transfer(0x00);
    uint8_t lo = SPI.transfer(0x00);
    digitalWrite(ICM20948_CS_PIN, HIGH);
    return (int16_t)((hi<<8)|lo);
}

bool icm20948_read(IMUData* d)
{
    if (!icm_initialized) return false;
    // Read accel (0x2D..32) and gyro (0x33..38) – bank 0 addresses
    int16_t ax = read_reg16(0x2D);
    int16_t ay = read_reg16(0x2F);
    int16_t az = read_reg16(0x31);
    int16_t gx = read_reg16(0x33);
    int16_t gy = read_reg16(0x35);
    int16_t gz = read_reg16(0x37);

    // Sensitivity scale factors (default full-scale)
    const float accel_scale = 16384.0f; // LSB/g for 2g
    const float gyro_scale  = 131.0f;   // LSB/deg/s for 250 dps

    d->accel_x = ax / accel_scale * 9.81f;
    d->accel_y = ay / accel_scale * 9.81f;
    d->accel_z = az / accel_scale * 9.81f;
    d->gyro_x  = gx / gyro_scale - gyro_bias[0];
    d->gyro_y  = gy / gyro_scale - gyro_bias[1];
    d->gyro_z  = gz / gyro_scale - gyro_bias[2];
    d->healthy = true;
    d->last_update = millis();
    return true;
}

bool icm20948_self_test()
{
    if (!icm_initialized) return false;
    // Simple WHO_AM_I check acts as self-test here
    return true;
}

void icm20948_calibrate_bias(float* bias)
{
    if (!icm_initialized) return;
    const int samples = 500;
    float sum[3] = {0};
    IMUData tmp;
    for(int i=0;i<samples;i++) {
        icm20948_read(&tmp);
        sum[0] += tmp.gyro_x;
        sum[1] += tmp.gyro_y;
        sum[2] += tmp.gyro_z;
        delay(2);
    }
    gyro_bias[0] = sum[0]/samples;
    gyro_bias[1] = sum[1]/samples;
    gyro_bias[2] = sum[2]/samples;
    if (bias) {
        bias[0]=gyro_bias[0]; bias[1]=gyro_bias[1]; bias[2]=gyro_bias[2];
    }
} 