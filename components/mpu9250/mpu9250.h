#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "Wire.h"

namespace esphome {
namespace mpu9250 {

#define MPU9250_ADDRESS_AD0_LOW             0x68
#define MPU9250_ADDRESS_AD0_HIGH            0x69

#define MPU9250_ACC_FULL_SCALE_2_G          0x00
#define MPU9250_ACC_FULL_SCALE_4_G          0x08
#define MPU9250_ACC_FULL_SCALE_8_G          0x10
#define MPU9250_ACC_FULL_SCALE_16_G         0x18

#define MPU9250_GYRO_FULL_SCALE_250_DPS     0x00
#define MPU9250_GYRO_FULL_SCALE_500_DPS     0x08
#define MPU9250_GYRO_FULL_SCALE_1000_DPS    0x10
#define MPU9250_GYRO_FULL_SCALE_2000_DPS    0x18

#define MPU9250_MAG_MODE_POWERDOWN          0x0
#define MPU9250_MAG_MODE_SINGLE             0x1
#define MPU9250_MAG_MODE_CONTINUOUS_8HZ     0x2
#define MPU9250_MAG_MODE_EXTERNAL           0x4
#define MPU9250_MAG_MODE_CONTINUOUS_100HZ   0x6
#define MPU9250_MAG_MODE_SELFTEST           0x8
#define MPU9250_MAG_MODE_FUSEROM            0xF

#define MPU9250_BUFF_LEN_ACCEL 6
#define MPU9250_BUFF_LEN_GYRO  6
#define MPU9250_BUFF_LEN_MAG   7

class MPU9250Component : public PollingComponent, public I2CDevice {
public:
    MPU9250Component(uint8_t address = MPU9250_ADDRESS_AD0_LOW)
        : PollingComponent(1000), I2CDevice(address), address(address) {}

    Sensor *accel_x_sensor = nullptr;
    Sensor *accel_y_sensor = nullptr;
    Sensor *accel_z_sensor = nullptr;
    Sensor *gyro_x_sensor = nullptr;
    Sensor *gyro_y_sensor = nullptr;
    Sensor *gyro_z_sensor = nullptr;
    Sensor *mag_x_sensor = nullptr;
    Sensor *mag_y_sensor = nullptr;
    Sensor *mag_z_sensor = nullptr;

    int16_t magXOffset, magYOffset, magZOffset;
    uint8_t accelBuff[MPU9250_BUFF_LEN_ACCEL];
    uint8_t gyroBuff[MPU9250_BUFF_LEN_GYRO];
    uint8_t magBuff[MPU9250_BUFF_LEN_MAG];

    void setup() override {
        ESP_LOGD("mpu9250", "Initialisation du MPU9250...");

        Wire.begin();
        write_register(0x1C, ACC_FULL_SCALE_16_G);
        write_register(0x1B, GYRO_FULL_SCALE_2000_DPS);
        write_register(0x37, 0x02);
        write_register(0x6A, 0x00);
        write_register(0x6B, 0x01);
        write_register(0x24, MAG_MODE_CONTINUOUS_8HZ);
    }

    void update() override {
        accelUpdate();
        gyroUpdate();
        magUpdate();
    }

    void beginAccel(uint8_t mode = ACC_FULL_SCALE_16_G) {
        write_register(0x1C, mode);
    }

    uint8_t accelUpdate() {
        read_register(0x3B, MPU9250_BUFF_LEN_ACCEL, accelBuff);
        if (accel_x_sensor) accel_x_sensor->publish_state(accelX());
        if (accel_y_sensor) accel_y_sensor->publish_state(accelY());
        if (accel_z_sensor) accel_z_sensor->publish_state(accelZ());
        return 0;
    }

    float accelX() { return ((int16_t)(accelBuff[0] << 8 | accelBuff[1])) / 16384.0; }
    float accelY() { return ((int16_t)(accelBuff[2] << 8 | accelBuff[3])) / 16384.0; }
    float accelZ() { return ((int16_t)(accelBuff[4] << 8 | accelBuff[5])) / 16384.0; }

    void beginGyro(uint8_t mode = GYRO_FULL_SCALE_2000_DPS) {
        write_register(0x1B, mode);
    }

    uint8_t gyroUpdate() {
        read_register(0x43, MPU9250_BUFF_LEN_GYRO, gyroBuff);
        if (gyro_x_sensor) gyro_x_sensor->publish_state(gyroX());
        if (gyro_y_sensor) gyro_y_sensor->publish_state(gyroY());
        if (gyro_z_sensor) gyro_z_sensor->publish_state(gyroZ());
        return 0;
    }

    float gyroX() { return ((int16_t)(gyroBuff[0] << 8 | gyroBuff[1])) / 131.0; }
    float gyroY() { return ((int16_t)(gyroBuff[2] << 8 | gyroBuff[3])) / 131.0; }
    float gyroZ() { return ((int16_t)(gyroBuff[4] << 8 | gyroBuff[5])) / 131.0; }

    void beginMag(uint8_t mode = MAG_MODE_CONTINUOUS_8HZ) {
        write_register(0x24, mode);
    }

    uint8_t magUpdate() {
        read_register(0x03, MPU9250_BUFF_LEN_MAG, magBuff);
        if (mag_x_sensor) mag_x_sensor->publish_state(magX());
        if (mag_y_sensor) mag_y_sensor->publish_state(magY());
        if (mag_z_sensor) mag_z_sensor->publish_state(magZ());
        return 0;
    }

    float magX() { return (int16_t)(magBuff[0] << 8 | magBuff[1]); }
    float magY() { return (int16_t)(magBuff[2] << 8 | magBuff[3]); }
    float magZ() { return (int16_t)(magBuff[4] << 8 | magBuff[5]); }

private:
    const uint8_t address;

    void write_register(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    void read_register(uint8_t reg, uint8_t len, uint8_t *buffer) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(address, len);
        for (uint8_t i = 0; i < len; i++) {
        buffer[i] = Wire.read();
        }
    }
};
}  // namespace mpu9250
}  // namespace esphome
