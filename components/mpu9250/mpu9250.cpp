#include "mpu9250.h"
#include "esphome.h"
#include <Wire.h>

#define AK8963_ADDRESS  0x0C
#define AK8963_RA_HXL   0x03
#define AK8963_RA_CNTL1 0x0A
#define AK8963_RA_ASAX  0x10

#define MPU9250_ADDR_ACCELCONFIG  0x1C
#define MPU9250_ADDR_INT_PIN_CFG  0x37
#define MPU9250_ADDR_ACCEL_XOUT_H 0x3B
#define MPU9250_ADDR_GYRO_XOUT_H  0x43
#define MPU9250_ADDR_PWR_MGMT_1   0x6B
#define MPU9250_ADDR_WHOAMI       0x75

MPU9250Component::MPU9250Component(uint8_t address) : address(address) {
  accelRange = 0;
  gyroRange  = 0;
  magXOffset = 0;
  magYOffset = 0;
  magZOffset = 0;
}

void MPU9250Component::beginAccel(uint8_t mode) {
  Wire.begin();
  switch (mode) {
    case ACC_FULL_SCALE_2_G: accelRange = 2.0; break;
    case ACC_FULL_SCALE_4_G: accelRange = 4.0; break;
    case ACC_FULL_SCALE_8_G: accelRange = 8.0; break;
    case ACC_FULL_SCALE_16_G: accelRange = 16.0; break;
    default: return;
  }
  i2cWriteByte(address, MPU9250_ADDR_ACCELCONFIG, mode);
  delay(10);
}

void MPU9250Component::beginGyro(uint8_t mode) {
  Wire.begin();
  switch (mode) {
    case GYRO_FULL_SCALE_250_DPS: gyroRange = 250.0; break;
    case GYRO_FULL_SCALE_500_DPS: gyroRange = 500.0; break;
    case GYRO_FULL_SCALE_1000_DPS: gyroRange = 1000.0; break;
    case GYRO_FULL_SCALE_2000_DPS: gyroRange = 2000.0; break;
    default: return;
  }
  i2cWriteByte(address, 27, mode);
  delay(10);
}

uint8_t MPU9250Component::i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  if (Wire.endTransmission() != 0) return 1;
  Wire.requestFrom(Address, Nbytes);
  for (uint8_t i = 0; i < Nbytes && Wire.available(); i++) {
    Data[i] = Wire.read();
  }
  return 0;
}

uint8_t MPU9250Component::i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  return Wire.endTransmission();
}

uint8_t MPU9250Component::accelUpdate() {
  return i2cRead(address, MPU9250_ADDR_ACCEL_XOUT_H, 6, accelBuff);
}

float MPU9250Component::accelX() { return accelGet(0, 1); }
float MPU9250Component::accelY() { return accelGet(2, 3); }
float MPU9250Component::accelZ() { return accelGet(4, 5); }
float MPU9250Component::accelGet(uint8_t highIndex, uint8_t lowIndex) {
  int16_t v = ((int16_t) accelBuff[highIndex]) << 8 | accelBuff[lowIndex];
  return ((float) -v) * accelRange / 32768.0;
}

uint8_t MPU9250Component::gyroUpdate() {
  return i2cRead(address, MPU9250_ADDR_GYRO_XOUT_H, 6, gyroBuff);
}

float MPU9250Component::gyroX() { return gyroGet(0, 1); }
float MPU9250Component::gyroY() { return gyroGet(2, 3); }
float MPU9250Component::gyroZ() { return gyroGet(4, 5); }
float MPU9250Component::gyroGet(uint8_t highIndex, uint8_t lowIndex) {
  int16_t v = ((int16_t) gyroBuff[highIndex]) << 8 | gyroBuff[lowIndex];
  return ((float) -v) * gyroRange / 32768.0;
}

void MPU9250Component::beginMag(uint8_t mode) {
  Wire.begin();
  magSetMode(MAG_MODE_POWERDOWN);
  magSetMode(MAG_MODE_FUSEROM);
  uint8_t buff[3];
  i2cRead(AK8963_ADDRESS, AK8963_RA_ASAX, 3, buff);
  magXAdjust = buff[0];
  magYAdjust = buff[1];
  magZAdjust = buff[2];
  magSetMode(MAG_MODE_POWERDOWN);
  magSetMode(mode);
  delay(10);
}

uint8_t MPU9250Component::magUpdate() {
  return i2cRead(AK8963_ADDRESS, AK8963_RA_HXL, 7, magBuff);
}

float MPU9250Component::magX() { return adjustMagValue(magGet(1, 0), magXAdjust) + magXOffset; }
float MPU9250Component::magY() { return adjustMagValue(magGet(3, 2), magYAdjust) + magYOffset; }
float MPU9250Component::magZ() { return adjustMagValue(magGet(5, 4), magZAdjust) + magZOffset; }
float MPU9250Component::magGet(uint8_t highIndex, uint8_t lowIndex) {
  return (((int16_t) magBuff[highIndex]) << 8) | magBuff[lowIndex];
}
float MPU9250Component::adjustMagValue(int16_t value, uint8_t adjust) {
  return ((float) value * (((((float) adjust - 128) * 0.5) / 128) + 1));
}
