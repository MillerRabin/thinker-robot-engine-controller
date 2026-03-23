#include "accelerometer.h"

uint64_t Accelerometer::serialize() {
  uint16_t x = floatToQ(this->x, Q1);
  uint16_t y = floatToQ(this->y, Q1);
  uint16_t z = floatToQ(this->z, Q1);

  return (uint64_t)x | (uint64_t)y << 16 | (uint64_t)z << 32;
}

void Accelerometer::deserialize(uint8_t data[8]) {
  uint16_t x = (int16_t)data[1] << 8 | data[0];
  uint16_t y = (int16_t)data[3] << 8 | data[2];
  uint16_t z = (int16_t)data[5] << 8 | data[4];
  this->x = qToFloat(x, Q1);
  this->y = qToFloat(y, Q1);
  this->z = qToFloat(z, Q1);
}

void Accelerometer::fromWitmotion(int16_t rawAccX, int16_t rawAccY,
                                  int16_t rawAccZ) {
  this->x = rawAccX / 32768.0f * 16.0f * 9.80665f;
  this->y = rawAccY / 32768.0f * 16.0f * 9.80665f;
  this->z = rawAccY / 32768.0f * 16.0f * 9.80665f;
}

void Accelerometer::fromBNO(int16_t rawAccX, int16_t rawAccY, int16_t rawAccZ) {
  this->x = qToFloat(rawAccX, Q1);
  this->y = qToFloat(rawAccY, Q1);
  this->z = qToFloat(rawAccZ, Q1);
}