#include "gyroscope.h"

uint64_t Gyroscope::serialize() {
  uint16_t x = floatToQ(this->x, Q1);
  uint16_t y = floatToQ(this->y, Q1);
  uint16_t z = floatToQ(this->z, Q1);

  return (uint64_t)x | (uint64_t)y << 16 | (uint64_t)z << 32;
}

void Gyroscope::deserialize(uint8_t data[8]) {
  this->x = (uint16_t)data[1] << 8 | data[0];
  this->y = (uint16_t)data[3] << 8 | data[2];
  this->z = (uint16_t)data[5] << 8 | data[4];
}

void Gyroscope::set(float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

void Gyroscope::fromWitmotion(int16_t rawGyroX, int16_t rawGyroY,
                              int16_t rawGyroZ) {
  this->x = rawGyroX / 32768.0f * 2000.0f;
  this->y = rawGyroY / 32768.0f * 2000.0f;
  this->z = rawGyroZ / 32768.0f * 2000.0f;
}

void Gyroscope::fromBNO(int16_t rawGyroX, int16_t rawGyroY, int16_t rawGyroZ) {
  this->x = qToFloat(rawGyroX, Q1);
  this->y = qToFloat(rawGyroY, Q1);
  this->z = qToFloat(rawGyroZ, Q1);
}