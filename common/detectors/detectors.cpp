#include "detectors.h"

uint64_t Quaternion::serialize() {
  return (uint64_t)this->i |
         (uint64_t)this->j << 16 |
         (uint64_t)this->k << 32 |
         (uint64_t)this->real << 48;
}

void Quaternion::deserialize(uint8_t data[8]) {
  this->i = (uint16_t)data[1] << 8 | data[0];
  this->j = (uint16_t)data[3] << 8 | data[2];
  this->k = (uint16_t)data[5] << 8 | data[4];
  this->real = (uint16_t)data[7] << 8 | data[6];
}

uint64_t Accelerometer::serialize() {
  return (uint64_t)this->x |
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

void Accelerometer::deserialize(uint8_t data[8]) {
  this->x = (uint16_t)data[1] << 8 | data[0];
  this->y = (uint16_t)data[3] << 8 | data[2];
  this->z = (uint16_t)data[5] << 8 | data[4];  
}

uint64_t Gyroscope::serialize() {
  return (uint64_t)this->x |
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

void Gyroscope::deserialize(uint8_t data[8]) {
  this->x = (uint16_t)data[1] << 8 | data[0];
  this->y = (uint16_t)data[3] << 8 | data[2];
  this->z = (uint16_t)data[5] << 8 | data[4];  
}
                  
uint64_t Accuracy::serialize() {
  return (uint64_t)this->quaternionAccuracy |
         (uint64_t)this->accelerometerAccuracy << 8 |
         (uint64_t)this->quaternionRadAccuracy << 16 |
         (uint64_t)this->gyroscopeAccuracy << 32;
}

void Accuracy::deserialize(uint8_t data[8]) {
  this->quaternionAccuracy = data[0];
  this->accelerometerAccuracy = data[1];
  this->quaternionRadAccuracy = (uint16_t)data[3] << 8 | data[2];
  this->gyroscopeAccuracy = data[4];  
}




