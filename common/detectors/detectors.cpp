#include "detectors.h"

const float RangeMap::getDestValue(const float sourceValue) {
  const float step = (sourceValue - source.from) / sourceDelta;  
  const float target = dest.from + (destDelta * step);  
  if ((target < dest.from) || (target > dest.to)) return NAN;
  return target;
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