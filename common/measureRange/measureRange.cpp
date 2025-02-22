#include "measureRange.h"

uint64_t MeasureRange::serialize() {
  return (uint64_t)range |
         (uint64_t)measureType << 16;
}

void MeasureRange::deserialize(uint8_t data[8]) {
  this->range = (uint32_t)data;
  this->measureType = (uint32_t)&data[4];
}

void MeasureRange::set(uint32_t range, uint8_t measureType) {
  this->range = range;
  this->measureType = measureType;
}