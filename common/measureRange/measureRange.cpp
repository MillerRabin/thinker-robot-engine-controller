#include "measureRange.h"

uint64_t MeasureRange::serialize() {    
  return (uint64_t)this->longRange |
         (uint64_t)this->shortRange << 16;
}

void MeasureRange::deserialize(uint8_t data[8]) {  
  memcpy(&this->longRange, data, 2);
  memcpy(&this->shortRange, &data[2], 2);
}

void MeasureRange::set(uint16_t longRange, uint16_t shortRange) {
  this->longRange = longRange;
  this->shortRange = shortRange;
}