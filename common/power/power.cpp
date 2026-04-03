#include "power.h"

uint32_t Power::serialize() const {
  uint32_t result = 0;
  result |= voltage << 16;
  result |= current;
  return result;
}

void Power::deserialize(uint8_t data[4]) {
  current = (static_cast<uint16_t>(data[1]) << 8) | data[0];
  voltage = (static_cast<uint16_t>(data[3]) << 8) | data[2];
}