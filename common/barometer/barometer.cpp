#include "barometer.h"

uint64_t Barometer::serialize() {
  return ((uint64_t)(uint32_t)temperatureRaw << 32) |
          (uint64_t)(uint32_t)heightRaw;
}

void Barometer::deserialize(uint8_t data[8])
{
  memcpy(&heightRaw, data, 4);    
  memcpy(&temperatureRaw, &data[4], 4); 
}

void Barometer::set(int32_t height, uint16_t temperature) {  
  this->temperatureRaw = temperature; 
  this->heightRaw = height;  
}
