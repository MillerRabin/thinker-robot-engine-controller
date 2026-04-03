#pragma once

#include "pico/stdlib.h"
#include <cstring>

class Barometer {
  private:
    uint16_t temperatureRaw;    
    int32_t heightRaw;
  public:
    uint64_t serialize();
    void deserialize(uint8_t data[8]);
    void set(int32_t height, uint16_t temperature);
    float getTemperature() { return temperatureRaw / 100.0f; };
    float getHeight() { return heightRaw / 100.0f; };
};
