#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include <iostream>

#include "../config/config.h"

class MeasureRange {
  private:
    uint16_t range;
    uint8_t measureType;;
  public:
    uint64_t serialize();
    void deserialize(uint8_t data[8]);
    void set(uint16_t range, uint8_t measureType);
};
