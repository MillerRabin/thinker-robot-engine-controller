#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"
#include <iostream>

class StructureBase {
  public:
    float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
    uint16_t floatToQ(float q, uint8_t qPoint);    
};
