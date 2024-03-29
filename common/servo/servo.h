#pragma once

#include <hardware/pwm.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/divider.h"
#include <iostream>

class Servo {
  private:
    uint slice;
    uint channel;    
    uint8_t clk_divider = 1;
    uint32_t wrap = 0;
    float period = 0;
    const float lowPeriod;
    const float highPeriod;
    const uint maxDegree;
    uint16_t lowSlices;
    uint16_t highSlices;
    uint16_t delta;
    uint16_t lowBorder;
    float step;
    uint16_t getSlices(float targetPeriod);
    uint setFrequency(const uint freq);    
  public:
    Servo(const uint pin, const uint maxDegree = 270, const float freq = 50, const float lowPeriod = 0.0005, const float highPeriod = 0.0025);    
    uint setDegree(const float degree);    
};