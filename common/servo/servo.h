#pragma once

#include <hardware/pwm.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/divider.h"
#include <math.h>
#include <iostream>
#include "../detectors/detectors.h"
#include "../euler/euler.h"

#define SERVO_DEGREE_IS_BELOW_MINIMUM 1
#define SERVO_DEGREE_IS_ABOVE_MAXIMUM 2
#define SERVO_DEGREE_IS_NAN 3

class Servo {
  private:
    uint slice;
    uint channel;    
    uint8_t clk_divider = 1;
    uint32_t wrap = 0;
    float period = 0;
    const float lowPeriod;
    const float highPeriod;
    const float maxDegree;
    const float minDegree;
    const float homePosition;
    uint16_t lowSlices;
    uint16_t highSlices;
    uint16_t delta;
    uint16_t lowBorder;
    float step;
    uint16_t getSlices(float targetPeriod);
    uint setFrequency(const uint freq);    
    ImuUseAngle useAngle;    
    RangeMap imuMap;
    float currentAngle = homePosition;
    uint setDegreeDirect(const float degree);    
    volatile float targetAngle = NAN;
    float speed = 0;
  public:
    Servo(
      const uint pin, 
      Range degreeRange, 
      Range imuRange, 
      ImuUseAngle useAngle, 
      const float homePosition, 
      const float freq = 50, 
      const float lowPeriod = 0.0005, 
      const float highPeriod = 0.0025
    );
    bool setTargetAngle(const float angle);
    bool isStopped() {
      return fabs(currentAngle - targetAngle) < 0.01f;
    }

    bool atHomePosition() {
      return isStopped() && (currentAngle == homePosition);
    }
    float getSpeed();
    float getImuAngle();
    Euler euler;
    void tick();
};