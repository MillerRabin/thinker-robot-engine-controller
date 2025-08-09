#pragma once

#include <hardware/pwm.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/divider.h"
#include <math.h>
#include <iostream>
#include "../../common/detectors/detectors.h"

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
    float currentAngle = homePosition;
    float imuAngle = NAN;
    uint setDegreeDirect(const float degree);    
    volatile float targetAngle = NAN;
    float speed = 0;
    float filteredImuAngle = NAN;
    static bool equalAngles(float a, float b);
  public:
    Servo(
        const uint pin, 
        Range degreeRange,
        const float homePosition,
        const float freq = 50,
        const float lowPeriod = 0.0005,
        const float highPeriod = 0.0025);
    bool setTargetAngle(const float angle);
    float getTargetAngle() { return targetAngle; };
    bool isStopped();
    bool atHomePosition();    
    void setIMUAngle(float value);
    float getCurrentAngle() { return currentAngle; }; 
    float getIMUAngle() { return imuAngle; }
    bool isCalibrating();    
    void tick();
};