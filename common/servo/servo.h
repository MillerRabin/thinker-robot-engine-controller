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
    const float homePosition;
    float deadZone = 0.01f;
    uint16_t timeMS = 1000;
    uint16_t lowSlices;
    uint16_t highSlices;
    uint16_t delta;
    uint16_t lowBorder;
    float step;
    float angleStep;
    uint16_t getSlices(float targetPeriod);
    uint setFrequency(const uint freq);    
    float physicalAngle = homePosition;
    float prevPhysicalAngle = NAN;
    float imuAngle = NAN;
    uint setDegreeDirect(const float degree);    
    volatile float targetAngle = NAN;    
    float filteredImuAngle = NAN;
    bool wasCalibrated = false;
    float prevImuAngle = NAN;
    static bool equalAngles(float a, float b);
    float getDir(float path);
    public:
      Servo(
          const uint pin,
          Range degreeRange,
          const float homePosition,
          const float freq = 50,
          const float lowPeriod = 0.0005,
          const float highPeriod = 0.0025);
      const float maxDegree;
      const float minDegree;
      bool setTargetAngle(const float angle, uint16_t timeMS, float deadZone);
      float getTargetAngle() { return targetAngle; };
      bool isStopped();
      bool atHomePosition();
      void setIMUAngle(float value);
      float getPhysicalAngle() { return physicalAngle; };
      float getIMUAngle() { return imuAngle; }
      void setTimeMS(uint16_t timeMS);
      void setDeadZone(float dz);
      bool isCalibrating();
      void tick();
    };