#pragma once

#include "../config/config.h"
#include "hardware/clocks.h"
#include "pico/divider.h"
#include "pico/stdlib.h"
#include <hardware/pwm.h>
#include <iostream>
#include <math.h>

#define SERVO_DEGREE_IS_BELOW_MINIMUM 1
#define SERVO_DEGREE_IS_ABOVE_MAXIMUM 2
#define SERVO_DEGREE_IS_NAN 3

class Range {
public:
  const float from;
  const float to;
  Range(const float from, const float to) : from(from), to(to) {}
};

class Servo {
private:
  uint slice;
  uint channel;
  uint8_t clk_divider = 1;
  uint32_t wrap = 0;
  float period = 0.0f;

  const float lowPeriod;
  const float highPeriod;
  const float homePosition;

  uint16_t lowSlices = 0;
  uint16_t highSlices = 0;
  uint16_t delta = 0;
  float pulseStep = 0.0f;

  float deadZone = 0.01f;
  uint16_t timeMS = 1000;

  // IMU data
  float currentAcceleration = 0.0f; // filtered accel along axis
  float currentAngularSpeed = 0.0f; // filtered deg/sec from gyro

  // regulator settings
  float maxAngularSpeedCmd = 180.0f; // deg/sec
  const float maxAngleStepPerTick = 5.0f;  // deg per tick

  const float Kp = 6.0f;
  const float Kd = 0.12f;
  const float Ka = 0.02f;

  float physicalAngle = homePosition; // commanded servo angle
  float imuAngle = NAN;               // real angle from IMU
  volatile float targetAngle = NAN;

  uint16_t getSlices(float targetPeriod);
  uint setFrequency(const uint freq);
  uint setDegreeDirect(const float degree);

public:
  Servo(const uint pin, Range degreeRange, const float homePosition,
        const float freq = 50, const float lowPeriod = 0.0005f,
        const float highPeriod = 0.0025f);

  const float maxDegree;
  const float minDegree;

  bool setTargetAngle(const float angle, uint16_t timeMS, float deadZone);
  float getTargetAngle() { return targetAngle; }

  void setIMUAngle(float value);

  // value from accelerometer axis
  void setAcceleration(float value) {
    currentAcceleration = currentAcceleration * 0.8f + value * 0.2f;
  }

  // value from gyro axis in RAD/SEC
  void setAngularSpeed(float value) {
    const float degPerSec = value * (180.0f / (float)M_PI);
    currentAngularSpeed = currentAngularSpeed * 0.8f + degPerSec * 0.2f;
  }

  float getPhysicalAngle() { return physicalAngle; }
  float getIMUAngle() { return imuAngle; }

  void setTimeMS(uint16_t timeMS);
  void setDeadZone(float dz);
  void tick();
};