#pragma once

#include "../config/config.h"
#include "../periodic/periodic.h"
#include "../speedBuffer/speedBuffer.h"
#include "hardware/clocks.h"
#include "pico/divider.h"
#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <algorithm>
#include <hardware/pwm.h>
#include <iostream>
#include <math.h>
#include <task.h>

#define SERVO_OK 0
#define SERVO_DEGREE_IS_BELOW_MINIMUM 1
#define SERVO_DEGREE_IS_ABOVE_MAXIMUM 2
#define SERVO_DEGREE_IS_NAN 3
#define SERVO_NOT_INITIALIZED 4
#define SERVO_FREQUENCY_OUT_OF_RANGE 5
#define SERVO_WRONG_PERIODS 6

class Range {
public:
  const float from;
  const float to;
  Range(const float from, const float to) : from(from), to(to) {}
};

struct PWMParameters {
  uint8_t divInt;
  uint8_t divFrac;
  uint32_t wrap;
};

class Servo {
private:
  uint pin;
  uint slice;
  uint channel;

  SpeedBuffer speedBuffer;
  uint8_t clk_divider = 1;
  const float lowPeriod;
  const float highPeriod;
    
  uint16_t lowSlices = 0;
  uint16_t highSlices = 0;
  uint16_t delta = 0;
  float pulseStep = 0.0f;
  bool initialized = false;  
  TickType_t positionTime = 0;
  const TickType_t positionInterval = pdMS_TO_TICKS(1000);
  constexpr static float tickInterval = (float)ENGINE_TASK_LOOP_TIMEOUT_US;
  // =========================================================
  // Servo / movement control settings
  // =========================================================
  float deadZone = 1.0f; // допустимая ошибка по углу (deg)
  const uint64_t stabilizationTimeUs = 500000;
  
  float stabilizationSpeed = 1.0f; // deg/s

  const float maxAngularSpeed = 60.0f;   // deg/s
  float maxAngularAccelerationCmd = 180.0f; // deg/s^2  
  const float maxAngleStepPerTick = 30.0f; // deg, to prevent too large steps in case of long delays
  int64_t timeUs = 1000000; // default 1 second
  absolute_time_t moveStarted = 0;
  absolute_time_t lastTickTime;
  float velocityKp = 0.3f;  // velocity loop gain
  
  float imuAngle = NAN;      // real angle from IMU
  float lastIMUAngle = NAN;  // last angle from IMU, used for acceleration estimation
  float physicalAngle = NAN; // commanded servo angle
  float targetAngle = NAN;   // target angle
  
  int getWrapAndDivider(const uint freq, PWMParameters& params) const;  
  uint16_t getSlices(const float targetPeriod, const float period, const uint32_t wrap) const;  
  Periodic printer;

public:  
  const float maxDegree;
  const float minDegree;
  int setDegreeDirect(const float degree);
  int setFrequency(const uint freq);

  Servo(const uint pin, Range degreeRange, const float homePosition,
        const float freq = 50, const float lowPeriod = 0.0005f,
        const float highPeriod = 0.0025f);
  bool setTargetAngle(const float angle, uint16_t timeMS, float deadZone);
  float getTargetAngle() const { return targetAngle; }

  void setIMUAngle(float value);
  
  float getPhysicalAngle() const { return physicalAngle; }
  float getIMUAngle() const { return imuAngle; }

  // =========================================================
  // Config setters
  // =========================================================
  void setTimeMS(uint16_t timeMS);
  void setDeadZone(float dz);  
  void tick();

  void stop() {
    targetAngle = NAN;    
  }

  bool isPositioned() const;
};