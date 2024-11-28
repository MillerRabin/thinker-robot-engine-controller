#pragma once

#include <cstdint>
#include <iostream>
#include <cstring>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>

#include "../config/config.h"
#include "pico/stdlib.h"

enum ImuUseAngle {
  IMU_USE_ROLL,
  IMU_USE_PITCH,
  IMU_USE_YAW
};

class Euler {
  public:
    Euler(float roll, float pitch, float yaw);
    float yaw;
    float pitch;
    float roll;
    float getRollAngle();
    float getPitchAngle();
    float getYawAngle();
    float getYawDegree();
    float getAngle(ImuUseAngle useAngle);
};
