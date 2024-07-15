#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"
#include <iostream>

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
