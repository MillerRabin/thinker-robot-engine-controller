#include "euler.h"

Euler::Euler(float roll, float pitch, float yaw) : 
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

float Euler::getRollAngle() { return roll * 180.0 / PI; }
float Euler::getPitchAngle() { return pitch * 180.0 / PI; }
float Euler::getYawAngle() { return yaw * 180.0 / PI; }

float Euler::getYawDegree() { 
  const float angle = getYawAngle();
  const float roll = getRollAngle();
  const float k = (roll > 0) ? 0 : 180;
  return k - angle;  
}

float Euler::getAngle(ImuUseAngle useAngle) {
  if (useAngle == IMU_USE_ROLL)
    return getRollAngle();
  if (useAngle == IMU_USE_PITCH)
    return getPitchAngle();  
  return getYawAngle();  
}

