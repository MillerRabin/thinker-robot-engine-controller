#pragma once

#include "../detectors/detectors.h"
#include "../quaternion/quaternion.h"
#include "../accelerometer/accelerometer.h"

class RemoteBNO {
  private:
    int16_t rotationVector_Q1 = 14;
  public:
    IMUQuaternion quaternion;
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
};