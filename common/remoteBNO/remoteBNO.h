#pragma once

#include "../accelerometer/accelerometer.h"
#include "../gyroscope/gyroscope.h"
#include "../quaternion/quaternion.h"
#include "../accuracy/accuracy.h"

class RemoteBNO {
  public:
    Quaternion quaternion{0, 0, 0, 0};
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
};