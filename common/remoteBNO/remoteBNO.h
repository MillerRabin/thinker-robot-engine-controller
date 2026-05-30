#pragma once

#include "../accelerometer/accelerometer.h"
#include "../gyroscope/gyroscope.h"
#include "../quaternion/quaternion.h"
#include "../accuracy/accuracy.h"

class RemoteBNO {
  public:
    AtomicQuaternion quaternion;
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
};