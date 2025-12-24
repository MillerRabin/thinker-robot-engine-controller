#pragma once

#include "../detectors/detectors.h"
#include "../measureRange/measureRange.h"
#include "../quaternion/quaternion.h"
#include "../accelerometer/accelerometer.h"

class IMUBase {
  public:
    IMUQuaternion quaternion;
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
    uint32_t height;    
    uint16_t temperature;
};