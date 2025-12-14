#pragma once

#include "../detectors/detectors.h"
#include "../measureRange/measureRange.h"
#include "../quaternion/quaternion.h"

class IMUBase {
  public:
    IMUQuaternion quaternion;
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
    uint16_t height;
    uint16_t pressure;
    uint8_t temperature;
};