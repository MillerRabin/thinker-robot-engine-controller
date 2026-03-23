#pragma once


#include <FreeRTOS.h>
#include <semphr.h>

#include "../measureRange/measureRange.h"
#include "../quaternion/quaternion.h"
#include "../accelerometer/accelerometer.h"
#include "../gyroscope/gyroscope.h"
#include "../accuracy/accuracy.h"

class IMUBase {
  public:
    Quaternion quaternion;
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
    Accelerometer lastAccelerometer;
    uint32_t height;    
    uint16_t temperature;
};