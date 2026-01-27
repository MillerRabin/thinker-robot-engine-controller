#pragma once


#include <FreeRTOS.h>
#include <semphr.h>

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
    Accelerometer lastAccelerometer;
    uint32_t height;    
    uint16_t temperature;
    Accelerometer getAccelerometer();
    void setAccelerometer(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ);
    static SemaphoreHandle_t accelerometerMutex;
    IMUBase();
};