#include "imuBase.h"

SemaphoreHandle_t IMUBase::accelerometerMutex;

IMUBase::IMUBase() {
  IMUBase::accelerometerMutex = xSemaphoreCreateMutex();
}

Accelerometer IMUBase::getAccelerometer() {
  if (xSemaphoreTake(IMUBase::accelerometerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    lastAccelerometer = accelerometer;
    xSemaphoreGive(IMUBase::accelerometerMutex);
  }
  return lastAccelerometer;
}

void IMUBase::setAccelerometer(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ) {
  if (xSemaphoreTake(IMUBase::accelerometerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    accelerometer.x = rawAccX;
    accelerometer.y = rawAccY;
    accelerometer.z = rawAccZ;    
    xSemaphoreGive(IMUBase::accelerometerMutex);
  }
}
