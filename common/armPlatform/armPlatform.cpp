
#include "armPlatform.h"

void ArmPlatform::dispatchMessage(can2040_msg frame)
{
  if (frame.id == CAN_PLATFORM_QUATERNION) {
    imu.quaternion.deserialize(frame.data);
    updateTime();
    updateQuaternionTime();
  }    
  if (frame.id == CAN_PLATFORM_GYROSCOPE) {
    imu.gyroscope.deserialize(frame.data);
    updateTime();
  }    
  if (frame.id == CAN_PLATFORM_ACCELEROMETER) {
    imu.accelerometer.deserialize(frame.data);
    updateTime();
  }    
  if (frame.id == CAN_PLATFORM_ACCURACY) {
    imu.accuracy.deserialize(frame.data);
    updateTime();
  }    
  if (frame.id == CAN_PLATFORM_BAROMETER) {
    barometer.deserialize(frame.data);
    updateTime();
  }    
  if (frame.id == CAN_PLATFORM_CPU_POWER) {
    detectors.deserialize(frame.data);
    cpu.deserialize(&frame.data[4]);
    updateTime();
  }
  if (frame.id == CAN_PLATFORM_ENGINES_POWER) {
    engines.deserialize(frame.data);
    updateTime();
  }
  if (frame.id == CAN_PLATFORM_STATUS) {
    memcpy(&status, frame.data, sizeof(status));
    updateTime();
    updateEnginesPowerTime(status);    
  }
}

bool ArmPlatform::isOnline() {
  auto now = xTaskGetTickCount();
  return (now - lastUpdated < maxInterval);
}

bool ArmPlatform::isPositionOK() {
  if (!getEnginesPowerStatus()) return false;
  auto now = xTaskGetTickCount();
  return (now - lastQuaternionUpdated < maxInterval);
}

bool ArmPlatform::getEnginesPowerStatus() {
  if (!isOnline()) return false;  
  if ((status & PLATFORM_STATUS_ENGINES_ON) == 0) return false;
  auto diff = xTaskGetTickCount() - lastEnginesPowerOn;  
  return (diff > powerEnginesInterval);
}

bool ArmPlatform::getCameraPowerStatus() {
  if (!isOnline())
    return false;
  return (status & PLATFORM_STATUS_CAMERA_ON) != 0;
}

bool ArmPlatform::getDetectorsPowerStatus() {
  if (!isOnline())
    return false;
  return (status & PLATFORM_STATUS_DETECTORS_ON) != 0;
}

bool ArmPlatform::getCPUPowerStatus() {
  if (!isOnline())
    return false;
  return (status & PLATFORM_STATUS_CPU_POWER_ON) != 0;
}

void ArmPlatform::updateEnginesPowerTime(uint32_t status) {  
  if ((status & PLATFORM_STATUS_ENGINES_ON) != 0) {
    if (lastEnginesPowerOn == 0) {
      lastEnginesPowerOn = xTaskGetTickCount();    
    }
  } else {
    lastEnginesPowerOn = 0;
  }  
}