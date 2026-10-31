
#include "armPlatform.h"
#include "../armPart/armPart.h"

void ArmPlatform::dispatchMessage(can2040_msg frame) {
  if (frame.id == CAN_PLATFORM_QUATERNION) {    
    Quaternion quat;
    quat.deserialize(frame.data);
    imu.quaternion.store(quat);
    updateTime();
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
  return imu.quaternion.isFresh();
}

bool ArmPlatform::getEnginesPowerStatus() {
  bool rawOn = isOnline() && ((status & PLATFORM_STATUS_ENGINES_ON) != 0);

  if (rawOn) {
    offSince = 0;
    auto diff = xTaskGetTickCount() - lastEnginesPowerOn;
    lastConfirmedEnginesOn = (diff > powerEnginesInterval);
    return lastConfirmedEnginesOn;
  }

  // A single bad reading (stale CAN / a misread status bit) isn't trusted as a real power-off until
  // it persists past offDebounceInterval - bridges over a brief glitch by holding the last confirmed
  // value instead. Never defaults to "on": if engines were never confirmed on in the first place,
  // lastConfirmedEnginesOn is already false here.
  if (offSince == 0) {
    offSince = xTaskGetTickCount();
  }
  if (xTaskGetTickCount() - offSince < offDebounceInterval) {
    return lastConfirmedEnginesOn;
  }
  lastConfirmedEnginesOn = false;
  return false;
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