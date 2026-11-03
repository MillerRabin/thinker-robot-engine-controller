
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
  // Signed: the CAN task may stamp lastUpdated after our tick read - see Bus::isBusAlive().
  return (int32_t)(now - lastUpdated) < (int32_t)maxInterval;
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

// Stricter than !getEnginesPowerStatus(), which also goes false when platform frames merely stop
// arriving: confirmed live (2026-10-31) that they do stop, for several seconds at a time, with engines
// still on - every module then released its servos and the elbow hung limp, swinging, until status
// came back. Only this may release (go limp): the host saying "off" (debounced), or a silence long
// enough that power really may have dropped (a host reboot) - then a stale PWM level must not wait
// for the power to return (Servo::release()'s comment).
bool ArmPlatform::isEnginesOffConfirmed() {
  TickType_t now = xTaskGetTickCount();
  if (isOnline()) {
    offlineSince = 0;
    if ((status & PLATFORM_STATUS_ENGINES_ON) != 0) {
      explicitOffSince = 0;
      return false;
    }
    if (explicitOffSince == 0) {
      explicitOffSince = now;
    }
    return now - explicitOffSince >= offDebounceInterval;
  }
  explicitOffSince = 0;
  if (offlineSince == 0) {
    offlineSince = now;
  }
  return now - offlineSince >= offlineReleaseInterval;
}

// True once per transition into isEnginesOffConfirmed() - each module releases its servos on it.
bool ArmPlatform::enginesOffEdge() {
  bool confirmed = isEnginesOffConfirmed();
  bool edge = confirmed && !lastEnginesOffConfirmed;
  lastEnginesOffConfirmed = confirmed;
  return edge;
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