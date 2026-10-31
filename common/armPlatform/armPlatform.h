#pragma once

#include "pico/stdlib.h"
#include "../../common/bus/bus.h"
#include "../../common/remoteBNO/remoteBNO.h"
#include "../../common/config/config.h"
#include "../../common/barometer/barometer.h"
#include "../../common/power/power.h"

class ArmPart;

class ArmPlatform {
  uint64_t status;
  TickType_t lastUpdated = 0;
  TickType_t lastEnginesPowerOn = 0;
  Quaternion lastQuaternion;
  const TickType_t maxInterval = pdMS_TO_TICKS(500);
  const TickType_t powerEnginesInterval = pdMS_TO_TICKS(1000);
  // Debounce for the ON->OFF transition only - confirmed live (2026-08-25) that a single corrupted
  // CAN_PLATFORM_STATUS frame (misread ENGINES_ON bit, plausibly CAN noise from the nearby servo
  // PWM) reads as instantaneous "off" and was enough to trigger a full engine-loop reset/jerk even
  // though engines power never actually dropped. See getEnginesPowerStatus().
  TickType_t offSince = 0;
  bool lastConfirmedEnginesOn = false;
  const TickType_t offDebounceInterval = pdMS_TO_TICKS(200);
  // isEnginesOffConfirmed() state - see there.
  TickType_t explicitOffSince = 0;
  TickType_t offlineSince = 0;
  bool lastEnginesOffConfirmed = false;
  const TickType_t offlineReleaseInterval = pdMS_TO_TICKS(10000);
  SemaphoreHandle_t quaternionSemaphore;
  ArmPart* part;
public:
  RemoteBNO imu;
  Barometer barometer;
  Power engines;
  Power cpu;
  Power detectors;  
  void dispatchMessage(can2040_msg frame);
  void updateTime() { lastUpdated = xTaskGetTickCount(); }  
  void updateEnginesPowerTime(uint32_t status);
  bool isOnline();
  bool getEnginesPowerStatus();
  bool isEnginesOffConfirmed();
  bool enginesOffEdge();
  bool getCameraPowerStatus();
  bool getDetectorsPowerStatus();
  bool getCPUPowerStatus();
  bool isPositionOK();
};