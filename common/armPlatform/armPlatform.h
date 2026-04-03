#pragma once

#include "pico/stdlib.h"
#include "../../common/bus/bus.h"
#include "../../common/remoteBNO/remoteBNO.h"
#include "../../common/config/config.h"
#include "../../common/barometer/barometer.h"
#include "../../common/power/power.h"

class ArmPlatform {
  uint64_t status;
  TickType_t lastUpdated = 0;
  TickType_t lastQuaternionUpdated = 0;
  TickType_t lastEnginesPowerOn = 0;
  const TickType_t maxInterval = pdMS_TO_TICKS(500);
  const TickType_t powerEnginesInterval = pdMS_TO_TICKS(500);

public:
  RemoteBNO imu;
  Barometer barometer;
  Power engines;
  Power cpu;
  Power detectors;
  void dispatchMessage(can2040_msg frame);
  void updateTime() { lastUpdated = xTaskGetTickCount(); }
  void updateQuaternionTime() { lastQuaternionUpdated = xTaskGetTickCount(); }
  void updateEnginesPowerTime(uint32_t status);
  bool isOnline();
  bool getEnginesPowerStatus();
  bool getCameraPowerStatus();
  bool getDetectorsPowerStatus();
  bool getCPUPowerStatus();
  bool isPositionOK();
};