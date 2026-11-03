#pragma once

#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/remoteBNO/remoteBNO.h"
#include <atomic>
#include "pico/stdlib.h"

class RemoteWrist {
private:
  TickType_t lastUpdated = 0;  
  const TickType_t maxInterval = pdMS_TO_TICKS(500);
  void updateTime() { lastUpdated = xTaskGetTickCount(); }  
public:
  std::atomic_ulong status;
  bool isCalibrated();  
  bool isPositionOK();
  bool isOnline();  
  RemoteBNO imu;
  // For diagnosing a lost wrist: isPositionOK() needs both within maxInterval.
  TickType_t statusAge() const { return xTaskGetTickCount() - lastUpdated; }
  TickType_t quaternionAge() const { return imu.quaternion.age(); }
  void dispatchMessage(can2040_msg frame);
};