#pragma once

#include "pico/stdlib.h"
#include "../../common/bus/bus.h"
#include "../../common/remoteBNO/remoteBNO.h"
#include "../../common/config/config.h"

class RemoteShoulder {
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
  void dispatchMessage(can2040_msg frame);
  // Its command speed while it settles a stream (deg/s), 0 when not or when stale - see
  // SETTLE_SPEED_STALE_MS.
  float getSettleSpeed() const;
private:
  volatile float settleSpeed = 0.0f;
  volatile uint32_t settleSpeedAtUs = 0;
public:
};