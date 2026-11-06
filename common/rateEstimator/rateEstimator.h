#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include <math.h>
#include "../logQueue/logQueue.h"

// A link's angular rate from its IMU angle, for Servo::setLinkRate() (active damping). Differenced
// at each new IMU sample, stamped with its store time, not at each tick: the IMU and the engine loop
// both run at 5 ms, unsynchronised, so per-tick differences alternate between 0 and twice the step.
// Low-passed (tauSec). NaN once the IMU has gone quiet - no rate, no damping.
class RateEstimator {
  const float tauSec;
  TickType_t sampleTick = 0;
  bool valid = false;
  float lastDeg = 0.0f;
  float rate = 0.0f;

public:
  explicit RateEstimator(float tauSec) : tauSec(tauSec) {}

  // angleDeg: the IMU angle now (unwrapped); sampleAge: how old the IMU sample it came from is
  // (AtomicValue::age()).
  float update(float angleDeg, TickType_t sampleAge) {
    const TickType_t now = xTaskGetTickCount();
    const TickType_t tick = now - sampleAge;
    if (!valid || tick != sampleTick) {
      const float dt = (int32_t)(tick - sampleTick) / (float)configTICK_RATE_HZ;
      if (valid && dt > 0.0f && dt < 0.05f) {
        rate += dt / (tauSec + dt) * ((angleDeg - lastDeg) / dt - rate);
      } else {
        rate = 0.0f;
      }
      sampleTick = tick;
      lastDeg = angleDeg;
      valid = true;
    }
    if ((int32_t)(now - sampleTick) > (int32_t)pdMS_TO_TICKS(30)) {
      return NAN;
    }
    return rate;
  }
};

// Peaks of a damped joint's rate and pulse offset: one [DAMP] log line per 500 ms while it moves.
class DampingMonitor {
  const char *name;
  float ratePeak = 0.0f;
  float offsetPeak = 0.0f;
  absolute_time_t windowStart = 0;

public:
  explicit DampingMonitor(const char *name) : name(name) {}

  void update(float rate, float offset) {
    if (!isnan(rate)) {
      ratePeak = fmaxf(ratePeak, fabsf(rate));
    }
    offsetPeak = fmaxf(offsetPeak, fabsf(offset));
    const absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(windowStart, now) >= 500000) {
      if (ratePeak > 2.0f) {
        LogQueue::Log("[DAMP] %s rate peak %.1f deg/s, offset peak %.2f deg\n", name, ratePeak, offsetPeak);
      }
      ratePeak = 0.0f;
      offsetPeak = 0.0f;
      windowStart = now;
    }
  }
};
