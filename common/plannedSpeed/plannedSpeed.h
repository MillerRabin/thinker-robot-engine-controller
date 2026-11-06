#pragma once

#include "pico/stdlib.h"
#include <algorithm>
#include <math.h>
#include "../config/config.h"

// A parent joint's planned speed while it streams (Servo::streamTo()), for the child's
// Servo::setParentPlannedSpeed(). Fed the parent's waypoints as the host sends them on the bus, it runs
// the same setpoint the parent's servo runs - from the plan, never from a measurement. waypoint() is
// called from the CAN task and speed() from the engine task; single floats, a torn read costs one tick.
class PlannedSpeed {
  volatile bool active = false;
  volatile float setpoint = 0.0f;
  volatile float target = 0.0f;
  volatile float rate = 0.0f;
  volatile uint32_t lastUs = 0;
  volatile uint32_t reachUs = 0;

public:
  // value: the parent's waypoint (deg, command space); timeMS as sent. Anything that isn't a stream
  // waypoint (NaN, 0 or over STREAM_MAX_MS) ends the parent's stream.
  void waypoint(float value, uint16_t timeMS) {
    const uint32_t now = time_us_32();
    if (isnan(value) || timeMS == 0 || timeMS > STREAM_MAX_MS) {
      active = false;
      rate = 0.0f;
      return;
    }
    if (!active) {
      // The parent starts its setpoint where its link is, which isn't known here - a stream starts
      // with a short step, so take the first waypoint as the start.
      setpoint = value;
      active = true;
    } else if ((int32_t)(now - reachUs) < 0) {
      setpoint = setpoint + rate * (int32_t)(now - lastUs) / 1000000.0f;
    } else {
      setpoint = target;
    }
    const float seconds = std::max<uint16_t>(timeMS, 20) / 1000.0f;
    target = value;
    rate = std::clamp((value - setpoint) / seconds, -60.0f, 60.0f);
    lastUs = now;
    reachUs = now + (uint32_t)(seconds * 1000000.0f);
  }

  float speed() const {
    return active && (int32_t)(time_us_32() - reachUs) < 0 ? rate : 0.0f;
  }
};
