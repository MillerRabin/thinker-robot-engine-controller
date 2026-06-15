#pragma once
#include "../config/config.h"
#include "../overwriteRingBuffer/overwriteRingBuffer.h"
#include "pico/stdlib.h"
#include "math.h"

struct AngleSample {
  float theta;
  uint32_t timestamp_us;
};

class SpeedBuffer {
private:
  OverwriteRingBuffer<AngleSample, SPEED_BUFFER_SIZE> buffer;
  float cachedSpeed = 0.0f;
  float cachedAcceleration = 0.0f;

  void recalculate() {
    if (buffer.count() < 2) {
      cachedSpeed = 0.0f;
      cachedAcceleration = 0.0f;
      return;
    }

    AngleSample newest, oldest;
    buffer.peek(buffer.count() - 1, &newest);
    buffer.peek(0, &oldest);

    float dt = (newest.timestamp_us - oldest.timestamp_us) / 1e6f;
    if (dt < 0.01f) {
      cachedSpeed = 0.0f;
      cachedAcceleration = 0.0f;
      return;
    }

    cachedSpeed = (newest.theta - oldest.theta) / dt;
    //cachedSpeed = fmaxf(fminf(cachedSpeed, 180.0f), -180.0f);
    cachedAcceleration = 0.0f;
  }

public:
  static constexpr float ANGLE_DEAD_BAND = 0.3f;
  static constexpr uint32_t STOP_TIMEOUT_US = 50000;
  void push(float theta) {
    uint32_t now = (uint32_t)to_us_since_boot(get_absolute_time());

    auto count = buffer.count();
    if (count > 0) {
      AngleSample last;
      buffer.peek(count - 1, &last);

      if (fabsf(theta - last.theta) < ANGLE_DEAD_BAND) {
        if (now - last.timestamp_us > STOP_TIMEOUT_US) {          
          buffer.clear();
          cachedSpeed = 0.0f;
          cachedAcceleration = 0.0f;
        }
        return;
      }
    }

    AngleSample s;
    s.theta = theta;
    s.timestamp_us = now;
    buffer.push(s);
    recalculate();
  }
  
  inline void reset() {    
    buffer.clear();    
    cachedSpeed = 0.0f;
    cachedAcceleration = 0.0f;
  }

  inline float speed() const { return cachedSpeed; }
  inline float acceleration() const { return cachedAcceleration; }
};