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
  uint16_t count = 0;
  float cachedSpeed = 0.0f;
  float cachedAcceleration = 0.0f;

  /*void recalculate() {
    if (count < 2) {
      cachedSpeed = 0.0f;
      cachedAcceleration = 0.0f;
      return;
    }

    // Берём реальное время из сэмплов
    AngleSample newest, oldest;
    buffer.peek(count - 1, &newest);
    buffer.peek(0, &oldest);

    float totalTime = (newest.timestamp_us - oldest.timestamp_us) / 1e6f;
    if (totalTime < 1e-6f) {
      cachedSpeed = 0.0f;
      cachedAcceleration = 0.0f;
      return;
    }

    float offset = totalTime / 2.0f;
    float s_t2 = 0, s_t4 = 0;
    float s_θ = 0, s_tθ = 0, s_t2θ = 0;
    uint16_t N = count;

    for (uint16_t i = 0; i < N; i++) {
      AngleSample s;
      buffer.peek(i, &s);
      float t = (s.timestamp_us - oldest.timestamp_us) / 1e6f - offset;
      float t2 = t * t;
      s_t2 += t2;
      s_t4 += t2 * t2;
      s_θ += s.theta;
      s_tθ += t * s.theta;
      s_t2θ += t2 * s.theta;
    }

    cachedSpeed = s_tθ / s_t2;

    if (count >= 3) {
      float c = (N * s_t2θ - s_t2 * s_θ) / (N * s_t4 - s_t2 * s_t2);
      cachedAcceleration = 2.0f * c;
    } else {
      cachedAcceleration = 0.0f;
    }
  }*/

  void recalculate() {
    if (count < 2) {
      cachedSpeed = 0.0f;
      cachedAcceleration = 0.0f;
      return;
    }

    AngleSample newest, oldest;
    buffer.peek(count - 1, &newest);
    buffer.peek(0, &oldest);

    float dt = (newest.timestamp_us - oldest.timestamp_us) / 1e6f;
    if (dt < 0.01f) { // меньше 10 мс — не считаем
      cachedSpeed = 0.0f;
      cachedAcceleration = 0.0f;
      return;
    }

    cachedSpeed = (newest.theta - oldest.theta) / dt;

    // физическое ограничение
    cachedSpeed = fmaxf(fminf(cachedSpeed, 180.0f), -180.0f);

    cachedAcceleration = 0.0f;
  }

public:
  static constexpr float ANGLE_DEAD_BAND = 0.3f;
  static constexpr uint32_t STOP_TIMEOUT_US = 50000;
  void push(float theta) {
    uint32_t now = (uint32_t)to_us_since_boot(get_absolute_time());

    if (count > 0) {
      AngleSample last;
      buffer.peek(count - 1, &last);

      if (fabsf(theta - last.theta) < ANGLE_DEAD_BAND) {
        if (now - last.timestamp_us > STOP_TIMEOUT_US) {
          buffer = OverwriteRingBuffer<AngleSample, SPEED_BUFFER_SIZE>();
          count = 0;
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
    if (count < SPEED_BUFFER_SIZE)
      count++;
    recalculate();
  }

  inline float speed() const { return cachedSpeed; }
  inline float acceleration() const { return cachedAcceleration; }
};