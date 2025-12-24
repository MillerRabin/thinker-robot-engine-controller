#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"
#include <iostream>
#include "../qBase/qBase.h"

class Accelerometer : public QBase {
public:
  int16_t x;
  int16_t y;
  int16_t z;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  float witMotionX() { return x / 32768.0f * 16.0f * 9.80665f; };
  float witMotionY() { return y / 32768.0f * 16.0f * 9.80665f; };
  float witMotionZ() { return z / 32768.0f * 16.0f * 9.80665f; };
};
