#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"
#include <iostream>
#include "../qBase/qBase.h"

class Gyroscope : public QBase {
public:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  float witMotionX() { return x / 32768.0f * 2000.0f; };
  float witMotionY() { return y / 32768.0f * 2000.0f; };
  float witMotionZ() { return z / 32768.0f * 2000.0f; };
};
