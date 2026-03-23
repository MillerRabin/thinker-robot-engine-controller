#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"
#include <iostream>
#include "../qBase/qBase.h"

class Accelerometer : public QBase {
public:
  float x;
  float y;
  float z;
  int16_t Q1 = 8;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  void fromWitmotion(int16_t rawAccX, int16_t rawAccY, int16_t rawAccZ);
  void fromBNO(int16_t rawAccX, int16_t rawAccY, int16_t rawAccZ);
};
