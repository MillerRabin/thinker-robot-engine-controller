#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"
#include <iostream>
#include "../qBase/qBase.h"

class Gyroscope : public QBase {
public:
  float x;
  float y;
  float z;
  int16_t Q1 = 9;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  void set(float x, float y, float z);
  void fromWitmotion(int16_t rawGyroX, int16_t rawGyroY, int16_t rawGyroZ);
  void fromBNO(int16_t rawGyroX, int16_t rawGyroY, int16_t rawGyroZ);
};