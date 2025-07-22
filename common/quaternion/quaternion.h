#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include <iostream>

#include "../config/config.h"
#include "../structureBase/structureBase.h"
#include "../euler/euler.h"

class Quaternion : public StructureBase {
private:
  void convertRawData();
  void applyRotate();
public:
  uint16_t rawI;
  uint16_t rawJ;
  uint16_t rawK;
  uint16_t rawReal;
  float i;
  float j;
  float k;
  float real;
  float rotateI = 0;
  float rotateJ = 0;
  float rotateK = 0;
  float rotateReal = 1;
  uint8_t Q1 = 14;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  Euler getEuler();
  bool set(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
  void setRotate(float i, float j, float k, float real);
  
};