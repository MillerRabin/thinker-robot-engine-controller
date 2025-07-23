#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include <iostream>

#include "../config/config.h"
#include "../structureBase/structureBase.h"
#include "../euler/euler.h"

class Quaternion;

class IMUQuaternion : public StructureBase {
public:
  float i;
  float j;
  float k;
  float real;
  uint8_t Q1 = 14;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  Euler getEuler();
  bool fromBNO(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
  bool fromWitmotion(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal, float divisor);
  void multiplyFirst(const Quaternion &b);
};

class Quaternion {
public:
  float i;
  float j;
  float k;
  float real;
  Quaternion(float i = 0, float j = 0, float k = 0, float real = 1.0f)
      : i(i), j(j), k(k), real(real) {}
  Quaternion(IMUQuaternion &q);
  Quaternion operator*(const IMUQuaternion &q) const
  {
    Quaternion result;
    result.real = real * q.real - i * q.i - j * q.j - k * q.k;
    result.i = real * q.i + i * q.real + j * q.k - k * q.j;
    result.j = real * q.j - i * q.k + j * q.real + k * q.i;
    result.k = real * q.k + i * q.j - j * q.i + k * q.real;
    return result;
  }
  Quaternion operator=(const IMUQuaternion &q) const
  {
    Quaternion result;
    result.real = q.real;
    result.i = q.i;
    result.j = q.j;
    result.k = q.k;
    return result;
  }
  static Quaternion FromEuler(float roll, float pitch, float yaw);
  static Quaternion Conjugate(const Quaternion &q);
  static Quaternion Multiply(const Quaternion &a, const Quaternion &b);
};