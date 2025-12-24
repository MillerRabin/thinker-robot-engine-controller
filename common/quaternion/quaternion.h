#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include <iostream>

#include "../config/config.h"
#include "../qBase/qBase.h"
#include "../euler/euler.h"

class Quaternion;

struct Vector3 {
  double x;
  double y;
  double z;
};


struct Matrix3 {
  double m00, m01, m02;
  double m10, m11, m12;
  double m20, m21, m22;
};

class IMUQuaternion : public QBase {
public:
  double i;
  double j;
  double k;
  double real;
  uint8_t Q1 = 14;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);  
  bool fromBNO(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
  bool fromWitmotion(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal, double divisor);
  void multiplyFirst(const Quaternion &b);
  static IMUQuaternion FromQuaternion(Quaternion &quat);
};

  class Quaternion {
  public:
    double i;
    double j;
    double k;
    double real;
    Quaternion(double i = 0, double j = 0, double k = 0, double real = 1.0f)
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
    static Quaternion FromEuler(double roll, double pitch, double yaw);
    static Quaternion Conjugate(const Quaternion &q);
    static Quaternion Multiply(const Quaternion &a, const Quaternion &b);
    static Quaternion Normalize(const Quaternion &q);
    static Quaternion Difference(Quaternion &start, Quaternion &end);
    Matrix3 toRotationMatrix() const;
    Euler getEuler();
  };