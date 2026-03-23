#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include <iostream>

#include "../config/config.h"
#include "../qBase/qBase.h"
#include "../euler/euler.h"

class Quaternion;
struct SwingTwist;

struct Vector3 {
  float x;
  float y;
  float z;
};


struct Matrix3 {
  float m00, m01, m02;
  float m10, m11, m12;
  float m20, m21, m22;
};

class Quaternion {
  public:
    float i;
    float j;
    float k;
    float real;
    uint8_t Q1 = 14;
    Quaternion(float i = NAN, float j = NAN, float k = NAN, float real = NAN)
        : i(i), j(j), k(k), real(real) {};
    uint64_t serialize() const;
    void deserialize(uint8_t data[8]);
    void fromBNO(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
    void fromWitmotion(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal, float divisor);
  
    Quaternion operator * (const Quaternion &q) const {
      return Multiply(*this, q);
    }    
    
    SwingTwist swingTwistDecomposition(const Vector3 &axis) const;
    Euler swingTwistToAngles() const;
    float twistAngle() const;
    Quaternion invert() const {
      return Conjugate(*this);
    }
    static Quaternion FromEuler(float roll, float pitch, float yaw);
    static Quaternion Conjugate(const Quaternion &q);
    static Quaternion Multiply(const Quaternion &a, const Quaternion &b);
    static Quaternion Normalize(const Quaternion &q);
    static Quaternion Difference(const Quaternion &start, const Quaternion &end);
    static Quaternion Clone(const Quaternion &q);
    Matrix3 toRotationMatrix() const;    
    Euler getEuler() const;
    bool isZero() const {
      return (i == 0) && (j == 0) && (k == 0) && (real == 0);
    }
    bool isValid() const {
      return !isnan(i) && !isnan(j) && !isnan(k) && !isnan(real);
    }
};

struct SwingTwist {
  Quaternion swing;
  Quaternion twist;
};
