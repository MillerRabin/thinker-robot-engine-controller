#include "quaternion.h"

uint64_t IMUQuaternion::serialize() {
  uint64_t rawI = floatToQ(i, Q1);
  uint64_t rawJ = floatToQ(j, Q1);
  uint64_t rawK = floatToQ(k, Q1);
  uint64_t rawReal = floatToQ(real, Q1);
  return (uint64_t)rawI |
         (uint64_t)rawJ << 16 |
         (uint64_t)rawK << 32 |
         (uint64_t)rawReal << 48;
}

bool IMUQuaternion::fromBNO(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal) {
  this->i = qToFloat(rawQuatI, Q1);
  this->j = qToFloat(rawQuatJ, Q1);
  this->k = qToFloat(rawQuatK, Q1);
  this->real = qToFloat(rawQuatReal, Q1);
  return true;
}

bool IMUQuaternion::fromWitmotion(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal, float divisor) {
  this->i = rawQuatI / divisor;
  this->j = rawQuatJ / divisor;
  this->k = rawQuatK / divisor;
  this->real = rawQuatReal / divisor;
  return true;
}

IMUQuaternion IMUQuaternion::FromQuaternion(Quaternion &quat) {
  IMUQuaternion imuQuat;
  imuQuat.i = quat.i;
  imuQuat.j = quat.j;
  imuQuat.k = quat.k;
  imuQuat.real = quat.real;
  return imuQuat;
}

void IMUQuaternion::deserialize(uint8_t data[8]) {
  uint16_t rawI = (uint16_t)data[1] << 8 | data[0];
  uint16_t rawJ = (uint16_t)data[3] << 8 | data[2];
  uint16_t rawK = (uint16_t)data[5] << 8 | data[4];
  uint16_t rawReal = (uint16_t)data[7] << 8 | data[6];
  this->i = qToFloat(rawI, Q1);
  this->j = qToFloat(rawJ, Q1);
  this->k = qToFloat(rawK, Q1);
  this->real = qToFloat(rawReal, Q1);
}

void IMUQuaternion::multiplyFirst(const Quaternion &b) {
  float r = real;
  float x = i;
  float y = j;
  float z = k;

  real = b.real * r - b.i * x - b.j * y - b.k * z;
  i = b.real * x + b.i * r + b.j * z - b.k * y;
  j = b.real * y - b.i * z + b.j * r + b.k * x;
  k = b.real * z + b.i * y - b.j * x + b.k * r;
}

Quaternion::Quaternion(IMUQuaternion &q) {
  i = q.i;
  j = q.j;
  k = q.k;
  real = q.real;
}

Quaternion Quaternion::FromEuler(float roll, float pitch, float yaw) {
  float cr = cos(roll / 2.0f);
  float sr = sin(roll / 2.0f);
  float cp = cos(pitch / 2.0f);
  float sp = sin(pitch / 2.0f);
  float cy = cos(yaw / 2.0f);
  float sy = sin(yaw / 2.0f);

  Quaternion q;
  q.real = cr * cp * cy + sr * sp * sy;
  q.i = sr * cp * cy - cr * sp * sy;
  q.j = cr * sp * cy + sr * cp * sy;
  q.k = cr * cp * sy - sr * sp * cy;

  return q;
}

Quaternion Quaternion::Conjugate(const Quaternion &q) {
  return {-q.i, -q.j, -q.k, q.real};
}

Quaternion Quaternion::Multiply(const Quaternion &a, const Quaternion &b) {
  Quaternion q;
  q.real = a.real * b.real - a.i * b.i - a.j * b.j - a.k * b.k;
  q.i = a.real * b.i + a.i * b.real + a.j * b.k - a.k * b.j;
  q.j = a.real * b.j - a.i * b.k + a.j * b.real + a.k * b.i;
  q.k = a.real * b.k + a.i * b.j - a.j * b.i + a.k * b.real;
  return q;
}

Quaternion Quaternion::Difference(Quaternion &start, Quaternion &end) {
  Quaternion startNorm = Normalize(start);
  Quaternion endNorm = Normalize(end);
  Quaternion startInv = Conjugate(startNorm);
  Quaternion res = Multiply(endNorm, startInv);
  return Normalize(res);
}

Quaternion Quaternion::Normalize(Quaternion &q) {
  float norm = std::sqrt(q.real * q.real + q.i * q.i + q.j * q.j + q.k * q.k);
  return {
      q.real / norm,
      q.i / norm,
      q.j / norm,
      q.k / norm};
}

Euler Quaternion::getEuler() {
  float dqw = real;
  float dqx = i;
  float dqy = j;
  float dqz = k;

  float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
  if (norm < 1e-6f)
    return Euler(0.0f, 0.0f, 0.0f);
  dqw /= norm;
  dqx /= norm;
  dqy /= norm;
  dqz /= norm;

  float ysqr = dqy * dqy;

  // Yaw (Z)
  float t3 = +2.0f * (dqw * dqz + dqx * dqy);
  float t4 = +1.0f - 2.0f * (ysqr + dqz * dqz);
  float yaw = atan2(t3, t4);

  // Pitch (Y)
  float t2 = +2.0f * (dqw * dqy - dqz * dqx);
  t2 = t2 > 1.0f ? 1.0f : t2;
  t2 = t2 < -1.0f ? -1.0f : t2;
  float pitch = asin(t2);

  // Roll (X)
  float t0 = +2.0f * (dqw * dqx + dqy * dqz);
  float t1 = +1.0f - 2.0f * (dqx * dqx + ysqr);
  float roll = atan2(t0, t1);
  return Euler(roll, pitch, yaw);
}
