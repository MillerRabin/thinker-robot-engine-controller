#include "quaternion.h"

uint64_t Quaternion::serialize() const {
  uint64_t rawI = QBase::floatToQ(i, Q1);
  uint64_t rawJ = QBase::floatToQ(j, Q1);
  uint64_t rawK = QBase::floatToQ(k, Q1);
  uint64_t rawReal = QBase::floatToQ(real, Q1);
  return (uint64_t)rawI |
         (uint64_t)rawJ << 16 |
         (uint64_t)rawK << 32 |
         (uint64_t)rawReal << 48;
}

void Quaternion::deserialize(uint8_t data[8]) {
  uint16_t rawI = (uint16_t)data[1] << 8 | data[0];
  uint16_t rawJ = (uint16_t)data[3] << 8 | data[2];
  uint16_t rawK = (uint16_t)data[5] << 8 | data[4];
  uint16_t rawReal = (uint16_t)data[7] << 8 | data[6];
  this->i = QBase::qToFloat(rawI, Q1);
  this->j = QBase::qToFloat(rawJ, Q1);
  this->k = QBase::qToFloat(rawK, Q1);
  this->real = QBase::qToFloat(rawReal, Q1);
}

void Quaternion::fromBNO(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal) {
  this->i = QBase::qToFloat(rawQuatI, Q1);
  this->j = QBase::qToFloat(rawQuatJ, Q1);
  this->k = QBase::qToFloat(rawQuatK, Q1);
  this->real = QBase::qToFloat(rawQuatReal, Q1);  
}

void Quaternion::fromWitmotion(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal, float divisor) {
  this->i = rawQuatI / divisor;
  this->j = rawQuatJ / divisor;
  this->k = rawQuatK / divisor;
  this->real = rawQuatReal / divisor;  
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

Quaternion Quaternion::Difference(const Quaternion &start, const Quaternion &end) {
  Quaternion startNorm = Normalize(start);
  Quaternion endNorm = Normalize(end);
  Quaternion startInv = Conjugate(startNorm);
  Quaternion res = Multiply(startInv, endNorm);
  Quaternion qn = Normalize(res);

  if (qn.real < 0.0f) {
    qn.real = -qn.real;
    qn.i = -qn.i;
    qn.j = -qn.j;
    qn.k = -qn.k;
  }

  return qn;
}

Euler Quaternion::swingTwistToAngles() const {
  static constexpr Vector3 AXIS_X{1, 0, 0};
  static constexpr Vector3 AXIS_Y{0, 1, 0};
  static constexpr Vector3 AXIS_Z{0, 0, 1};

  const SwingTwist yawT = swingTwistDecomposition(AXIS_Z);
  auto pEuler = yawT.twist.getEuler();
  const float yaw = yawT.twist.twistAngle();

  const SwingTwist pitchT = swingTwistDecomposition(AXIS_Y);
  const float pitch = pitchT.twist.twistAngle();

  const SwingTwist rollT = swingTwistDecomposition(AXIS_X);
  const float roll = rollT.twist.twistAngle();

  return Euler(roll, pitch, yaw);
}

Quaternion Quaternion::Normalize(const Quaternion &q) {
  float norm = std::sqrt(q.real * q.real + q.i * q.i + q.j * q.j + q.k * q.k);
  return {
      q.real / norm,
      q.i / norm,
      q.j / norm,
      q.k / norm
  };
}

Euler Quaternion::getEuler() const {
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

Matrix3 Quaternion::toRotationMatrix() const {
  Matrix3 R;

  float w = real;
  float x = i;
  float y = j;
  float z = k;

  float xx = x * x;
  float yy = y * y;
  float zz = z * z;
  float xy = x * y;
  float xz = x * z;
  float yz = y * z;
  float wx = w * x;
  float wy = w * y;
  float wz = w * z;

  R.m00 = 1.0f - 2.0f * (yy + zz);
  R.m01 = 2.0f * (xy - wz);
  R.m02 = 2.0f * (xz + wy);

  R.m10 = 2.0f * (xy + wz);
  R.m11 = 1.0f - 2.0f * (xx + zz);
  R.m12 = 2.0f * (yz - wx);

  R.m20 = 2.0f * (xz - wy);
  R.m21 = 2.0f * (yz + wx);
  R.m22 = 1.0f - 2.0f * (xx + yy);

  return R;
}

SwingTwist Quaternion::swingTwistDecomposition(const Vector3 &axis) const {
  Vector3 v{this->i, this->j, this->k};  
  float dot = v.x * axis.x + v.y * axis.y + v.z * axis.z;
  Quaternion twist{ axis.x * dot, axis.y * dot, axis.z * dot, this->real };

  float len = std::hypot(std::hypot(twist.real, twist.i), std::hypot(twist.j, twist.k));

  twist.real /= len;
  twist.i /= len;
  twist.j /= len;
  twist.k /= len;

  Quaternion swing = (*this) * Conjugate(twist);
  return {swing, twist};
}

float Quaternion::twistAngle() const {
  float clamped = std::max(-1.0f, std::min(1.0f, real));
  float angle = 2.0f * std::acos(clamped);
  float sign = (i + j + k) >= 0 ? 1.0f : -1.0f;
  return angle * sign;
}

Quaternion Quaternion::Clone(const Quaternion &q) {
  Quaternion result;
  result.real = q.real;
  result.i = q.i;
  result.j = q.j;
  result.k = q.k;
  return result;
}