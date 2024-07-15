#include "quaternion.h"

uint64_t Quaternion::serialize() {
  return (uint64_t)this->rawI |
         (uint64_t)this->rawJ << 16 |
         (uint64_t)this->rawK << 32 |
         (uint64_t)this->rawReal << 48;
}

void Quaternion::deserialize(uint8_t data[8]) {
  this->rawI = (uint16_t)data[1] << 8 | data[0];
  this->rawJ = (uint16_t)data[3] << 8 | data[2];
  this->rawK = (uint16_t)data[5] << 8 | data[4];
  this->rawReal = (uint16_t)data[7] << 8 | data[6];
}

void Quaternion::convertRawData() {
  this->i = qToFloat(rawI, Q1);
  this->j = qToFloat(rawJ, Q1);
  this->k = qToFloat(rawK, Q1);
  this->real = qToFloat(rawReal, Q1);
  //applyRotate();
}

bool Quaternion::set(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal) {
  uint16_t id = (rawQuatI > rawI) ? rawQuatI - rawI : rawI - rawQuatI;
  uint16_t jd = (rawQuatJ > rawJ) ? rawQuatJ - rawJ : rawJ - rawQuatJ;
  uint16_t kd = (rawQuatK > rawK) ? rawQuatK - rawK : rawK - rawQuatK;
  uint16_t rd = (rawQuatReal >rawReal) ? rawQuatReal - rawReal : rawReal - rawQuatReal;
  this->rawI = rawQuatI;
  this->rawJ = rawQuatJ;
  this->rawK = rawQuatK;
  this->rawReal = rawQuatReal;
  return ((id > 1) || (jd > 1) || (kd > 1) || (rd > 1));
}

void Quaternion::setRotate(float i, float j, float k, float real) {
  this->rotateI = i;
  this->rotateJ = j;
  this->rotateK = k;
  this->rotateReal = real; 
}

Euler Quaternion::getEuler() {
  convertRawData();  
  float dqw = real;
	float dqx = i;
	float dqy = j;
	float dqz = k;

	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;
	
	float t3 = +2.0 * (dqw * dqz + dqx * dqy);
	float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
	float yaw = atan2(t3, t4);

	float t2 = +2.0 * (dqw * dqy - dqz * dqx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	float pitch = asin(t2);
  	
	float t0 = +2.0 * (dqw * dqx + dqy * dqz);
	float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
	float roll = atan2(t0, t1);
	  
  return Euler(roll, pitch, yaw);
}

void Quaternion::applyRotate() {
  const float sreal = this->real * this->rotateReal - this->i * this->rotateI - this->j * j - this->k * this->rotateK;
  const float si = this->real * this->rotateI + this->i * this->rotateReal + this->j * this->rotateK - this->k * this->rotateJ;
  const float sj = this->real * this->rotateJ - this->i * k + this->j * this->rotateReal + this->k * this->rotateI;
  const float sk = this->real * this->rotateK + this->i * this->rotateJ - this->j * this->rotateI + this->k * this->rotateReal;
  this->real = sreal;
  this->i = si;
  this->j = sj;
  this->k = sk;
}