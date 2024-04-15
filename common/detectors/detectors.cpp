#include "detectors.h"


const float RangeMap::getDestValue(const float sourceValue) {
  const float zeroVal = sourceValue - source.from;  
  return dest.from + (step * zeroVal);
}

Euler::Euler(const float roll, const float pitch, const float yaw) : 
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

const float Euler::getRollAngle() { return roll * 180.0 / PI; }
const float Euler::getPitchAngle() { return pitch * 180.0 / PI; }
const float Euler::getYawAngle() { return yaw * 180.0 / PI; }

const float Euler::getAngle(ImuUseAngle useAngle) {
  if (useAngle == IMU_USE_ROLL)
    return getRollAngle();
  if (useAngle == IMU_USE_PITCH)
    return getPitchAngle();  
  return getYawAngle();  
}

float StructureBasic::qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return qFloat;
}

uint64_t Quaternion::serialize() {
  return (uint64_t)this->i |
         (uint64_t)this->j << 16 |
         (uint64_t)this->k << 32 |
         (uint64_t)this->real << 48;
}

void Quaternion::deserialize(uint8_t data[8]) {
  this->i = (uint16_t)data[1] << 8 | data[0];
  this->j = (uint16_t)data[3] << 8 | data[2];
  this->k = (uint16_t)data[5] << 8 | data[4];
  this->real = (uint16_t)data[7] << 8 | data[6];
}

void Quaternion::convertRawData() {
  this->i = qToFloat(rawI, Q1);
  this->j = qToFloat(rawJ, Q1);
  this->k = qToFloat(rawK, Q1);
  this->real = qToFloat(rawReal, Q1);
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


uint64_t Accelerometer::serialize() {
  return (uint64_t)this->x |
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

void Accelerometer::deserialize(uint8_t data[8]) {
  this->x = (uint16_t)data[1] << 8 | data[0];
  this->y = (uint16_t)data[3] << 8 | data[2];
  this->z = (uint16_t)data[5] << 8 | data[4];  
}

uint64_t Gyroscope::serialize() {
  return (uint64_t)this->x |
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

void Gyroscope::deserialize(uint8_t data[8]) {
  this->x = (uint16_t)data[1] << 8 | data[0];
  this->y = (uint16_t)data[3] << 8 | data[2];
  this->z = (uint16_t)data[5] << 8 | data[4];  
}
                  
uint64_t Accuracy::serialize() {
  return (uint64_t)this->quaternionAccuracy |
         (uint64_t)this->accelerometerAccuracy << 8 |
         (uint64_t)this->quaternionRadAccuracy << 16 |
         (uint64_t)this->gyroscopeAccuracy << 32;
}

void Accuracy::deserialize(uint8_t data[8]) {
  this->quaternionAccuracy = data[0];
  this->accelerometerAccuracy = data[1];
  this->quaternionRadAccuracy = (uint16_t)data[3] << 8 | data[2];
  this->gyroscopeAccuracy = data[4];  
}




