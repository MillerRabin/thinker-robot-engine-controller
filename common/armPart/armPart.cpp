#include "armPart.h"

uint64_t Quaternion::serializeData() {  
  return (uint64_t)this->i | 
         (uint64_t)this->j << 16 |
         (uint64_t)this->k << 32 |
         (uint64_t)this->real << 48;
}

uint64_t Accelerometer::serializeData() {  
  return (uint64_t)this->x | 
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

uint64_t Gyroscope::serializeData() {  
  return (uint64_t)this->x | 
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

uint64_t Accuracy::serializeData() {  
  return (uint64_t)this->quaternionAccuracy | 
         (uint64_t)this->accelerometerAccuracy << 8 | 
         (uint64_t)this->quaternionRadAccuracy << 16 |
         (uint64_t)this->gyroscopeAccuracy << 32;
}

ArmPart::ArmPart(const uint canRxPin, const uint canTxPin) {
  Bus bus = Bus(canRxPin, canTxPin);  
}

int ArmPart::sendQuaternionInternal(uint32_t id, Quaternion quat) {  
  uint64_t data = quat.serializeData();  
  return bus.send(id, data);
}

int ArmPart::sendGyroscopeInternal(uint32_t id, Gyroscope gyro) {  
  uint64_t data = gyro.serializeData();  
  return bus.send(id, data);
}

int ArmPart::sendAccelerometerInternal(uint32_t id, Accelerometer acc) {  
  uint64_t data = acc.serializeData();  
  return bus.send(id, data);
}

int ArmPart::sendAccuracyInternal(uint32_t id, Accuracy acc) {
  uint64_t data = acc.serializeData();  
  return bus.send(id, data);
}
