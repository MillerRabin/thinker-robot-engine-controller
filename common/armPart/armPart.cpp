#include "armPart.h"

ArmPart::ArmPart(const uint canRxPin, const uint canTxPin) :
  bus(canRxPin, canTxPin, (void*)this, canCallback)
{}

int ArmPart::sendQuaternionInternal(uint32_t id, Quaternion quat) {  
  uint64_t data = quat.serialize();  
  return bus.send(id, data);
}

int ArmPart::sendGyroscopeInternal(uint32_t id, Gyroscope gyro) {  
  uint64_t data = gyro.serialize();  
  return bus.send(id, data);
}

int ArmPart::sendAccelerometerInternal(uint32_t id, Accelerometer acc) {  
  uint64_t data = acc.serialize();  
  return bus.send(id, data);
}

int ArmPart::sendAccuracyInternal(uint32_t id, Accuracy acc) {
  uint64_t data = acc.serialize();  
  return bus.send(id, data);
}

void ArmPart::canCallback(void* pArmPart, can2040_msg frame) {
  uint32_t ident = frame.id;
  ArmPart* armPart = (ArmPart*)pArmPart;
  armPart->busReceiveCallback(frame);  
}