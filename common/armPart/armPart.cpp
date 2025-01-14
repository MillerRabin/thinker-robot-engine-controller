#include "armPart.h"

ArmPart::ArmPart(const uint canRxPin, const uint canTxPin) :
  bus(canRxPin, canTxPin, (void*)this, canCallback)
{}

int ArmPart::updateQuaternion(Quaternion quat) {  
  uint64_t data = quat.serialize();  
  uint8_t id = getQuaternionMessageId();  
  if (id == 0) return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateGyroscope(Gyroscope gyro) {  
  uint64_t data = gyro.serialize();
  uint8_t id = getGyroscopeMessageId();
  if (id == 0) return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateAccelerometer(Accelerometer acc) {  
  uint64_t data = acc.serialize();
  uint8_t id = getAccelerometerMessageId();
  if (id == 0) return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateAccuracy(Accuracy acc) {
  uint64_t data = acc.serialize();
  uint8_t id = getAccuracyMessageId();
  if (id == 0) return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateHeight(uint32_t height) {
  uint64_t data = (uint64_t)height;
  uint8_t id = getHeightMessageId();
  if (id == 0) return -1;
  bus.send(id, data);
  return 0;
}

void ArmPart::canCallback(void* pArmPart, can2040_msg frame) {
  uint32_t ident = frame.id;
  ArmPart* armPart = (ArmPart*)pArmPart;
  armPart->platform.dispatchMessage(frame);
  armPart->busReceiveCallback(frame);  
}