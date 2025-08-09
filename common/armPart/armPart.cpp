#include "armPart.h"

ArmPart::ArmPart(const uint canRxPin, const uint canTxPin) :
  bus(canRxPin, canTxPin, this, canCallback)
{}

int ArmPart::updateQuaternion(IMUQuaternion quat) {  
  Quaternion alignedQuat = align(quat);
  IMUQuaternion imuQuat = IMUQuaternion::FromQuaternion(alignedQuat);
  uint64_t data = imuQuat.serialize();
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

int ArmPart::updateStatuses() {  
  uint8_t id = getStatusesMessageId();
  if (id == 0)
    return -1;
  bus.send(id, statuses);
  statuses = 0;
  return 0;
}

int ArmPart::updateRange(uint16_t range, uint16_t measureType) {
  MeasureRange mRange;  
  mRange.set(range, measureType);  
  uint64_t data = mRange.serialize();
  uint8_t id = getRangeMessageId();
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

void ArmPart::setPositionTaskStatus(bool value) {
  if (value) {
    statuses |= ARM_POSITION_TASK_OK;
  } else {
    statuses &= ~ARM_POSITION_TASK_OK;
  }
}

void ArmPart::setEngineTaskStatus(bool value) {
  if (value) {
    statuses |= ARM_ENGINE_TASK_OK;
  } else {
    statuses &= ~ARM_ENGINE_TASK_OK;
  }
}

void ArmPart::setBusReceivingTaskStatus(bool value) {
  if (value) {
    statuses |= BUS_RECEIVING_TASK_OK;
  }
  else {
    statuses &= ~BUS_RECEIVING_TASK_OK;
  }
}

void ArmPart::setYCalibrating(bool value) {
  if (value) {
    statuses |= ARM_Y_CALIBRATING;
  } else {
    statuses &= ~ARM_Y_CALIBRATING;
  }
}

void ArmPart::setZCalibrating(bool value) {
  if (value) {
    statuses |= ARM_Z_CALIBRATING;
  }
  else {
    statuses &= ~ARM_Z_CALIBRATING;
  }
}

void ArmPart::setXCalibrating(bool value) {
  if (value) {
    statuses |= ARM_X_CALIBRATING;
  } else {
    statuses &= ~ARM_X_CALIBRATING;
  }
}

void ArmPart::setUpgrading(bool value) {
  if (value) {
    statuses |= ARM_UPGRADING;
  }
  else {
    statuses &= ~ARM_UPGRADING;
  }
}

void ArmPart::setHomeQuaternion(IMUQuaternion &homeQuaternion, IMUQuaternion &platformQuaternion) {
  this->homeQuaternion = Quaternion(homeQuaternion);
  this->platformHomeQuaternion = Quaternion(platformQuaternion);
  Quaternion homeInverted = Quaternion::Conjugate(homeQuaternion);
  this->offsetQuaternion = Quaternion::Multiply(platformHomeQuaternion, homeInverted);
  this->alignedOffsetQuaternion = align(homeQuaternion);
}

Quaternion ArmPart::align(IMUQuaternion &quat) {
  return Quaternion::Multiply(offsetQuaternion, quat);
}

Quaternion ArmPart::difference(IMUQuaternion &quat) {
  Quaternion qt = Quaternion(quat);
  return Quaternion::Difference(qt, homeQuaternion);
}

int ArmPart::sendFirmwareUpgradeMessage() {
  uint8_t id = getFirmwareUpgradeMessageId();
  if (id == 0) return -1;
  bus.send(id, 0);
  return 0;
}