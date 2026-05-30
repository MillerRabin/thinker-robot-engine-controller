#include "armPart.h"

ArmPart::ArmPart(const uint canRxPin, const uint canTxPin)
    : bus(canRxPin, canTxPin, this, canCallback) {
  auto status = bus.begin();
  if (status != CAN_SUCCESS) {
    printf("Can error %d\n", status);
  }
}

int ArmPart::updateQuaternion(Quaternion quat) {
  uint64_t data = quat.serialize();
  uint8_t id = getQuaternionMessageId();
  if (id == 0)
    return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateGyroscope(Gyroscope gyro) {
  uint64_t data = gyro.serialize();
  uint8_t id = getGyroscopeMessageId();
  if (id == 0)
    return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateAccelerometer(Accelerometer acc) {
  uint64_t data = acc.serialize();
  uint8_t id = getAccelerometerMessageId();
  if (id == 0)
    return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateAccuracy(Accuracy acc) {
  uint64_t data = acc.serialize();
  uint8_t id = getAccuracyMessageId();
  if (id == 0)
    return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateHeight(uint32_t height, uint16_t temperature) {
  uint64_t data = (uint64_t)temperature << 32 | (uint64_t)height;
  uint8_t id = getHeightMessageId();
  if (id == 0)
    return -1;
  bus.send(id, data);
  return 0;
}

int ArmPart::updateStatuses() {
  uint8_t id = getStatusesMessageId();
  if (id == 0)
    return -1;
  bus.send(id, statuses);
  return 0;
}

int ArmPart::updateRange(float longRange, float shortRange) {
  MeasureRange mRange;
  mRange.set(longRange, shortRange);
  uint64_t data = mRange.serialize();
  uint8_t id = getRangeMessageId();
  if (id == 0)
    return -1;
  bus.send(id, data);
  return 0;
}

void ArmPart::canCallback(void *pArmPart, can2040_msg frame) {
  uint32_t ident = frame.id;
  ArmPart *armPart = (ArmPart *)pArmPart;
  armPart->platform.dispatchMessage(frame);
  armPart->busReceiveCallback(frame);
}

void ArmPart::setPositionStatus(bool value) {
  if (value) {
    statuses |= ARM_POSITION_OK;
  } else {
    statuses &= ~ARM_POSITION_OK;
  }
}

bool ArmPart::getPositionStatus() { return (statuses & ARM_POSITION_OK) != 0; }

void ArmPart::setEngineTaskStatus(bool value) {
  if (value) {
    statuses |= ARM_ENGINE_OK;
  } else {
    statuses &= ~ARM_ENGINE_OK;
  }
}

void ArmPart::setBusReceivingTaskStatus(bool value) {
  if (value) {
    statuses |= BUS_RECEIVING_OK;
  } else {
    statuses &= ~BUS_RECEIVING_OK;
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
  } else {
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

void ArmPart::setArmCalibrated(bool value) {
  if (value) {
    statuses |= ARM_CALIBRATED;
  } else {
    statuses &= ~ARM_CALIBRATED;
  }
}

void ArmPart::setUpgrading(bool value) {
  if (value) {
    statuses |= ARM_UPGRADING;
  } else {
    statuses &= ~ARM_UPGRADING;
  }
}

int ArmPart::sendFirmwareUpgradeMessage() {
  uint8_t id = getFirmwareUpgradeMessageId();
  if (id == 0)
    return -1;
  bus.send(id, 0);
  return 0;
}

float ArmPart::NormalizeAngle(float angle) {
  if (angle <= 0 && angle >= -90) {
    return angle;
  }
  angle = fmodf(angle, 360.0f);
  if (angle < 0)
    angle += 360.0f;
  return angle;
}

bool ArmPart::Equals(float a, float b, float threshold) {
  return (fabs(a - b) < threshold);
}

void ArmPart::setOffsetZYX(const float angleX, const float angleY,
                           const float angleZ, const Quaternion &imu) {
  
  Vector3 la = getIMUAngles(angleX, angleY, angleZ);
  Quaternion qx = Quaternion::AngleAxis(la.x * DEG_TO_RAD, 1.0f, 0.0f, 0.0f);
  Quaternion qy = Quaternion::AngleAxis(la.y * DEG_TO_RAD, 0.0f, 1.0f, 0.0f);
  Quaternion qz = Quaternion::AngleAxis(la.z * DEG_TO_RAD, 0.0f, 0.0f, 1.0f);
  Quaternion qm = qz * qy * qx;
  this->offset = qm * imu.invert(); 
}

void ArmPart::setOffsetYZX(const float angleX, const float angleY,
                           const float angleZ, const Quaternion &imu) {
  Vector3 la = getIMUAngles(angleX, angleY, angleZ);
  Quaternion qx = Quaternion::AngleAxis(la.x * DEG_TO_RAD, 1.0f, 0.0f, 0.0f);
  Quaternion qy = Quaternion::AngleAxis(la.y * DEG_TO_RAD, 0.0f, 1.0f, 0.0f);
  Quaternion qz = Quaternion::AngleAxis(la.z * DEG_TO_RAD, 0.0f, 0.0f, 1.0f);
  Quaternion qm = qy * qz * qx;
  this->offset = qm * imu.invert();
}