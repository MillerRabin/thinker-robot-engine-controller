#include "armPart.h"

ArmPart::ArmPart(const uint canRxPin, const uint canTxPin) : bus(canRxPin, canTxPin, this, canCallback) {
  FlashSettings::init();
  auto status = bus.begin();
  if (status!= CAN_SUCCESS) { 
    printf("Can error %d\n", status); 
  }
  FlashSettings::init();
  loadHomeQuaternion();
}

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

int ArmPart::updateHeight(uint32_t height, uint16_t temperature) {
  uint64_t data = (uint64_t)temperature << 32 | (uint64_t)height;
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
  return 0;
}

int ArmPart::updateRange(float longRange, float shortRange) {
  MeasureRange mRange;  
  mRange.set(longRange, shortRange);  
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

void ArmPart::setPositionStatus(bool value) {
  if (value) {
    statuses |= ARM_POSITION_OK;
  } else {
    statuses &= ~ARM_POSITION_OK;
  }
}

bool ArmPart::getPositionStatus() { 
  return (statuses & ARM_POSITION_OK) != 0;
}

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
  }
  else {
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

void ArmPart::setTareError(bool value) {
  if (value) {
    statuses |= ARM_TARE_ERROR;
  } else {
    statuses &= ~ARM_TARE_ERROR;
  }
}

bool ArmPart::getTareError() { 
  return (statuses & ARM_TARE_ERROR) != 0; 
}

void ArmPart::setHomeQuaternion(Quaternion homeQuaternion, Quaternion platformQuaternion) {  
  Quaternion pi = Quaternion::Conjugate(platformQuaternion);
  this->offset = pi * homeQuaternion;
  EEPROMPositionData ep;
  ep.set(this->offset);  
  FlashSettings::save(ep);
}

Quaternion ArmPart::align(const Quaternion& dest, const Quaternion& source) {  
  return Quaternion::Multiply(dest, source);
}

Quaternion ArmPart::difference(const Quaternion& a, const Quaternion& b) {
  Quaternion pInv = Quaternion::Conjugate(b);
  Quaternion qn = Quaternion::Multiply(pInv, a);
  if (qn.real < 0.0f) {
    qn.real = -qn.real;
    qn.i = -qn.i;
    qn.j = -qn.j;
    qn.k = -qn.k;
  }
  
  return qn;
}

int ArmPart::sendFirmwareUpgradeMessage() {
  uint8_t id = getFirmwareUpgradeMessageId();
  if (id == 0) return -1;
  bus.send(id, 0);
  return 0;
}

bool ArmPart::loadHomeQuaternion() {
  EEPROMPositionData quaternionData;  
  if (!FlashSettings::load(quaternionData)) {    
    setTareError(true);
    return false;
  }  
  this->offset.deserialize(quaternionData.getBuffer());  
  setTareError(!this->offset.isZero());
  return true;
}

void ArmPart::scheduleSave() {
  needSaveQuaternion = true;
}

void ArmPart::tick(const Quaternion &origin, const Quaternion &current) {
  TickType_t now = xTaskGetTickCount();
  if (needSaveQuaternion) {
    if (tareStart != 0) {
      if (needSaveQuaternion && (now - tareStart) > tareDelay) {
        setHomeQuaternion(current, origin);
        tareStart = 0;
        needSaveQuaternion = false;
        printf("Home quaternion saved\n");
      }
    } else {
      tareStart = now;
    }
  }
}