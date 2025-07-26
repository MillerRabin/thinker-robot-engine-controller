#pragma once

#include "pico/stdlib.h"
#include "../servo/servo.h"
#include "../bus/bus.h"
#include "../config/config.h"
#include "../armPlatform/armPlatform.h"
#include "../imuBase/imuBase.h"

#define ARM_POSITION_TASK_OK 1
#define ARM_ENGINE_TASK_OK 2

class ArmPart
{
private:
  Bus bus;
  static void canCallback(void *pArmPart, can2040_msg frame);
  volatile uint64_t statuses;

protected:
  virtual void busReceiveCallback(can2040_msg frame) {};
  int updateQuaternion(IMUQuaternion quat);
  int updateAccelerometer(Accelerometer acc);
  int updateGyroscope(Gyroscope gyro);
  int updateAccuracy(Accuracy acc);
  int updateHeight(uint32_t height);

public:
  ArmPlatform platform;
  void setPositionTaskStatus(bool value);
  void setEngineTaskStatus(bool value);
  virtual uint32_t getQuaternionMessageId() { return 0; };
  virtual uint32_t getAccelerometerMessageId() { return 0; };
  virtual uint32_t getGyroscopeMessageId() { return 0; };
  virtual uint32_t getAccuracyMessageId() { return 0; };
  virtual uint32_t getHeightMessageId() { return 0; };
  virtual uint32_t getRangeMessageId() { return 0; };
  virtual uint32_t getStatusesMessageId() { return 0; };
  virtual int updateQuaternion(IMUBase *position) { return 0; };
  virtual int updateAccelerometer(IMUBase *position) { return 0; };
  virtual int updateGyroscope(IMUBase *position) { return 0; };
  virtual int updateAccuracy(IMUBase *position) { return 0; };
  virtual int updateHeight(IMUBase *position) { return 0; };
  virtual int updateRange(uint16_t range, uint16_t measureType);
  virtual int updateStatuses();
  ArmPart(const uint canRxPin, const uint canTxPin);
};
