#pragma once

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "../servo/servo.h"
#include "../bus/bus.h"
#include "../config/config.h"
#include "../armPlatform/armPlatform.h"
#include "../imuBase/imuBase.h"
#include "../flashSettings/flashSettings.h"

struct __attribute__((packed, aligned(4))) EEPROMPositionData : public FlashSettingsData {
  uint64_t homeData;

  EEPROMPositionData() : 
    FlashSettingsData(sizeof(homeData), HOME_POSITIONS_EEPROM_OFFSET) {      
  }

  void set(const Quaternion& q) {
    homeData = q.serialize();
  }

  uint8_t *getBuffer() override {
    return reinterpret_cast<uint8_t *>(&homeData);
  }

  const uint8_t *getBuffer() const override {
    return reinterpret_cast<const uint8_t *>(&homeData);
  }
};

class ArmPart {
private:  
  Bus bus;
  static void canCallback(void *pArmPart, can2040_msg frame);
  volatile uint64_t statuses;
  bool needSaveQuaternion = false;
  TickType_t tareStart = 0;
  const TickType_t tareDelay = pdMS_TO_TICKS(QUATERNION_SAVE_DELAY);  
protected:
  constexpr static TickType_t taskInterval = pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT_US / 1000);
  virtual void busReceiveCallback(can2040_msg frame) {};
  int updateQuaternion(Quaternion quat);
  int updateAccelerometer(Accelerometer acc);
  int updateGyroscope(Gyroscope gyro);
  int updateAccuracy(Accuracy acc);
  int updateHeight(uint32_t height, uint16_t temperature);
  void scheduleSave();
  void tick(const Quaternion& origin, const Quaternion& current);
public:
  ArmPlatform platform;
  void setPositionStatus(bool value);
  bool getPositionStatus();
  void setEngineTaskStatus(bool value);
  void setBusReceivingTaskStatus(bool value);
  void setTareError(bool value);
  bool getTareError();
  void setYCalibrating(bool value);
  void setZCalibrating(bool value);
  void setXCalibrating(bool value);
  void setUpgrading(bool value);
  Quaternion align(const Quaternion& dest, const Quaternion& source);
  Quaternion difference(const Quaternion& a, const Quaternion& b);
  Quaternion offset;
  void setHomeQuaternion(Quaternion homeQuaternion, Quaternion platformQuaternion);  
  bool loadHomeQuaternion();
  virtual uint32_t getQuaternionMessageId() { return 0; };
  virtual uint32_t getAccelerometerMessageId() { return 0; };
  virtual uint32_t getGyroscopeMessageId() { return 0; };
  virtual uint32_t getAccuracyMessageId() { return 0; };
  virtual uint32_t getHeightMessageId() { return 0; };
  virtual uint32_t getRangeMessageId() { return 0; };
  virtual uint32_t getStatusesMessageId() { return 0; };
  virtual uint32_t getFirmwareUpgradeMessageId() { return 0; };  
  virtual int updateQuaternion(IMUBase *position) { return 0; };
  virtual int updateAccelerometer(IMUBase *position) { return 0; };
  virtual int updateGyroscope(IMUBase *position) { return 0; };
  virtual int updateAccuracy(IMUBase *position) { return 0; };
  virtual int updateHeight(IMUBase *position) { return 0; };
  virtual int updateRange(float longRange, float shortRange);
  virtual int sendFirmwareUpgradeMessage();
  virtual int updateStatuses();
  static float NormalizeAngle(float angle);
  ArmPart(const uint canRxPin, const uint canTxPin);
};