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
  float homeI, homeJ, homeK, homeReal;
  float platformHomeI, platformHomeJ, platformHomeK, platformHomeReal;

  EEPROMPositionData() : 
    FlashSettingsData(sizeof(homeI) * 8, HOME_POSITIONS_EEPROM_OFFSET) {    
    homeI = homeJ = homeK = homeReal = 0.0f;
    platformHomeI = platformHomeJ = platformHomeK = platformHomeReal = 0.0f;
  }

  uint8_t *getBuffer() override {
    return reinterpret_cast<uint8_t *>(&homeI);
  }

  const uint8_t *getBuffer() const override {
    return reinterpret_cast<const uint8_t *>(&homeI);
  }
};

class ArmPart {
private:
  Bus bus;
  static void canCallback(void *pArmPart, can2040_msg frame);
  volatile uint64_t statuses;      
protected:
  virtual void busReceiveCallback(can2040_msg frame) {};
  int updateQuaternion(Quaternion quat);
  int updateAccelerometer(Accelerometer acc);
  int updateGyroscope(Gyroscope gyro);
  int updateAccuracy(Accuracy acc);
  int updateHeight(uint32_t height, uint16_t temperature);
public:
  ArmPlatform platform;
  void setPositionTaskStatus(bool value);
  void setEngineTaskStatus(bool value);
  void setBusReceivingTaskStatus(bool value);
  void setYCalibrating(bool value);
  void setZCalibrating(bool value);
  void setXCalibrating(bool value);
  void setUpgrading(bool value);
  Quaternion align(const Quaternion& dest, const Quaternion& source);
  Quaternion difference(const Quaternion& a, const Quaternion& b);
  Quaternion homeQuaternion;
  Quaternion platformHomeQuaternion;  
  Quaternion alignedOffsetQuaternion;
  void setHomeQuaternion(Quaternion homeQuaternion, Quaternion platformQuaternion);
  void saveHomeQuaternionsToEEPROM();
  bool loadHomeQuaternionsFromEEPROM();
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
  ArmPart(const uint canRxPin, const uint canTxPin);
};