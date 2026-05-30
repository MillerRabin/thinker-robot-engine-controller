#pragma once

#include "../armPlatform/armPlatform.h"
#include "../bus/bus.h"
#include "../config/config.h"
#include "../flashSettings/flashSettings.h"
#include "../imuBase/imuBase.h"
#include "../servo/servo.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

struct __attribute__((packed, aligned(4))) EEPROMPositionData
    : public FlashSettingsData {
  uint64_t homeData;

  EEPROMPositionData()
      : FlashSettingsData(sizeof(homeData), HOME_POSITIONS_EEPROM_OFFSET) {}

  void set(const Quaternion &q) { homeData = q.serialize(); }

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
  volatile uint64_t statuses = 0;

protected:
  constexpr static TickType_t taskInterval =
      pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT_US / 1000);
  virtual void busReceiveCallback(can2040_msg frame) {};
  int updateQuaternion(Quaternion quat);
  int updateAccelerometer(Accelerometer acc);
  int updateGyroscope(Gyroscope gyro);
  int updateAccuracy(Accuracy acc);
  int updateHeight(uint32_t height, uint16_t temperature);  
  Quaternion offset;

public:
  ArmPlatform platform;
  void setPositionStatus(bool value);
  bool getPositionStatus();
  void setEngineTaskStatus(bool value);
  void setBusReceivingTaskStatus(bool value);
  void setYCalibrating(bool value);
  void setZCalibrating(bool value);
  void setXCalibrating(bool value);
  void setArmCalibrated(bool value);
  void setUpgrading(bool value);
  void setOffsetZYX(const float angleX, const float angleY, const float angleZ,
                    const Quaternion &imu);
  void setOffsetYZX(const float angleX, const float angleY, const float angleZ,
                    const Quaternion &imu);
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
  virtual Vector3 getIMUAngles(float physicalX, float physicalY, float physicalZ) { return Vector3{0, 0, 0}; };
  virtual Vector3 getIMUAngles() { return Vector3{0, 0, 0}; };
  virtual Vector3 getPhysicalAngles( Vector3& imuAngles) { return Vector3{0, 0, 0}; };
  static float NormalizeAngle(float angle);
  static bool Equals(float a, float b, float threshold);
  ArmPart(const uint canRxPin, const uint canTxPin);
};