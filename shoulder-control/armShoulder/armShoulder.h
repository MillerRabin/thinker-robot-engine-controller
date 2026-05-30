#pragma once

#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/logQueue/logQueue.h"
#include "../../common/quaternion/quaternion.h"
#include "../../common/remoteElbow/remoteElbow.h"
#include "../../common/servo/servo.h"

#include "pico/stdlib.h"
#include <FreeRTOS.h>

class ArmShoulder : public ArmPart {
private:
  LocalBNO imu;
  static void engineTask(void *instance);
  void busReceiveCallback(can2040_msg frame);
  TaskHandle_t taskHandle = NULL;
  void calibrateYLoop();
  void calibrateZLoop();
  void calibrateLoop();
  void engineLoop();
  Quaternion base;
  float angleFromGravityY();    
  Quaternion getHomeQuaternion();
  float yAxisSign = 1.0f;
public:
  Servo shoulderZ;
  Servo shoulderY;
  ArmShoulder(uint memsSCKPin, uint memsMISOPin, uint memsMOSIPin,
              uint memsCSPin, uint memsIntPin, uint memsRstPin, uint engineZPin,
              uint engineYPin, uint canRxPin, uint canTxPin);
  uint32_t getQuaternionMessageId() { return CAN_SHOULDER_QUATERNION; };
  uint32_t getAccelerometerMessageId() { return CAN_SHOULDER_ACCELEROMETER; };
  uint32_t getGyroscopeMessageId() { return CAN_SHOULDER_GYROSCOPE; };
  uint32_t getAccuracyMessageId() { return CAN_SHOULDER_ACCURACY; };
  uint32_t getStatusesMessageId() { return CAN_SHOULDER_STATUSES; };
  uint32_t getUpgradeConfimedMessageId() {
    return CAN_SHOULDER_FIRMWARE_UPGRADE_CONFIRMED;
  };

  Vector3 getIMUAngles(float physicalX, float physicalY, float physicalZ);
  float getIMUAngleY();  
  float getIMUAngleZ();
  float getIMUAngleY(float physicalY);
  float getIMUAngleZ(float physicalZ);
  Vector3 getIMUAngles();  
  Vector3 getPhysicalAngles(Vector3 &imuAngles);
  int updateAccelerometer(IMUBase *position);
  int updateGyroscope(IMUBase *position);
  int updateAccuracy(IMUBase *position);
  int updateQuaternion(IMUBase *position);
  int begin();      
};