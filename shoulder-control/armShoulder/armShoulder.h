#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/quaternion/quaternion.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"

class ArmShoulder : public ArmPart
{
private:
  LocalBNO imu;  
  static void engineTask(void *instance);
  void busReceiveCallback(can2040_msg frame);
  static volatile QueueHandle_t angleQueue;  
public:
  Servo shoulderZ;
  Servo shoulderY;
  ArmShoulder(
      uint memsSdaPin,
      uint memsSclPin,
      uint memsIntPin,
      uint memsRstPin,
      uint engineZPin,
      uint engineYPin,
      uint canRxPin,
      uint canTxPin);
  uint32_t getQuaternionMessageId() { return CAN_SHOULDER_QUATERNION; };
  uint32_t getAccelerometerMessageId() { return CAN_SHOULDER_ACCELEROMETER; };
  uint32_t getGyroscopeMessageId() { return CAN_SHOULDER_GYROSCOPE; };
  uint32_t getAccuracyMessageId() { return CAN_SHOULDER_ACCURACY; };
  uint32_t getStatusesMessageId() { return CAN_SHOULDER_STATUSES; };
  int updateAccelerometer(IMUBase *position);
  int updateGyroscope(IMUBase *position);
  int updateAccuracy(IMUBase *position);
  int updateQuaternion(IMUBase *position);  
};