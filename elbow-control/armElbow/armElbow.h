#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"
#include "../../common/remoteShoulder/remoteShoulder.h"

class ArmElbowQueueParams
{
public:
  float elbowY = NAN;
};

class ArmElbow : public ArmPart {
private:
  RemoteShoulder shoulder;
  LocalBNO imu;  
  static void engineTask(void *instance);
  void busReceiveCallback(can2040_msg frame);
  static volatile QueueHandle_t queue;
public:
  Servo elbowY;
  ArmElbow(
      uint memsSdaPin,
      uint memsSclPin,
      uint memsIntPin,
      uint memsRstPin,
      uint engineYPin,
      uint canRxPin,
      uint canTxPin);
  uint32_t getQuaternionMessageId() { return CAN_ELBOW_QUATERNION; };
  uint32_t getAccelerometerMessageId() { return CAN_ELBOW_ACCELEROMETER; };
  uint32_t getGyroscopeMessageId() { return CAN_ELBOW_GYROSCOPE; };
  uint32_t getAccuracyMessageId() { return CAN_ELBOW_ACCURACY; };
  uint32_t getStatusesMessageId() { return CAN_ELBOW_STATUSES; };
  int updateQuaternion(IMUBase *position);
  int updateAccelerometer(IMUBase *position);
  int updateGyroscope(IMUBase *position);
  int updateAccuracy(IMUBase *position);
  int begin();
};