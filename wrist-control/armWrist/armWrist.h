#pragma once

#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/remoteClaw/remoteClaw.h"
#include "../../common/servo/servo.h"
#include "pico/stdlib.h"

class ArmWristQueueParams {
public:
  float wristY = NAN;
  float wristZ = NAN;
};

class ArmWrist : public ArmPart {
private:
  LocalBNO imu;
  static void engineTask(void *instance);
  void busReceiveCallback(can2040_msg frame);
  void calibrateYLoop();
  void calibrateZLoop();
  void calibrateLoop();
  void engineLoop();
  TaskHandle_t taskHandle = NULL;
  RemoteClaw claw;

public:
  Servo wristY;
  Servo wristZ;
  ArmWrist(uint memsSdaPin, uint memsSclPin, uint memsIntPin, uint memsRstPin,
           uint engineZPin, uint engineYPin, uint canRxPin, uint canTxPin);
  uint32_t getQuaternionMessageId() { return CAN_WRIST_QUATERNION; };
  uint32_t getAccelerometerMessageId() { return CAN_WRIST_ACCELEROMETER; };
  uint32_t getGyroscopeMessageId() { return CAN_WRIST_GYROSCOPE; };
  uint32_t getAccuracyMessageId() { return CAN_WRIST_ACCURACY; };
  uint32_t getStatusesMessageId() { return CAN_WRIST_STATUSES; };
  int updateQuaternion(IMUBase *position);
  int updateAccelerometer(IMUBase *position);
  int updateGyroscope(IMUBase *position);
  int updateAccuracy(IMUBase *position);
  float getLocalX(float physicalX) { return 0; }
  float getLocalY(float physicalY) { return physicalY + WRIST_Y_HOME_POSITION; }
  float getLocalZ(float physicalZ) { return physicalZ + WRIST_Z_HOME_POSITION; }
};