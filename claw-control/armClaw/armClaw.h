#pragma once

#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/localWitmotion/localWitmotion.h"
#include "../../common/rangeDetector/rangeDetector.h"
#include "../../common/remoteWrist/remoteWrist.h"
#include "../../common/servo/servo.h"
#include "../../common/statusLed/statusLed.h"
#include "pico/stdlib.h"

class ArmClaw : public ArmPart {
private:
  LocalWitmotion imu;
  static void engineTask(void *instance);
  void busReceiveCallback(can2040_msg frame);
  RangeDetector rangeDetector;
  void calibrateLoop();
  void calibrateYLoop();
  void calibrateXLoop();
  void engineLoop();
  TaskHandle_t taskHandle = NULL;
  RemoteWrist wrist;
  StatusLed statusLed{STATUS_LED_PIN};
  TickType_t lastGuardTripTick = 0;
  void updateStatusLed(bool calibrating);
  float angleY(const Quaternion &q);
  float angleX(const Quaternion &q);
  Quaternion makeRotationX(float angleX);
  AtomicQuaternion base;
  AtomicValue<uint8_t> useIMUMode{static_cast<uint8_t>(USE_IMU_USE), pdMS_TO_TICKS(500)};
  // Mounting correction: LocalWitmotion has no setRotate() hook, so this is applied manually via
  // correctedQuat(). Identity until measured live (motor-driven single-axis moves, per wrist's q_corr).
  Quaternion q_corr{0.0f, 0.0f, 0.0f, 1.0f};
  Quaternion correctedQuat() { return imu.quaternion.load() * q_corr; }

public:
  Servo clawX;
  Servo clawY;
  Servo clawGripper;
  ArmClaw(const uint8_t detectorsSdaPin, const uint8_t detectorsSclPin,
          const uint8_t engineXPin, const uint8_t engineYPin,
          const uint8_t engineGripperPin, const uint8_t canRxPin,
          const uint8_t canTxPin, const uint8_t memsRxPin,
          const uint8_t memsTxPin, const uint8_t memsRstPin,
          const uint8_t memsIntPin, const uint8_t shortDetectorShutPin,
          const uint8_t longDetectorShutPin);

  uint32_t getQuaternionMessageId() { return CAN_CLAW_QUATERNION; };
  uint32_t getAccelerometerMessageId() { return CAN_CLAW_ACCELEROMETER; };
  uint32_t getGyroscopeMessageId() { return CAN_CLAW_GYROSCOPE; };
  uint32_t getHeightMessageId() { return CAN_CLAW_HEIGHT; };
  uint32_t getRangeMessageId() { return CAN_CLAW_RANGE; };
  uint32_t getStatusesMessageId() { return CAN_CLAW_STATUSES; };
  int updateQuaternion(IMUBase *position);
  int updateAccelerometer(IMUBase *position);
  int updateGyroscope(IMUBase *position);
  int updateAccuracy(IMUBase *position);
  int updateHeight(IMUBase *position);
  float getLocalX(float physicalX) { return physicalX + CLAW_X_HOME_POSITION; }
  float getLocalY(float physicalY) { return physicalY + CLAW_Y_HOME_POSITION; }
  float getLocalZ(float physicalZ) { return 0; }
};