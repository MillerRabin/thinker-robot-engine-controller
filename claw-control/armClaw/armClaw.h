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
  bool calibrateLoop();
  bool calibrateYLoop();
  bool calibrateXLoop();
  void engineLoop();
  Vector3 trackTick();
  bool settled(bool withinTolerance, TickType_t &stableSince);
  TaskHandle_t taskHandle = NULL;
  RemoteWrist wrist;
  StatusLed statusLed{STATUS_LED_PIN};
  TickType_t lastGuardTripTick = 0;
  void updateStatusLed(bool calibrating, bool ready = false);
  // Edge-detected in updateStatusLed() - on the engines-on-to-off transition, proactively
  // invalidates base/pitchIntegrationValid rather than waiting for the next calibration.
  bool lastEnginesEnabled = true;
  AtomicQuaternion base;
  AtomicValue<uint8_t> useIMUMode{static_cast<uint8_t>(USE_IMU_USE), pdMS_TO_TICKS(500)};
  // Mounting correction, measured live via motor-driven single-axis moves (2026-08-14) -
  // LocalWitmotion has no setRotate() hook, so this is applied manually via correctedQuat().
  Quaternion q_corr{0.573523f, 0.608072f, -0.360074f, -0.414326f};
  Quaternion correctedQuat() { return imu.quaternion.load() * q_corr; }
  // PWM angle calibrateXLoop()/calibrateYLoop() last landed on (gravity-verified).
  float clawXHomeAngle = CLAW_X_HOME_POSITION;
  float clawYHomeAngle = CLAW_Y_HOME_POSITION;
  // Delta-integrated per tick rather than re-derived absolutely - see getIMUAngles().
  Quaternion lastOrientation;
  bool pitchIntegrationValid = false;
  float accumulatedPitchX = 0.0f;
  float accumulatedPitchY = 0.0f;
  float angleFromGravityX();
  float angleFromGravityY();

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
  Vector3 getIMUAngles();
  Vector3 getPhysicalAngles(Vector3 &imuAngles);
  int updateQuaternion(IMUBase *position);
  int updateAccelerometer(IMUBase *position);
  int updateGyroscope(IMUBase *position);
  int updateAccuracy(IMUBase *position);
  int updateHeight(IMUBase *position);
  int updateStatuses();
  void onIMUReset();
  float getLocalX(float physicalX) { return physicalX + CLAW_X_HOME_POSITION; }
  float getLocalY(float physicalY) { return physicalY + CLAW_Y_HOME_POSITION; }
  float getLocalZ(float physicalZ) { return 0; }
};