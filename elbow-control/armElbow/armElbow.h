#pragma once

#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/remoteShoulder/remoteShoulder.h"
#include "../../common/servo/servo.h"
#include "pico/stdlib.h"

class ArmElbowQueueParams {
public:
  float elbowY = NAN;
};

class ArmElbow : public ArmPart {
private:
  LocalBNO imu;
  static void engineTask(void *instance);
  void busReceiveCallback(can2040_msg frame);
  bool calibrateYLoop();
  bool calibrateLoop();
  void engineLoop();
  Vector3 trackTick();
  bool settled(bool withinTolerance, TickType_t &stableSince);
  TaskHandle_t taskHandle = NULL;
  RemoteShoulder shoulder;
  float angleY();
  // Relative-to-shoulder orientation at the moment calibrateYLoop() last converged, so getIMUAngles() reads 0 there.
  Quaternion base;
  // pitchY is tracked by accumulating the small swing rotation between
  // consecutive ticks (see getIMUAngles()) rather than re-deriving an
  // absolute magnitude+sign every tick - same fix as ArmShoulder's pitchX,
  // ported here before making useIMU:use elbow's default. Reset alongside
  // base whenever calibration re-anchors it.
  Quaternion lastOrientation;
  bool pitchIntegrationValid = false;
  float accumulatedPitchY = 0.0f;
  // use is the intended default (matches ArmShoulder); not-use is the
  // emergency fallback, not the normal state. Safe now that getIMUAngles()
  // has the same delta-integration + drift-correction protection shoulder's
  // does (see lastOrientation/pitchIntegrationValid/accumulatedPitchY above).
  AtomicValue<uint8_t> useIMUMode{static_cast<uint8_t>(USE_IMU_USE), pdMS_TO_TICKS(500)};
  float angleFromGravityY();
  float shoulderYAngleDeg();
  // PWM angle calibrateYLoop() last landed on (accelerometer-verified vertical).
  float elbowYHomeAngle = ELBOW_Y_HOME_POSITION;
  public:
    Servo elbowY;
    ArmElbow(uint memsSdaPin, uint memsSclPin, uint memsIntPin, uint memsRstPin,
             uint engineYPin, uint canRxPin, uint canTxPin);
    uint32_t getQuaternionMessageId() { return CAN_ELBOW_QUATERNION; };
    uint32_t getAccelerometerMessageId() { return CAN_ELBOW_ACCELEROMETER; };
    uint32_t getGyroscopeMessageId() { return CAN_ELBOW_GYROSCOPE; };
    uint32_t getAccuracyMessageId() { return CAN_ELBOW_ACCURACY; };
    uint32_t getStatusesMessageId() { return CAN_ELBOW_STATUSES; };
    int updateQuaternion(IMUBase * position);
    int updateAccelerometer(IMUBase * position);
    int updateGyroscope(IMUBase * position);
    int updateAccuracy(IMUBase * position);
    int begin();
    int updateStatuses();
    Vector3 getIMUAngles();
    Vector3 getPhysicalAngles(Vector3 & imuAngles);
  };