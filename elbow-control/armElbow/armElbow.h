#pragma once

#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/remoteShoulder/remoteShoulder.h"
#include "../../common/servo/servo.h"
#include "../../common/statusLed/statusLed.h"
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
  StatusLed statusLed{STATUS_LED_PIN};
  // Latched, not live isDiverging() - a trip can self-clear within one tick, too fast to see.
  TickType_t lastGuardTripTick = 0;
  void updateStatusLed(bool calibrating, bool ready = false);
  // Edge-detected in updateStatusLed() - on the engines-on-to-off transition, proactively
  // invalidates base/pitchIntegrationValid rather than waiting for the next calibration.
  bool lastEnginesEnabled = true;
  bool settled(bool withinTolerance, TickType_t &stableSince);
  TaskHandle_t taskHandle = NULL;
  RemoteShoulder shoulder;
  float angleY();
  // Relative-to-shoulder orientation at the moment calibrateYLoop() last converged, so getIMUAngles() reads 0 there.
  Quaternion base;
  bool pitchIntegrationValid = false;
  float accumulatedPitchY = 0.0f;
  // Absolute gravity pitch (rad) captured when pitchIntegrationValid re-anchors - see getIMUAngles().
  float gravityPitchAtBase = 0.0f;
  // use is the intended default; not-use is the emergency fallback.
  AtomicValue<uint8_t> useIMUMode{static_cast<uint8_t>(USE_IMU_USE), pdMS_TO_TICKS(500)};
  float angleFromGravityY();
  float shoulderYAngleDeg();
  // PWM angle calibrateYLoop() last landed on (gravity-verified vertical) - also maps the gravity
  // reading into PWM-space for the next seed, see calibrateYLoop().
  float elbowYHomeAngle = ELBOW_Y_PWM_AT_VERTICAL;
  float accelPitchDeg();
  public:
    Servo elbowY;
    ArmElbow(const uint memsSCKPin, const uint memsMISOPin,
             const uint memsMOSIPin, const uint memsCSPin,
             const uint memsIntPin, const uint memsRstPin,
             const uint engineYPin, const uint canRxPin, 
             const uint canTxPin);
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
    void onIMUReset();
    Vector3 getIMUAngles();
    Vector3 getPhysicalAngles(Vector3 & imuAngles);
  };