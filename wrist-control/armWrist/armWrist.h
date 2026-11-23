#pragma once

#include <atomic>

#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/rateEstimator/rateEstimator.h"
#include "../../common/plannedSpeed/plannedSpeed.h"
#include "../../common/remoteClaw/remoteClaw.h"
#include "../../common/remoteElbow/remoteElbow.h"
#include "../../common/remoteShoulder/remoteShoulder.h"
#include "../../common/servo/servo.h"
#include "../../common/statusLed/statusLed.h"
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
  bool calibrateYLoop();
  bool calibrateZLoop();
  bool calibrateLoop();
  void engineLoop();
  Vector3 trackTick();
  bool settled(bool withinTolerance, TickType_t &stableSince);
  TaskHandle_t taskHandle = NULL;
  RemoteClaw claw;
  RemoteElbow elbow;
  // Wrist Y's pitch rate for the active damping, and its [DAMP] log line.
  RateEstimator pitchRate{DAMPING_RATE_TAU_SEC};
  DampingMonitor dampingMonitor{"wrist y"};
  // The elbow's planned speed while it streams, off its waypoints on the bus.
  PlannedSpeed elbowPlan;
  // Only for the arm's turn about the vertical, see armYaw().
  RemoteShoulder shoulder;
  float lastArmYaw = 0.0f;
  float armYaw();
  float shoulderZCommandDeg();
  // The (turn as read - shoulder-z command) and yawOffset when the wrist last got where gravity can't
  // anchor the turn (see WRIST_YAW_CMD_MAX_OFF_DEG); NAN while it can.
  float yawHoldGap = NAN;
  float yawHoldOffset = 0.0f;
  // What armYaw() is off by for this wrist, and how far the wrist's frame has turned about its own up
  // axis since calibration - both kept by gravity (see getIMUAngles()).
  float yawOffset = 0.0f;              // rad
  float wristRoll = 0.0f;              // rad
  absolute_time_t yawOffsetAt = 0;
  float gravityBendZ = 0.0f;           // rad, the side bend read off gravity alone (getIMUAngles())
  // At a new base the arm hasn't turned: whatever the shoulder's game rotation vector has drifted to
  // since its own start (12deg after an hour, 2026-11-19 - the shoulder's base keeps its yaw) is offset.
  // The roll starts over too: learned against the old base. Done by getIMUAngles() on its next call:
  // onIMUReset() comes from the BNO's task.
  std::atomic_bool yawResetPending{true};
  void resetYawOffset() { yawResetPending.store(true); }
  StatusLed statusLed{STATUS_LED_PIN};
  void updateStatusLed(bool calibrating, bool ready = false);
  // Edge-detected in updateStatusLed() - on the engines-on-to-off transition, proactively
  // invalidates base/pitchIntegrationValid rather than waiting for the next calibration.
  bool lastEnginesEnabled = true;
  AtomicQuaternion base;
  AtomicValue<uint8_t> useIMUMode{static_cast<uint8_t>(USE_IMU_USE), pdMS_TO_TICKS(500)};
  // PWM angle calibrateYLoop() last landed on (gravity-verified vertical); base is relative to this.
  // Also maps the gravity reading into PWM-space for the next seed, see calibrateYLoop().
  float wristYHomeAngle = WRIST_Y_PWM_AT_VERTICAL;
  // PWM angle calibrateZLoop() last landed on (gravity-verified upright), the same role for Z.
  float wristZHomeAngle = WRIST_Z_HOME_POSITION;
  bool pitchIntegrationValid = false;
  float accumulatedPitchY = 0.0f;
  bool twistZIntegrationValid = false;
  float accumulatedTwistZ = 0.0f;
  float angleFromGravityY();
  float angleFromGravityZ();
  float elbowYAngleDeg();
  // Last trusted angleFromGravityY() reading and when it was taken, for its own glitch filter -
  // time-aware, matching ArmShoulder's identical fix (this function is called from both a tight
  // tracking loop and a slower idle-diagnostic loop, so a flat per-call jump threshold is wrong for one).
  float lastGoodGravityY = NAN;
  absolute_time_t lastGoodGravityYTime = 0;
  float lastGoodGravityZ = NAN;
  absolute_time_t lastGoodGravityZTime = 0;

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
  uint32_t getTunedMessageId() { return CAN_WRIST_TUNED; };
  Vector3 getIMUAngles();
  Vector3 getPhysicalAngles(Vector3 &imuAngles);
  int updateQuaternion(IMUBase *position);
  int updateAccelerometer(IMUBase *position);
  int updateGyroscope(IMUBase *position);
  int updateAccuracy(IMUBase *position);
  int updateStatuses();
  void onIMUReset();
  float getLocalX(float physicalX) { return 0; }
  float getLocalY(float physicalY) { return physicalY + WRIST_Y_HOME_POSITION; }
  float getLocalZ(float physicalZ) { return physicalZ + WRIST_Z_HOME_POSITION; }
};