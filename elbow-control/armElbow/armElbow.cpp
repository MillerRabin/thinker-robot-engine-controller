#include "armElbow.h"
#include "../common/periodic/periodic.h"

void ArmElbow::calibrateLoop() {
  setArmCalibrated(false);
  updateStatuses();
  calibrateYLoop();
  setArmCalibrated(true);
  updateStatuses();
}

// Closes the loop on the accelerometer, not on the servo's own commanded
// angle. angleFromGravityY() reads in "gravity-formula degrees" where 90
// means vertical (same convention ArmShoulder's formula uses) - that is a
// DIFFERENT scale than ELBOW_Y_HOME_POSITION (a PWM/servo angle), so it must
// never be fed directly into setDegreeDirect() - doing that previously
// caused a hard, uncontrolled jump the instant calibration started. Instead
// ramp smoothly (speed-limited via tick()) from wherever physicalAngle
// already is toward wherever the accelerometer reads 90.
void ArmElbow::calibrateYLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  setYCalibrating(true);
  // Force physicalAngle back to NaN before every calibration, not just after
  // engineLoop()'s own reset() on a later re-run. On a fresh boot/reflash
  // physicalAngle still holds the Servo constructor's nominal home value
  // (ELBOW_Y_HOME_POSITION), not the servo's true physical position - the
  // very first tick() below would otherwise write that nominal value to the
  // PWM immediately (a hard jump) before settling into the intended smooth
  // ramp. Resetting here makes setIMUAngle()'s NaN-seed path (below) always
  // seed physicalAngle from the live accelerometer reading instead.
  elbowY.reset();
  elbowY.setTargetAngle(90.0f, 1000, ELBOW_DEAD_ZONE);
  while (!elbowY.isPositioned()) {
    float gravityY = angleFromGravityY();
    printer.interval([&]() {
      LogQueue::Log("Calibrating Y... IMU Y: %.3f, Physical Y: %.3f\n", gravityY, elbowY.getPhysicalAngle());
    });
    elbowY.setIMUAngle(gravityY);
    elbowY.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  elbowYHomeAngle = elbowY.getPhysicalAngle();
  setYCalibrating(false);
}


void ArmElbow::engineTask(void *instance) {
  auto *elbow = static_cast<ArmElbow *>(instance);  
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    auto sp = elbow->imu.isPositionOK();
    auto pp = elbow->shoulder.isPositionOK();        
    if (!sp || !pp ) {
      elbow->setEngineTaskStatus(false);
      elbow->updateStatuses();
      LogQueue::Log("Engine task failed sp-%d, pp-%d\n", sp, pp);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    
    Quaternion eQuat = elbow->imu.quaternion.load();
    Quaternion sQuat = elbow->shoulder.imu.quaternion.load();

    if (!eQuat.isValid() || !sQuat.isValid()) {
      elbow->setEngineTaskStatus(false);
      elbow->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    elbow->setEngineTaskStatus(true);
    elbow->updateStatuses();
    elbow->calibrateLoop();
    elbow->engineLoop();

    elbow->setEngineTaskStatus(false);
    elbow->updateStatuses();    
  }
}

void ArmElbow::engineLoop() {
  LogQueue::Log("Starting Engine Loop\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  base = shoulder.imu.quaternion.load().invert();
  // calibrateYLoop() leaves targetAngle at 90 in gravity/accelerometer-space
  // (90 = vertical), but this loop tracks physicalAngle self-referentially
  // in PWM-space (setIMUAngle(getPhysicalAngle())). Left unretargeted, that
  // scale mismatch drags physicalAngle away from the true calibrated
  // vertical (elbowYHomeAngle, a PWM value ~135) down toward the number 90
  // in PWM-space, which is nowhere near vertical - retarget here so holding
  // is anchored to where calibration actually converged.
  elbowY.setTargetAngle(elbowYHomeAngle, 500, ELBOW_DEAD_ZONE);
  while (true) {
    auto sp = imu.isPositionOK();
    auto pp = shoulder.isPositionOK();
    if (!sp || !pp) {
      LogQueue::Log("Engine is turned off\n");
      elbowY.reset();
      break;
    }

    elbowY.setIMUAngle(elbowY.getPhysicalAngle());
    elbowY.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

ArmElbow::ArmElbow(const uint memsSdaPin, const uint memsSclPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineYPin, const uint canRxPin,
                   const uint canTxPin) : ArmPart(canRxPin, canTxPin), elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100),
                                          imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {
  
  
  // Derived empirically: moved elbow-y and shoulder-z independently by known
  // amounts, measured the resulting raw-quaternion rotation axis for each
  // (common/quaternion Difference + GetAxis), then solved for the rotation
  // that maps target axes (0,-1,0)/(0,0,-1) - matching ArmShoulder's Y/Z
  // convention - onto the measured axes. Validated: after applying this,
  // an elbow-y-only move shows up as a pure Y-twist (Z leakage < 1 deg),
  // and a shoulder-z-only move (elbow held still) shows up as a pure
  // Z-twist matching the shoulder's own reported Z angle within ~0.4 deg.
  Quaternion q_corr = {-0.314134f, -0.584772f, -0.348743f, 0.661619f};
  imu.setRotate(q_corr);
}

int ArmElbow::begin() {
  if (xTaskCreateAffinitySet(ArmElbow::engineTask, "ArmElbow::engineTask",
                             4096, this, 4, ARM_CORE, &taskHandle) == pdFAIL) {
    printf("Failed to create ArmElbow engine task\n");
    return ERROR_ENGINE_TASK_CREATION_FAILED;
  } else {
  }
  return 0;
}

int ArmElbow::updateQuaternion(IMUBase *position) {
  Quaternion quat = position->quaternion.load();
  return ArmPart::updateQuaternion(quat);
}

int ArmElbow::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmElbow::updateAccelerometer(IMUBase *position) {  
  Accelerometer acc = position->accelerometer.load();
  return ArmPart::updateAccelerometer(acc);
}

int ArmElbow::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmElbow::busReceiveCallback(can2040_msg frame) {
  shoulder.dispatchMessage(frame);

  if (frame.id == CAN_ELBOW_SET_Y_DEGREE) {

    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
    uint16_t timeMS = (raw >> 16) & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    LogQueue::Log("CAN_ELBOW_SET_Y_DEGREE received: angleY=%.2f timeMS=%u physicalAngle=%.2f\n", angleY, timeMS, elbowY.getPhysicalAngle());

    if (!isnan(angleY)) {
      elbowY.setTargetAngle(angleY, timeMS, ELBOW_DEAD_ZONE);
    }
  }

  if (frame.id == CAN_ELBOW_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}

float ArmElbow::angleY() {
  Accelerometer acc = imu.accelerometer.load();  
  return std::atan2(acc.y, acc.z);    
}

Vector3 ArmElbow::getPhysicalAngles(Vector3 &imuAngles) {
  return {0, (imuAngles.y * RAD_TO_DEG) + elbowYHomeAngle, 0};
}

// shoulder.imu.quaternion is the shoulder's base-corrected quaternion (shoulder
// sends base * rawQuaternion over CAN), home-relative, same decomposition as
// ArmShoulder::getIMUAngles() uses to get its own Y angle.
float ArmElbow::shoulderYAngleDeg() {
  Quaternion sq = shoulder.imu.quaternion.load();
  if (!sq.isValid()) {
    return 0.0f;
  }
  float yawZ = sq.twistAngle({0.0f, 0.0f, 1.0f});
  Quaternion qZ = Quaternion::AngleAxis(yawZ, 0.0f, 0.0f, 1.0f);
  Quaternion qSwing = qZ.invert() * sq;
  float pitchY = qSwing.twistAngle({0.0f, 1.0f, 0.0f});
  return pitchY * RAD_TO_DEG;
}

// Confirmed via a full PWM-tagged sweep of the entire 0-270 range (both
// directions, results repeatable/no hysteresis): the corrected reading is
// monotonic across the whole range once the atan2 +/-180 wrap is undone -
// continuous band from about -30 (PWM=0) through +180 (PWM~225) to +216
// (PWM=270). The wrap into large-negative values only happens past
// PWM~225, so any strongly-negative reading is that continuation, not a
// fresh cycle - add 360 to unwrap it back onto the same continuous scale.
// Bounds give a little margin around the actual tested extremes.
constexpr float ELBOW_GRAVITY_UNWRAP_THRESHOLD = -90.0f;
constexpr float ELBOW_GRAVITY_Y_MIN = -35.0f;
constexpr float ELBOW_GRAVITY_Y_MAX = 220.0f;

float ArmElbow::angleFromGravityY() {
  Accelerometer acc = imu.accelerometer.load();
  float res = atan2(acc.y, acc.z) * RAD_TO_DEG;
  // Elbow's own accelerometer sees gravity in its local frame, which is
  // already tilted by the shoulder's current Y angle - subtract it out so
  // the result reflects only the elbow's own joint angle.
  res -= shoulderYAngleDeg();
  if (res < ELBOW_GRAVITY_UNWRAP_THRESHOLD) {
    res += 360.0f;
  }
  if (res < ELBOW_GRAVITY_Y_MIN || res > ELBOW_GRAVITY_Y_MAX) {
    // Out of the trusted window - report invalid rather than risk driving
    // the servo on a corrupted reading.
    return NAN;
  }
  return res;
}

Vector3 ArmElbow::getIMUAngles() {
  Quaternion sq = shoulder.imu.quaternion.load();
  Quaternion qm = imu.quaternion.load();
  float pitchX = qm.twistAngle({1.0f, 0.0f, 0.0f});
  /*float pitchY = qm.twistAngle({0.0f, 1.0f, 0.0f});
  float pitchZ = qm.twistAngle({0.0f, 0.0f, 1.0f});*/  
  Quaternion qZ = Quaternion::AngleAxis(pitchX, 0.0f, 0.0f, 1.0f);
  Quaternion qSwing = qZ.invert() * qm;
  float yawX = qSwing.twistAngle({1.0f, 1.0f, 0.0f});
  float yawY = qSwing.twistAngle({0.0f, 1.0f, 0.0f});
  float yawZ = qSwing.twistAngle({0.0f, 0.0f, 1.0f});
  LogQueue::Log("Raw IMU angles: X: %.2f, pitchX: %.2f, pitchY: %.2f, pitchZ: %.2f\n", pitchX * RAD_TO_DEG, yawX * RAD_TO_DEG, yawY * RAD_TO_DEG, yawZ * RAD_TO_DEG);
  return {0, pitchX, yawX};
}
