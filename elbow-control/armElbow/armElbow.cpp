#include "armElbow.h"
#include "../common/periodic/periodic.h"

int ArmElbow::updateStatuses() {
  setUseIMUStatus(useIMUMode.load());
  return ArmPart::updateStatuses();
}

bool ArmElbow::calibrateLoop() {
  setArmCalibrated(false);
  updateStatuses();
  bool ok = calibrateYLoop();
  setArmCalibrated(ok);
  updateStatuses();
  return ok;
}

bool ArmElbow::settled(bool withinTolerance, TickType_t &stableSince) {
  if (!withinTolerance) {
    stableSince = 0;
    return false;
  }
  if (stableSince == 0) {
    stableSince = xTaskGetTickCount();
    return false;
  }
  return (xTaskGetTickCount() - stableSince) >= pdMS_TO_TICKS(CALIBRATION_STABLE_TIME_MS);
}

bool ArmElbow::calibrateYLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  setYCalibrating(true);

  // Require a run of consecutive, mutually-close gravity readings before
  // seeding - the very first raw reading right after a power cycle can be
  // wildly wrong (observed live: res=217, near the sensor's own range
  // limit, as the first sample), and elbowY.setIMUAngle() commits whatever
  // it's given instantly (physicalAngle jumps straight to it, no ramp,
  // since physicalAngle starts NaN after reset()) - an unvalidated bad seed
  // produces an uncontrolled physical jerk.
  constexpr int seedStableSamples = 5;
  constexpr float seedStableTolerance = 5.0f; // deg
  float lastSeedSample = NAN;
  int seedStableCount = 0;
  while (true) {
    if (!shoulder.isPositionOK()) {
      LogQueue::Log("Calibrating Y aborted: shoulder lost position (seeding)\n");
      setYCalibrating(false);
      return false;
    }
    float gravityY = angleFromGravityY();
    if (!isnan(gravityY) && !isnan(lastSeedSample) &&
        fabsf(gravityY - lastSeedSample) <= seedStableTolerance) {
      seedStableCount++;
    } else {
      seedStableCount = isnan(gravityY) ? 0 : 1;
    }
    lastSeedSample = gravityY;
    if (seedStableCount >= seedStableSamples) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }

  elbowY.reset();
  elbowY.setTargetAngle(90.0f, 1000, ELBOW_DEAD_ZONE);
  TickType_t stableSince = 0;
  bool ok = true;
  while (true) {
    if (!shoulder.isPositionOK()) {
      LogQueue::Log("Calibrating Y aborted: shoulder lost position\n");
      ok = false;
      break;
    }
    // A frozen (diverging) servo can never satisfy settled() - without this
    // check the loop spins forever once the guard freezes it (reproduced
    // live: guard correctly froze at physical=270 with "pinned at limit"
    // message, but the loop kept spinning for 15+ more iterations reading
    // wild gravity swings until an unrelated shoulder-position loss finally
    // broke it out). Same fix as ArmShoulder's calibration loops.
    if (elbowY.isDiverging()) {
      LogQueue::Log("Calibrating Y aborted: elbowY diverging\n");
      ok = false;
      break;
    }
    float gravityY = angleFromGravityY();
    printer.interval([&]() {
      LogQueue::Log("Calibrating Y... IMU Y: %.3f, Physical Y: %.3f\n", gravityY, elbowY.getPhysicalAngle());
    });
    elbowY.setIMUAngle(gravityY);
    elbowY.tick();
    updateStatuses();
    if (settled(fabsf(90.0f - gravityY) <= CALIBRATION_SETTLE_TOLERANCE, stableSince)) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  if (ok) {
    elbowYHomeAngle = elbowY.getPhysicalAngle();
    base = imu.quaternion.load().invert();
    pitchIntegrationValid = false; // re-anchor pitchY to 0 at the new base
  } else {
    elbowY.reset();
  }
  setYCalibrating(false);
  return ok;
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
    // engineLoop() retargets to elbowYHomeAngle/base from the *last*
    // successful calibration - if this attempt failed (aborted), those are
    // stale, and engineLoop() would still immediately drive toward them as
    // if they were current. Only proceed on success; otherwise loop back
    // and let the outer gate (sp/pp) decide whether to retry.
    if (elbow->calibrateLoop()) {
      elbow->engineLoop();
    }

    elbow->setEngineTaskStatus(false);
    elbow->updateStatuses();
  }
}

Vector3 ArmElbow::trackTick() {
  Vector3 imuAngles = getIMUAngles();
  Vector3 physicalAngles = getPhysicalAngles(imuAngles);
  if (useIMUMode.load() == USE_IMU_NOT_USE) {
    elbowY.setIMUAngle(elbowY.getPhysicalAngle());
  } else {
    elbowY.setIMUAngle(physicalAngles.y);
  }
  elbowY.tick();
  updateStatuses();
  return physicalAngles;
}

void ArmElbow::engineLoop() {
  LogQueue::Log("Starting Engine Loop\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  elbowY.setTargetAngle(elbowYHomeAngle, 500, ELBOW_DEAD_ZONE);
  while (true) {
    auto sp = imu.isPositionOK();
    auto pp = shoulder.isPositionOK();
    if (!sp || !pp || elbowY.isDiverging()) {
      LogQueue::Log("Engine is turned off (diverging=%d)\n", elbowY.isDiverging());
      elbowY.reset();
      break;
    }

    Vector3 physicalAngles = trackTick();
    angleFromGravityY(); // diagnostic-only call, not used for control here - logs internally
    printer.interval([&]() {
      LogQueue::Log("Y: %.2f, physical: %.2f\n", physicalAngles.y, elbowY.getPhysicalAngle());
    });
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

ArmElbow::ArmElbow(const uint memsSdaPin, const uint memsSclPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineYPin, const uint canRxPin,
                   const uint canTxPin) : ArmPart(canRxPin, canTxPin), elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100),
                                          imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {
  // Mounting correction: maps raw IMU axes onto ArmShoulder's Y/Z convention.
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

  if (frame.id == CAN_USE_IMU) {
    uint8_t mode = frame.data32[0] & 0xFF;
    useIMUMode.store(mode);
    setUseIMUStatus(mode);
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

// Monotonic across the full 0-270 PWM range once the atan2 +/-180 wrap is undone (confirmed via full-range sweep).
constexpr float ELBOW_GRAVITY_UNWRAP_THRESHOLD = -90.0f;
constexpr float ELBOW_GRAVITY_Y_MIN = -35.0f;
constexpr float ELBOW_GRAVITY_Y_MAX = 220.0f;

float ArmElbow::angleFromGravityY() {
  Accelerometer acc = imu.accelerometer.load();
  float raw = atan2(acc.y, acc.z) * RAD_TO_DEG;
  float shoulderCorr = shoulderYAngleDeg();
  float res = raw - shoulderCorr;
  if (res < ELBOW_GRAVITY_UNWRAP_THRESHOLD) {
    res += 360.0f;
  }

  static Periodic gravityPrinter(pdMS_TO_TICKS(300));
  gravityPrinter.interval([&]() {
    LogQueue::Log("[DIAG] angleFromGravityY: raw=%.2f shoulderCorr=%.2f res=%.2f physical=%.2f\n",
                  raw, shoulderCorr, res, elbowY.getPhysicalAngle());
  });

  if (res < ELBOW_GRAVITY_Y_MIN || res > ELBOW_GRAVITY_Y_MAX) {
    return NAN;
  }
  return res;
}

Vector3 ArmElbow::getIMUAngles() {
  Quaternion qm = base * imu.quaternion.load();
  // Yaw is extracted absolutely every tick - matches ArmShoulder, where this
  // never showed the discontinuity described below.
  float yawZ = qm.twistAngle({0.0f, 0.0f, 1.0f});
  Quaternion qZ = Quaternion::AngleAxis(yawZ, 0.0f, 0.0f, 1.0f); // used below for the drift-correction reference

  // pitchY used to be re-derived from scratch every tick as an absolute
  // twist extraction off qSwing - the same pattern that caused ArmShoulder's
  // sustained self-inflicted oscillation (a discrete sign/axis decision
  // every tick can jump discontinuously when the swing axis isn't pinned
  // near +-Y). Ported the same fix here before making useIMU:use elbow's
  // default: accumulate the small swing rotation between consecutive ticks
  // instead - always small/well-defined at our ~5ms tick rate, no discrete
  // choice to get wrong. Anchored at 0 whenever base is freshly set
  // (calibration defines the current orientation as the zero point).
  //
  // Pure integration drifts (random walk from accumulated per-tick noise,
  // confirmed live on ArmShoulder: ~5.7deg over 16s/3200+ ticks while
  // stationary), so also ported ArmShoulder's complementary-filter
  // correction: nudge the accumulator a small fixed fraction toward a fresh
  // absolute extraction each tick, small enough that an occasional bad-sign
  // absolute sample barely perturbs it, but enough to cancel long-term
  // drift within ~1s.
  float pitchY;
  if (!qm.isValid()) {
    // Glitched/missing reading - hold the last known value rather than
    // integrating garbage into the accumulator permanently.
    pitchY = accumulatedPitchY;
  } else if (!pitchIntegrationValid) {
    pitchY = 0.0f;
    accumulatedPitchY = 0.0f;
    pitchIntegrationValid = true;
    lastOrientation = qm;
  } else {
    Quaternion qDelta = lastOrientation.invert() * qm;
    float deltaYawZ = qDelta.twistAngle({0.0f, 0.0f, 1.0f});
    Quaternion qDeltaZ = Quaternion::AngleAxis(deltaYawZ, 0.0f, 0.0f, 1.0f);
    Quaternion qDeltaSwing = qDeltaZ.invert() * qDelta;
    float deltaPitchY = qDeltaSwing.twistAngle({0.0f, 1.0f, 0.0f});
    accumulatedPitchY += deltaPitchY;

    constexpr float driftCorrectionAlpha = 0.01f; // ~1s time constant at 5ms/tick
    Quaternion qSwing = qZ.invert() * qm;
    float pitchYAbs = qSwing.twistAngle({0.0f, 1.0f, 0.0f});
    accumulatedPitchY += driftCorrectionAlpha * (pitchYAbs - accumulatedPitchY);

    pitchY = accumulatedPitchY;
    lastOrientation = qm;
  }

  return {0, pitchY, 0};
}
