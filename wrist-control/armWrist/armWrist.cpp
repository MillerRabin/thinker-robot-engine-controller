#include "armWrist.h"
#include "../common/periodic/periodic.h"
#include "../common/quaternion/quaternion.h"

bool ArmWrist::settled(bool withinTolerance, TickType_t &stableSince) {
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

void ArmWrist::updateStatusLed(bool calibrating) {
  constexpr TickType_t guardTripHoldTicks = pdMS_TO_TICKS(1500);
  if (wristY.isDiverging() || wristZ.isDiverging()) {
    lastGuardTripTick = xTaskGetTickCount();
  }
  bool guardTripRecent = lastGuardTripTick != 0 &&
      (xTaskGetTickCount() - lastGuardTripTick) < guardTripHoldTicks;
  if (guardTripRecent) {
    statusLed.setState(LedState::GuardTripped);
  } else if (!platform.getEnginesPowerStatus()) {
    statusLed.setState(LedState::EnginesDisabled);
  } else if (calibrating) {
    statusLed.setState(LedState::Calibrating);
  } else {
    statusLed.setState(LedState::Off);
  }
}

bool ArmWrist::calibrateYLoop() {
  LogQueue::Log("[DIAG] wrist calibrateYLoop: enter\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setYCalibrating(true);

  // Require stable readings before seeding - a transient first sample caused a jerk at calibration start.
  constexpr int seedStableSamples = 5;
  constexpr float seedStableTolerance = 5.0f; // deg
  float lastSeedSample = NAN;
  int seedStableCount = 0;
  while (seedStableCount < seedStableSamples) {
    updateStatusLed(true);
    if (!elbow.isPositionOK()) {
      LogQueue::Log("Wrist calibrating Y aborted: elbow lost position (seeding)\n");
      setYCalibrating(false);
      return false;
    }
    float sample = angleFromGravityY();
    if (!isnan(sample) && !isnan(lastSeedSample) &&
        fabsf(sample - lastSeedSample) <= seedStableTolerance) {
      seedStableCount++;
    } else {
      seedStableCount = isnan(sample) ? 0 : 1;
    }
    lastSeedSample = sample;
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }

  float imuY = std::clamp(angleFromGravityY(), 0.0f, 270.0f);
  LogQueue::Log("[DIAG] wrist calibrateYLoop: seed physicalY=%.2f (raw gravityY=%.2f)\n", imuY, angleFromGravityY());
  wristY.setDegreeDirect(imuY);
  vTaskDelay(pdMS_TO_TICKS(2000));
  wristY.setTargetAngle(WRIST_Y_HOME_POSITION, WRIST_Y_RECOVERY_TIME_MS, WRIST_DEAD_ZONE);
  TickType_t stableSince = 0;
  uint32_t iterations = 0;
  while (true) {
    updateStatusLed(true);
    if (!elbow.isPositionOK()) {
      LogQueue::Log("Wrist calibrating Y aborted: elbow lost position\n");
      setYCalibrating(false);
      return false;
    }
    // A frozen servo can never satisfy settled(), so this loop needs its own way out.
    if (wristY.isDiverging()) {
      LogQueue::Log("[DIAG] wrist calibrateYLoop: aborted, wristY diverging\n");
      setYCalibrating(false);
      return false;
    }
    float gravityY = angleFromGravityY();
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] wrist calibrateYLoop: iter=%lu gravityY=%.3f physicalY=%.3f\n", (unsigned long)iterations, gravityY, wristY.getPhysicalAngle());
    });
    wristY.setIMUAngle(gravityY);
    wristY.tick();
    updateStatuses();
    if (settled(fabsf(WRIST_Y_HOME_POSITION - gravityY) <= CALIBRATION_SETTLE_TOLERANCE, stableSince)) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  wristYHomeAngle = wristY.getPhysicalAngle();
  setYCalibrating(false);
  LogQueue::Log("[DIAG] wrist calibrateYLoop: exit after %lu iterations, wristYHomeAngle=%.2f\n", (unsigned long)iterations, wristYHomeAngle);
  return true;
}

bool ArmWrist::calibrateZLoop() {
  LogQueue::Log("[DIAG] wrist calibrateZLoop: enter\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setZCalibrating(true);
  float startZ = wristZ.getPhysicalAngle();
  if (isnan(startZ)) {
    startZ = WRIST_Z_HOME_POSITION;
  }
  wristZ.setDegreeDirect(startZ);
  wristZ.setTargetAngle(WRIST_Z_HOME_POSITION, 1000, WRIST_DEAD_ZONE);
  uint32_t iterations = 0;
  while (!wristZ.isPositioned()) {
    updateStatusLed(true);
    if (wristZ.isDiverging()) {
      LogQueue::Log("[DIAG] wrist calibrateZLoop: aborted, wristZ diverging\n");
      setZCalibrating(false);
      return false;
    }
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] wrist calibrateZLoop: iter=%lu physicalZ=%.3f imuZ=%.3f\n", (unsigned long)iterations, wristZ.getPhysicalAngle(), wristZ.getIMUAngle());
    });
    wristZ.setIMUAngle(wristZ.getPhysicalAngle());
    wristZ.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
  setZCalibrating(false);
  LogQueue::Log("[DIAG] wrist calibrateZLoop: exit after %lu iterations\n", (unsigned long)iterations);
  return true;
}

Vector3 ArmWrist::trackTick() {
  Vector3 imuAngles = getIMUAngles();
  Vector3 physicalAngles = getPhysicalAngles(imuAngles);
  if (useIMUMode.load() == USE_IMU_NOT_USE) {
    wristY.setIMUAngle(wristY.getPhysicalAngle());
    wristZ.setIMUAngle(wristZ.getPhysicalAngle());
  } else {
    wristY.setIMUAngle(physicalAngles.y);
    wristZ.setIMUAngle(physicalAngles.z);
  }
  wristY.tick();
  wristZ.tick();
  updateStatuses();
  return physicalAngles;
}

bool ArmWrist::calibrateLoop() {
  LogQueue::Log("[DIAG] wrist calibrateLoop: enter\n");
  setArmCalibrated(false);
  updateStatuses();
  bool ok = calibrateYLoop(); // rough pass: lift off gravity's resting position, safe for Z to move next
  if (ok) ok = calibrateZLoop();
  if (ok) ok = calibrateYLoop(); // Z is home now, so gravityY is no longer skewed by it - refine Y

  if (ok) {
    base.store(imu.quaternion.load().invert());
    pitchIntegrationValid = false; // re-anchor pitchY to 0 at the new base
    // Retarget to wristYHomeAngle (PWM-space) - feedback is PWM-space from here on, target must match.
    wristY.setTargetAngle(wristYHomeAngle, WRIST_Y_RECOVERY_TIME_MS, WRIST_DEAD_ZONE);
    LogQueue::Log("[DIAG] wrist calibrateLoop: base refreshed, starting final settle wait\n");

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t stableSince = 0;
    Periodic printer(pdMS_TO_TICKS(500));
    uint32_t iterations = 0;
    while (true) {
      updateStatusLed(true);
      if (wristY.isDiverging() || wristZ.isDiverging()) {
        LogQueue::Log("[DIAG] wrist calibrateLoop: aborted, servo diverging during settle wait\n");
        ok = false;
        break;
      }
      if (!elbow.isPositionOK()) {
        LogQueue::Log("[DIAG] wrist calibrateLoop: aborted, elbow lost position during settle wait\n");
        ok = false;
        break;
      }
      Vector3 physicalAngles = trackTick();
      iterations++;
      bool withinTolerance =
          fabsf(wristYHomeAngle - physicalAngles.y) <= CALIBRATION_SETTLE_TOLERANCE &&
          fabsf(WRIST_Z_HOME_POSITION - physicalAngles.z) <= CALIBRATION_SETTLE_TOLERANCE;
      printer.interval([&]() {
        LogQueue::Log("[DIAG] wrist calibrateLoop: settle iter=%lu physicalY=%.2f physicalZ=%.2f withinTolerance=%d\n",
               (unsigned long)iterations, physicalAngles.y, physicalAngles.z, withinTolerance);
      });
      if (settled(withinTolerance, stableSince)) {
        break;
      }
      vTaskDelayUntil(&lastWakeTime, taskInterval);
    }
    LogQueue::Log("[DIAG] wrist calibrateLoop: exit after %lu settle iterations\n", (unsigned long)iterations);
  }

  setArmCalibrated(ok);
  updateStatuses();
  return ok;
}

void ArmWrist::onIMUReset() {
  LogQueue::Log("wrist onIMUReset: invalidating base\n");
  base.store(Quaternion());
  pitchIntegrationValid = false;
}

int ArmWrist::updateStatuses() {
  setUseIMUStatus(useIMUMode.load());
  return ArmPart::updateStatuses();
}

void ArmWrist::engineLoop() {
  LogQueue::Log("[DIAG] wrist engineLoop: enter\n");
  wristY.setTargetAngle(wristYHomeAngle, WRIST_Y_RECOVERY_TIME_MS, WRIST_DEAD_ZONE);
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  while (true) {
    updateStatusLed(false);
    auto sp = imu.isPositionOK();
    auto pp = elbow.isPositionOK();
    auto bp = base.load().isValid();
    auto diverging = wristY.isDiverging() || wristZ.isDiverging();
    if (!sp || !pp || !bp || diverging) {
      LogQueue::Log("[DIAG] wrist engineLoop: exit (sp=%d pp=%d bp=%d diverging=%d) physicalY=%.2f physicalZ=%.2f\n", sp, pp, bp, diverging, wristY.getPhysicalAngle(), wristZ.getPhysicalAngle());
      wristY.reset();
      wristZ.stop();
      break;
    }

    Vector3 physicalAngles = trackTick();
    printer.interval([&]() {
      LogQueue::Log("Wrist Y: %.2f, Z: %.2f\n", physicalAngles.y, physicalAngles.z);
    });
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

void ArmWrist::engineTask(void *instance) {
  auto *wrist = static_cast<ArmWrist *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool wasOK = false;

  // Window to attach a serial log reader before calibration's first seed happens.
  vTaskDelay(pdMS_TO_TICKS(3000));

  while (true) {
    auto sp = wrist->imu.isPositionOK();
    auto pp = wrist->elbow.isPositionOK();

    if (!sp || !pp) {
      if (wasOK) {
        LogQueue::Log("wrist engineTask: position NOT OK (sp=%d pp=%d), stopped\n", sp, pp);
        wasOK = false;
      }
      wrist->updateStatusLed(false);
      wrist->setEngineTaskStatus(false);
      wrist->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    Quaternion position = wrist->imu.quaternion.load();
    Quaternion elbowPosition = wrist->elbow.imu.quaternion.load();

    if (!position.isValid() || !elbowPosition.isValid()) {
      if (wasOK) {
        LogQueue::Log("wrist engineTask: quaternion invalid (imu=%d elbow=%d), stopped\n", position.isValid(), elbowPosition.isValid());
        wasOK = false;
      }
      wrist->updateStatusLed(false);
      wrist->setEngineTaskStatus(false);
      wrist->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    LogQueue::Log("[DIAG] wrist engineTask: position OK -> entering calibrateLoop (physicalY=%.2f physicalZ=%.2f)\n", wrist->wristY.getPhysicalAngle(), wrist->wristZ.getPhysicalAngle());
    wasOK = true;
    wrist->setEngineTaskStatus(true);
    // Only proceed on a successful calibration - engineLoop() would otherwise retarget to stale values.
    if (wrist->calibrateLoop()) {
      wrist->engineLoop();
      LogQueue::Log("[DIAG] wrist engineTask: engineLoop returned, back to top\n");
    }
    wrist->setEngineTaskStatus(false);
    wrist->updateStatuses();
  }
}

ArmWrist::ArmWrist(const uint memsSdaPin, const uint memsSclPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineZPin, const uint engineYPin,
                   const uint canRxPin, const uint canTxPin)
    : ArmPart(canRxPin, canTxPin),
      wristZ(engineZPin, Range(0, 270), WRIST_Z_HOME_POSITION, 330),
      wristY(engineYPin, Range(0, 270), WRIST_Y_HOME_POSITION, 330),
      imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {
  // Mounting correction: maps raw IMU axes onto ArmShoulder's Y/Z convention.
  Quaternion q_corr = {0.033409f, -0.063652f, -0.418612f, -0.905316f};
  imu.setRotate(q_corr);
  if (taskHandle == NULL) {
    if (xTaskCreateAffinitySet(ArmWrist::engineTask,
                               "ArmWrist::engineTask", 4096, this, 4,
                               ARM_CORE, &taskHandle) == pdFAIL) {
      printf("Failed to create ArmWrist engine task\n");
    } else {}
  }
}

int ArmWrist::updateQuaternion(IMUBase *position) {
  Quaternion baseQ = base.load();
  if (!baseQ.isValid()) {
    return ERROR_PART_IS_NOT_CALIBRATED;
  }
  Quaternion quat = baseQ * position->quaternion.load();
  return ArmPart::updateQuaternion(quat);
}

int ArmWrist::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmWrist::updateAccelerometer(IMUBase *position) {
  Accelerometer acc = position->accelerometer.load();
  return ArmPart::updateAccelerometer(acc);
}

int ArmWrist::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmWrist::busReceiveCallback(can2040_msg frame) {
  claw.dispatchMessage(frame);
  elbow.dispatchMessage(frame);

  if (frame.id == CAN_WRIST_SET_YZ_DEGREE) {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;

    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
    float angleZ = (angleZS == PARAMETER_IS_NAN) ? NAN : angleZS / 10.0f;
    uint32_t raw1 = frame.data32[1];
    uint16_t timeMS = raw1 & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    if (!isnan(angleY)) {
      wristY.setTargetAngle(angleY, timeMS, WRIST_DEAD_ZONE);
    }
    if (!isnan(angleZ)) {
      wristZ.setTargetAngle(angleZ, timeMS, WRIST_DEAD_ZONE);
    }
  }

  if (frame.id == CAN_USE_IMU) {
    uint8_t mode = frame.data32[0] & 0xFF;
    useIMUMode.store(mode);
    setUseIMUStatus(mode);
  }

  if (frame.id == CAN_WRIST_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}

Vector3 ArmWrist::getIMUAngles() {
  Quaternion qm = base.load() * imu.quaternion.load();
  float yawZ = qm.twistAngle({0.0f, 0.0f, 1.0f});

  // pitchY is delta-integrated (drift-corrected below) rather than re-derived absolutely, matching ArmShoulder's pitchX fix.
  float pitchY;
  if (!qm.isValid()) {
    // Glitched reading - hold the last value instead of integrating NaN permanently.
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

    // Gravity-vector-based, not a yaw-strip-then-pitch decomposition of qm - see
    // ArmShoulder::getIMUAngles()'s identical fix for why (order-sensitive, breaks under a
    // large yaw change).
    constexpr float driftCorrectionAlpha = 0.01f; // ~1s time constant at 5ms/tick
    Vector3 gAbs = qm.getGravityVector();
    float pitchYAbs = atan2(-gAbs.x, gAbs.z);
    accumulatedPitchY += driftCorrectionAlpha * (pitchYAbs - accumulatedPitchY);

    pitchY = accumulatedPitchY;
    lastOrientation = qm;
  }

  static Periodic axisPrinter(pdMS_TO_TICKS(300));
  axisPrinter.interval([&]() {
    LogQueue::Log("[DIAG] wrist pitchY=%.2f yawZ=%.2f\n", pitchY * RAD_TO_DEG, yawZ * RAD_TO_DEG);
  });

  return {0, pitchY, yawZ};
}

Vector3 ArmWrist::getPhysicalAngles(Vector3 &imuAngles) {
  return { 0, (imuAngles.y * RAD_TO_DEG + wristYHomeAngle), (imuAngles.z * RAD_TO_DEG + WRIST_Z_HOME_POSITION) };
}

// elbow.imu.quaternion is elbow's base-anchored broadcast (zero at elbow's own calibration),
// so this tracks elbow's current pitch deviation since then - same pattern as ArmElbow::shoulderYAngleDeg().
// Uses the gravity vector rather than a yaw-strip-then-pitch-extract decomposition of eq - that
// decomposition is order-sensitive and breaks under a large yaw change (confirmed live via the
// same bug in ArmElbow::shoulderYAngleDeg(), which this mirrors). getGravityVector() is
// yaw-invariant, so this can't happen.
float ArmWrist::elbowYAngleDeg() {
  Quaternion eq = elbow.imu.quaternion.load();
  if (!eq.isValid()) {
    return 0.0f;
  }
  Vector3 g = eq.getGravityVector();
  return atan2(-g.x, g.z) * RAD_TO_DEG;
}

// Uses the quaternion's gravity vector rather than raw accelerometer atan2 (shoulder/elbow's
// approach) - unlike accelerometer readings, the quaternion already has q_corr applied, so this
// reuses the mounting correction already verified live instead of needing a fresh raw-axis fit.
// Sign/wrap constants are a starting point - verify live like ELBOW_GRAVITY_* was tuned.
float ArmWrist::angleFromGravityY() {
  Quaternion q = imu.quaternion.load();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  float raw = atan2(-g.x, g.z) * RAD_TO_DEG;
  float elbowCorr = elbowYAngleDeg();
  float res = raw - elbowCorr;

  static Periodic gravityPrinter(pdMS_TO_TICKS(300));
  gravityPrinter.interval([&]() {
    LogQueue::Log("[DIAG] wrist angleFromGravityY: raw=%.2f elbowCorr=%.2f res=%.2f physical=%.2f\n",
                  raw, elbowCorr, res, wristY.getPhysicalAngle());
  });

  return res;
}
