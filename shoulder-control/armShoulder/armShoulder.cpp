#include "armShoulder.h"
#include "../common/periodic/periodic.h"
#include "../common/quaternion/quaternion.h"
#include "../common/speedBuffer/speedBuffer.h"

bool ArmShoulder::settled(bool withinTolerance, TickType_t &stableSince) {
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

void ArmShoulder::updateStatusLed(bool calibrating, bool ready) {
  bool enginesEnabled = platform.getEnginesPowerStatus();
  if (lastEnginesEnabled && !enginesEnabled) {
    onIMUReset();
  }
  lastEnginesEnabled = enginesEnabled;

  constexpr TickType_t guardTripHoldTicks = pdMS_TO_TICKS(1500);
  if (shoulderY.isDiverging() || shoulderZ.isDiverging()) {
    lastGuardTripTick = xTaskGetTickCount();
  }
  bool guardTripRecent = lastGuardTripTick != 0 &&
      (xTaskGetTickCount() - lastGuardTripTick) < guardTripHoldTicks;
  if (guardTripRecent) {
    statusLed.setState(LedState::GuardTripped);
  } else if (!enginesEnabled) {
    statusLed.setState(LedState::EnginesDisabled);
  } else if (calibrating) {
    statusLed.setState(LedState::Calibrating);
  } else if (ready) {
    statusLed.setState(LedState::Ready);
  } else {
    statusLed.setState(LedState::Off);
  }
}

bool ArmShoulder::calibrateYLoop() {
  LogQueue::Log("[DIAG] calibrateYLoop: enter\n");
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

  float imuY = std::clamp(angleFromGravityY(), 0.0f, 180.0f);
  LogQueue::Log("[DIAG] calibrateYLoop: seed physicalY=%.2f (raw gravityY=%.2f)\n", imuY, angleFromGravityY());
  shoulderY.setDegreeDirect(imuY);
  vTaskDelay(pdMS_TO_TICKS(2000));
  LogQueue::Log("[DIAG] calibrateYLoop: post-seed delay done, starting settle loop\n");
  shoulderY.setTargetAngle(SHOULDER_Y_HOME_POSITION, SHOULDER_Y_RECOVERY_TIME_MS, SHOULDER_DEAD_ZONE);
  TickType_t stableSince = 0;
  uint32_t iterations = 0;
  while (true) {
    updateStatusLed(true);
    // A frozen servo can never satisfy settled(), so this loop needs its own way out.
    if (shoulderY.isDiverging()) {
      LogQueue::Log("[DIAG] calibrateYLoop: aborted, shoulderY diverging\n");
      setYCalibrating(false);
      return false;
    }
    // Without this, tick() keeps advancing physicalAngle (PWM bookkeeping) toward closing the
    // error even with no servo power - it then snaps to that stale value once power returns.
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] calibrateYLoop: aborted, engines powered off\n");
      setYCalibrating(false);
      return false;
    }
    float gravityY = angleFromGravityY();
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] calibrateYLoop: iter=%lu gravityY=%.3f physicalY=%.3f\n", (unsigned long)iterations, gravityY, shoulderY.getPhysicalAngle());
    });
    shoulderY.setIMUAngle(gravityY);
    shoulderY.tick();
    updateStatuses();
    if (settled(fabsf(SHOULDER_Y_HOME_POSITION - gravityY) <= CALIBRATION_SETTLE_TOLERANCE, stableSince)) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  shoulderYHomeAngle = shoulderY.getPhysicalAngle();
  setYCalibrating(false);
  LogQueue::Log("[DIAG] calibrateYLoop: exit after %lu iterations, shoulderYHomeAngle=%.2f\n", (unsigned long)iterations, shoulderYHomeAngle);
  return true;
}

bool ArmShoulder::calibrateZLoop() {
  LogQueue::Log("[DIAG] calibrateZLoop: enter\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setZCalibrating(true);
  float startZ = shoulderZ.getPhysicalAngle();
  if (isnan(startZ)) {
    startZ = SHOULDER_Z_HOME_POSITION;
  }
  LogQueue::Log("[DIAG] calibrateZLoop: startZ=%.2f\n", startZ);
  shoulderZ.setDegreeDirect(startZ);
  shoulderZ.setTargetAngle(SHOULDER_Z_HOME_POSITION, 1000, SHOULDER_DEAD_ZONE);
  uint32_t iterations = 0;
  while (!shoulderZ.isPositioned()) {
    updateStatusLed(true);
    if (shoulderZ.isDiverging()) {
      LogQueue::Log("[DIAG] calibrateZLoop: aborted, shoulderZ diverging\n");
      setZCalibrating(false);
      return false;
    }
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] calibrateZLoop: aborted, engines powered off\n");
      setZCalibrating(false);
      return false;
    }
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] calibrateZLoop: iter=%lu physicalZ=%.3f imuZ=%.3f\n", (unsigned long)iterations, shoulderZ.getPhysicalAngle(), shoulderZ.getIMUAngle());
    });
    shoulderZ.setIMUAngle(shoulderZ.getPhysicalAngle());
    shoulderZ.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
  setZCalibrating(false);
  LogQueue::Log("[DIAG] calibrateZLoop: exit after %lu iterations\n", (unsigned long)iterations);
  return true;
}

Vector3 ArmShoulder::trackTick() {
  Vector3 imuAngles = getIMUAngles();
  Vector3 physicalAngles = getPhysicalAngles(imuAngles);
  if (useIMUMode.load() == USE_IMU_NOT_USE) {
    shoulderY.setIMUAngle(shoulderY.getPhysicalAngle());
    shoulderZ.setIMUAngle(shoulderZ.getPhysicalAngle());
  } else {
    shoulderY.setIMUAngle(physicalAngles.y);
    shoulderZ.setIMUAngle(physicalAngles.z);
  }
  shoulderY.tick();
  shoulderZ.tick();
  updateStatuses();
  return physicalAngles;
}

bool ArmShoulder::calibrateLoop() {
  LogQueue::Log("[DIAG] calibrateLoop: enter\n");
  setArmCalibrated(false);
  updateStatuses();
  bool ok = calibrateYLoop(); // rough pass: lift off gravity's resting position, safe for Z to move next
  if (ok) ok = calibrateZLoop();
  if (ok) ok = calibrateYLoop(); // Z is home now, so gravityY is no longer skewed by it - refine Y

  if (ok) {
    base.store(imu.quaternion.load().invert());
    pitchIntegrationValid = false; // re-anchor pitchX to 0 at the new base
    // Retarget to shoulderYHomeAngle (PWM-space) - feedback is PWM-space from here on, target must match.
    shoulderY.setTargetAngle(shoulderYHomeAngle, SHOULDER_Y_RECOVERY_TIME_MS, SHOULDER_DEAD_ZONE);
    LogQueue::Log("[DIAG] calibrateLoop: base refreshed, starting final settle wait\n");

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t stableSince = 0;
    Periodic printer(pdMS_TO_TICKS(500));
    uint32_t iterations = 0;
    while (true) {
      updateStatusLed(true);
      if (shoulderY.isDiverging() || shoulderZ.isDiverging()) {
        LogQueue::Log("[DIAG] calibrateLoop: aborted, servo diverging during settle wait\n");
        ok = false;
        break;
      }
      if (!platform.getEnginesPowerStatus()) {
        LogQueue::Log("[DIAG] calibrateLoop: aborted, engines powered off during settle wait\n");
        ok = false;
        break;
      }
      Vector3 physicalAngles = trackTick();
      iterations++;
      bool withinTolerance =
          fabsf(shoulderYHomeAngle - physicalAngles.y) <= CALIBRATION_SETTLE_TOLERANCE &&
          fabsf(SHOULDER_Z_HOME_POSITION - physicalAngles.z) <= CALIBRATION_SETTLE_TOLERANCE;
      printer.interval([&]() {
        LogQueue::Log("[DIAG] calibrateLoop: settle iter=%lu physicalY=%.2f physicalZ=%.2f withinTolerance=%d\n",
               (unsigned long)iterations, physicalAngles.y, physicalAngles.z, withinTolerance);
      });
      if (settled(withinTolerance, stableSince)) {
        break;
      }
      vTaskDelayUntil(&lastWakeTime, taskInterval);
    }
    LogQueue::Log("[DIAG] calibrateLoop: exit after %lu settle iterations\n", (unsigned long)iterations);
  }

  setArmCalibrated(ok);
  updateStatuses();
  return ok;
}

void ArmShoulder::onIMUReset() {
  LogQueue::Log("onIMUReset: invalidating base\n");
  base.store(Quaternion());
  pitchIntegrationValid = false;
}

int ArmShoulder::updateStatuses() {
  setUseIMUStatus(useIMUMode.load());
  return ArmPart::updateStatuses();
}

void ArmShoulder::engineLoop() {
  LogQueue::Log("[DIAG] engineLoop: enter\n");
  shoulderY.setTargetAngle(shoulderYHomeAngle, SHOULDER_Y_RECOVERY_TIME_MS, SHOULDER_DEAD_ZONE);
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  while (true) {
    updateStatusLed(false, true);
    auto sp = imu.isPositionOK();
    auto pp = platform.isPositionOK();
    auto bp = base.load().isValid();
    auto diverging = shoulderY.isDiverging() || shoulderZ.isDiverging();
    if (!sp || !pp || !bp || diverging) {
      LogQueue::Log("[DIAG] engineLoop: exit (sp=%d pp=%d bp=%d diverging=%d) physicalY=%.2f physicalZ=%.2f\n", sp, pp, bp, diverging, shoulderY.getPhysicalAngle(), shoulderZ.getPhysicalAngle());
      shoulderY.reset();
      shoulderZ.stop();
      break;
    }

    Vector3 physicalAngles = trackTick();
    printer.interval([&]() {
      LogQueue::Log("Y: %.2f, Z: %.2f\n", physicalAngles.y, physicalAngles.z);
    });
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool wasOK = false;

  // Window to attach a serial log reader before calibration's first seed happens.
  vTaskDelay(pdMS_TO_TICKS(3000));

  while (true) {
    auto sp = shoulder->imu.isPositionOK();
    auto pp = shoulder->platform.isPositionOK();

    if (!sp || !pp) {
      if (wasOK) {
        LogQueue::Log("engineTask: position NOT OK (sp=%d pp=%d), stopped\n", sp, pp);
        wasOK = false;
      }
      shoulder->updateStatusLed(false);
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    auto position = shoulder->imu.quaternion.load();
    auto platformPosition = shoulder->platform.imu.quaternion.load();

    if (!position.isValid() || !platformPosition.isValid()) {
      if (wasOK) {
        LogQueue::Log("engineTask: quaternion invalid (imu=%d platform=%d), stopped\n", position.isValid(), platformPosition.isValid());
        wasOK = false;
      }
      shoulder->updateStatusLed(false);
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    LogQueue::Log("[DIAG] engineTask: position OK -> entering calibrateLoop (physicalY=%.2f physicalZ=%.2f)\n", shoulder->shoulderY.getPhysicalAngle(), shoulder->shoulderZ.getPhysicalAngle());
    wasOK = true;
    shoulder->setEngineTaskStatus(true);
    // Only proceed on a successful calibration - engineLoop() would otherwise retarget to stale values.
    if (shoulder->calibrateLoop()) {
      shoulder->engineLoop();
      LogQueue::Log("[DIAG] engineTask: engineLoop returned, back to top\n");
    }
    shoulder->setEngineTaskStatus(false);
    shoulder->updateStatuses();
  }
}

ArmShoulder::ArmShoulder(uint memsSCKPin, uint memsMISOPin, uint memsMOSIPin,
                         uint memsCSPin, uint memsIntPin, uint memsRstPin,
                         uint engineZPin, uint engineYPin, uint canRxPin,
                         uint canTxPin)
    : ArmPart(canRxPin, canTxPin),
      shoulderZ(engineZPin, Range(0, 270), SHOULDER_Z_HOME_POSITION, 330),
      shoulderY(engineYPin, Range(0, 180), SHOULDER_Y_HOME_POSITION, 330,
                0.00055),
      imu(this, memsSCKPin, memsMISOPin, memsMOSIPin, memsCSPin, memsRstPin,
          memsIntPin) {
    Quaternion q_corr = {0.7071068f, 0.0f, 0.0f, 0.7071068f};
    imu.setRotate(q_corr);
}

int ArmShoulder::updateQuaternion(IMUBase *position) {
  Quaternion baseQ = base.load();
  if (!baseQ.isValid()) {
    return ERROR_PART_IS_NOT_CALIBRATED;
  }
  Quaternion quat = baseQ * position->quaternion.load();
  return ArmPart::updateQuaternion(quat);
}

int ArmShoulder::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmShoulder::updateAccelerometer(IMUBase *position) {
  Accelerometer acc = position->accelerometer.load();
  return ArmPart::updateAccelerometer(acc);
}

int ArmShoulder::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmShoulder::busReceiveCallback(can2040_msg frame) {

  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
    float angleZ = (angleZS == PARAMETER_IS_NAN) ? NAN : angleZS / 10.0f;
    uint32_t raw1 = frame.data32[1];
    uint16_t timeMS = raw1 & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    if (!isnan(angleY)) {
      shoulderY.setTargetAngle(angleY, timeMS, SHOULDER_DEAD_ZONE);
    }
    if (!isnan(angleZ)) {
      shoulderZ.setTargetAngle(angleZ, timeMS, SHOULDER_DEAD_ZONE);
    }
  }

  if (frame.id == CAN_USE_IMU) {
    uint8_t mode = frame.data32[0] & 0xFF;
    useIMUMode.store(mode);
    setUseIMUStatus(mode);
  }

  if (frame.id == CAN_SHOULDER_FIRMWARE_UPGRADE) {
    ArmPart::sendFirmwareUpgradeMessage();
    setUpgrading(true);
    updateStatuses();
    vTaskDelay(pdMS_TO_TICKS(1000));
    rebootInBootMode();
  }
}

int ArmShoulder::begin() {
  if (taskHandle == NULL) {
    if (xTaskCreateAffinitySet(ArmShoulder::engineTask,
                               "ArmShoulder::engineTask", 4096, this, 4,
                               ARM_CORE, &taskHandle) == pdFAIL) {
      printf("Failed to create ArmShoulder engine task\n");
      return ERROR_ENGINE_TASK_CREATION_FAILED;
    } else {}
  }
  return 0;
}

Vector3 ArmShoulder::getIMUAngles() {
  Quaternion qm = base.load() * imu.quaternion.load();
  float yawZ = qm.twistAngle({0.0f, 0.0f, 1.0f});

  // pitchX is delta-integrated (drift-corrected below) rather than re-derived absolutely, which caused ~11deg one-tick jumps.
  float pitchX;
  if (!qm.isValid()) {
    // Glitched reading - hold the last value instead of integrating NaN permanently.
    pitchX = accumulatedPitchX;
  } else if (!pitchIntegrationValid) {
    pitchX = 0.0f;
    accumulatedPitchX = 0.0f;
    pitchIntegrationValid = true;
    lastOrientation = qm;
  } else {
    Quaternion qDelta = lastOrientation.invert() * qm;
    float deltaYawZ = qDelta.twistAngle({0.0f, 0.0f, 1.0f});
    Quaternion qDeltaZ = Quaternion::AngleAxis(deltaYawZ, 0.0f, 0.0f, 1.0f);
    Quaternion qDeltaSwing = qDeltaZ.invert() * qDelta;
    float deltaPitchX = qDeltaSwing.twistAngle({0.0f, 1.0f, 0.0f});
    accumulatedPitchX += deltaPitchX;

    // Gravity-vector-based, not a yaw-strip-then-pitch decomposition of qm - that decomposition
    // is order-sensitive and breaks under a large yaw change (confirmed live: the same pattern
    // in ArmElbow::shoulderYAngleDeg() got stuck outputting ~-121deg after a 30deg shoulder-z
    // move, corrupting elbow's target; this driftCorrectionAlpha-weighted term would have fed
    // the same kind of bad value into the accumulator here every tick, growing into a real
    // oscillation). getGravityVector() is yaw-invariant, so this can't happen.
    constexpr float driftCorrectionAlpha = 0.01f; // ~1s time constant at 5ms/tick
    Vector3 gAbs = qm.getGravityVector();
    float pitchXAbs = atan2(-gAbs.x, gAbs.z);
    accumulatedPitchX += driftCorrectionAlpha * (pitchXAbs - accumulatedPitchX);

    pitchX = accumulatedPitchX;
    lastOrientation = qm;
  }

  static Periodic axisPrinter(pdMS_TO_TICKS(300));
  axisPrinter.interval([&]() {
    LogQueue::Log("[DIAG] pitchX=%.2f yawZ=%.2f\n", pitchX * RAD_TO_DEG, yawZ * RAD_TO_DEG);
  });

  return {0, pitchX, yawZ};
}

Vector3 ArmShoulder::getPhysicalAngles(Vector3 &imuAngles) {
  return { 0, (imuAngles.y * RAD_TO_DEG + shoulderYHomeAngle), (imuAngles.z * RAD_TO_DEG + SHOULDER_Z_HOME_POSITION) };
}

float ArmShoulder::angleFromGravityY() {
  Accelerometer acc = imu.accelerometer.load();  
  float res = -1 * atan2(acc.y, acc.x) * RAD_TO_DEG;
  if (res < -45) {
    res = 360 + res;
  }
  return res;
}
