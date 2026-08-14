#include "armClaw.h"
#include "../common/periodic/periodic.h"
#include "../common/quaternion/quaternion.h"

bool ArmClaw::settled(bool withinTolerance, TickType_t &stableSince) {
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

void ArmClaw::updateStatusLed(bool calibrating, bool ready) {
  bool enginesEnabled = platform.getEnginesPowerStatus();
  if (lastEnginesEnabled && !enginesEnabled) {
    onIMUReset();
  }
  lastEnginesEnabled = enginesEnabled;

  constexpr TickType_t guardTripHoldTicks = pdMS_TO_TICKS(1500);
  if (clawX.isDiverging() || clawY.isDiverging() || clawGripper.isDiverging()) {
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

bool ArmClaw::calibrateXLoop() {
  LogQueue::Log("[DIAG] claw calibrateXLoop: enter\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setXCalibrating(true);

  // Require stable readings before seeding - a transient first sample caused a jerk elsewhere.
  constexpr int seedStableSamples = 5;
  constexpr float seedStableTolerance = 5.0f; // deg
  float lastSeedSample = NAN;
  int seedStableCount = 0;
  while (seedStableCount < seedStableSamples) {
    updateStatusLed(true);
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] claw calibrateXLoop: aborted, engines powered off (seeding)\n");
      setXCalibrating(false);
      return false;
    }
    float sample = angleFromGravityX();
    if (!isnan(sample) && !isnan(lastSeedSample) &&
        fabsf(sample - lastSeedSample) <= seedStableTolerance) {
      seedStableCount++;
    } else {
      seedStableCount = isnan(sample) ? 0 : 1;
    }
    lastSeedSample = sample;
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }

  float imuX = std::clamp(angleFromGravityX(), 0.0f, 270.0f);
  LogQueue::Log("[DIAG] claw calibrateXLoop: seed physicalX=%.2f (raw gravityX=%.2f)\n", imuX, angleFromGravityX());
  clawX.setDegreeDirect(imuX);
  vTaskDelay(pdMS_TO_TICKS(2000));
  clawX.setTargetAngle(CLAW_X_HOME_POSITION, 1000, CLAW_DEAD_ZONE);
  TickType_t stableSince = 0;
  uint32_t iterations = 0;
  while (true) {
    updateStatusLed(true);
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] claw calibrateXLoop: aborted, engines powered off\n");
      setXCalibrating(false);
      return false;
    }
    // A frozen servo can never satisfy settled(), so this loop needs its own way out.
    if (clawX.isDiverging()) {
      LogQueue::Log("[DIAG] claw calibrateXLoop: aborted, clawX diverging\n");
      setXCalibrating(false);
      return false;
    }
    float gravityX = angleFromGravityX();
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] claw calibrateXLoop: iter=%lu gravityX=%.3f physicalX=%.3f\n", (unsigned long)iterations, gravityX, clawX.getPhysicalAngle());
    });
    clawX.setIMUAngle(gravityX);
    clawX.tick();
    updateStatuses();
    if (settled(fabsf(CLAW_X_HOME_POSITION - gravityX) <= CALIBRATION_SETTLE_TOLERANCE, stableSince)) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  clawXHomeAngle = clawX.getPhysicalAngle();
  setXCalibrating(false);
  LogQueue::Log("[DIAG] claw calibrateXLoop: exit after %lu iterations, clawXHomeAngle=%.2f\n", (unsigned long)iterations, clawXHomeAngle);
  return true;
}

bool ArmClaw::calibrateYLoop() {
  LogQueue::Log("[DIAG] claw calibrateYLoop: enter\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setYCalibrating(true);

  constexpr int seedStableSamples = 5;
  constexpr float seedStableTolerance = 5.0f; // deg
  float lastSeedSample = NAN;
  int seedStableCount = 0;
  while (seedStableCount < seedStableSamples) {
    updateStatusLed(true);
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] claw calibrateYLoop: aborted, engines powered off (seeding)\n");
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

  float imuY = std::clamp(angleFromGravityY(), 0.0f, 180.0f);
  LogQueue::Log("[DIAG] claw calibrateYLoop: seed physicalY=%.2f (raw gravityY=%.2f)\n", imuY, angleFromGravityY());
  clawY.setDegreeDirect(imuY);
  vTaskDelay(pdMS_TO_TICKS(2000));
  clawY.setTargetAngle(CLAW_Y_HOME_POSITION, 1000, CLAW_DEAD_ZONE);
  TickType_t stableSince = 0;
  uint32_t iterations = 0;
  while (true) {
    updateStatusLed(true);
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] claw calibrateYLoop: aborted, engines powered off\n");
      setYCalibrating(false);
      return false;
    }
    if (clawY.isDiverging()) {
      LogQueue::Log("[DIAG] claw calibrateYLoop: aborted, clawY diverging\n");
      setYCalibrating(false);
      return false;
    }
    float gravityY = angleFromGravityY();
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] claw calibrateYLoop: iter=%lu gravityY=%.3f physicalY=%.3f\n", (unsigned long)iterations, gravityY, clawY.getPhysicalAngle());
    });
    clawY.setIMUAngle(gravityY);
    clawY.tick();
    updateStatuses();
    if (settled(fabsf(CLAW_Y_HOME_POSITION - gravityY) <= CALIBRATION_SETTLE_TOLERANCE, stableSince)) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  clawYHomeAngle = clawY.getPhysicalAngle();
  setYCalibrating(false);
  LogQueue::Log("[DIAG] claw calibrateYLoop: exit after %lu iterations, clawYHomeAngle=%.2f\n", (unsigned long)iterations, clawYHomeAngle);
  return true;
}

Vector3 ArmClaw::trackTick() {
  Vector3 imuAngles = getIMUAngles();
  Vector3 physicalAngles = getPhysicalAngles(imuAngles);
  if (useIMUMode.load() == USE_IMU_NOT_USE) {
    clawX.setIMUAngle(clawX.getPhysicalAngle());
    clawY.setIMUAngle(clawY.getPhysicalAngle());
  } else {
    clawX.setIMUAngle(physicalAngles.x);
    clawY.setIMUAngle(physicalAngles.y);
  }
  clawX.tick();
  clawY.tick();
  // Gripper isn't IMU-tracked (open/close mechanism, not an orientation axis) - self-referential
  // so setTargetAngle() commands still ramp via tick().
  clawGripper.setIMUAngle(clawGripper.getPhysicalAngle());
  clawGripper.tick();
  updateStatuses();
  return physicalAngles;
}

bool ArmClaw::calibrateLoop() {
  LogQueue::Log("[DIAG] claw calibrateLoop: enter\n");
  setArmCalibrated(false);
  updateStatuses();
  bool ok = calibrateXLoop(); // rough pass: lift off gravity's resting position, safe for Y to move next
  if (ok) ok = calibrateYLoop();
  if (ok) ok = calibrateXLoop(); // Y is home now, so gravityX is no longer skewed by it - refine X

  if (ok) {
    base.store(correctedQuat().invert());
    pitchIntegrationValid = false; // re-anchor pitchX/pitchY to 0 at the new base
    // Retarget to the calibrated home angles (PWM-space) - feedback is PWM-space from here on.
    clawX.setTargetAngle(clawXHomeAngle, 1000, CLAW_DEAD_ZONE);
    clawY.setTargetAngle(clawYHomeAngle, 1000, CLAW_DEAD_ZONE);
    LogQueue::Log("[DIAG] claw calibrateLoop: base refreshed, starting final settle wait\n");

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t stableSince = 0;
    Periodic printer(pdMS_TO_TICKS(500));
    uint32_t iterations = 0;
    while (true) {
      updateStatusLed(true);
      if (clawX.isDiverging() || clawY.isDiverging()) {
        LogQueue::Log("[DIAG] claw calibrateLoop: aborted, servo diverging during settle wait\n");
        ok = false;
        break;
      }
      if (!wrist.isPositionOK()) {
        LogQueue::Log("[DIAG] claw calibrateLoop: aborted, wrist lost position during settle wait\n");
        ok = false;
        break;
      }
      if (!platform.getEnginesPowerStatus()) {
        LogQueue::Log("[DIAG] claw calibrateLoop: aborted, engines powered off during settle wait\n");
        ok = false;
        break;
      }
      Vector3 physicalAngles = trackTick();
      iterations++;
      bool withinTolerance =
          fabsf(clawXHomeAngle - physicalAngles.x) <= CALIBRATION_SETTLE_TOLERANCE &&
          fabsf(clawYHomeAngle - physicalAngles.y) <= CALIBRATION_SETTLE_TOLERANCE;
      printer.interval([&]() {
        LogQueue::Log("[DIAG] claw calibrateLoop: settle iter=%lu physicalX=%.2f physicalY=%.2f withinTolerance=%d\n",
               (unsigned long)iterations, physicalAngles.x, physicalAngles.y, withinTolerance);
      });
      if (settled(withinTolerance, stableSince)) {
        break;
      }
      vTaskDelayUntil(&lastWakeTime, taskInterval);
    }
    LogQueue::Log("[DIAG] claw calibrateLoop: exit after %lu settle iterations\n", (unsigned long)iterations);
  }

  setArmCalibrated(ok);
  updateStatuses();
  return ok;
}

void ArmClaw::onIMUReset() {
  LogQueue::Log("claw onIMUReset: invalidating base\n");
  base.store(Quaternion());
  pitchIntegrationValid = false;
}

int ArmClaw::updateStatuses() {
  setUseIMUStatus(useIMUMode.load());
  return ArmPart::updateStatuses();
}

void ArmClaw::engineLoop() {
  LogQueue::Log("[DIAG] claw engineLoop: enter\n");
  clawX.setTargetAngle(clawXHomeAngle, 1000, CLAW_DEAD_ZONE);
  clawY.setTargetAngle(clawYHomeAngle, 1000, CLAW_DEAD_ZONE);
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  while (true) {
    updateStatusLed(false, true);
    auto sp = imu.isPositionOK();
    auto pp = wrist.isPositionOK();
    auto bp = base.load().isValid();
    auto ep = platform.getEnginesPowerStatus();
    auto diverging = clawX.isDiverging() || clawY.isDiverging();
    if (!sp || !pp || !bp || !ep || diverging) {
      LogQueue::Log("[DIAG] claw engineLoop: exit (sp=%d pp=%d bp=%d ep=%d diverging=%d) physicalX=%.2f physicalY=%.2f\n", sp, pp, bp, ep, diverging, clawX.getPhysicalAngle(), clawY.getPhysicalAngle());
      clawX.reset();
      clawY.reset();
      break;
    }

    Vector3 physicalAngles = trackTick();
    printer.interval([&]() {
      LogQueue::Log("Claw X: %.2f, Y: %.2f\n", physicalAngles.x, physicalAngles.y);
    });
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

void ArmClaw::engineTask(void *instance) {
  auto *claw = static_cast<ArmClaw *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool wasOK = false;

  // Window to attach a serial log reader before calibration's first seed happens.
  vTaskDelay(pdMS_TO_TICKS(3000));

  while (true) {
    auto sp = claw->imu.isPositionOK();
    auto pp = claw->wrist.isPositionOK();

    if (!sp || !pp) {
      if (wasOK) {
        LogQueue::Log("claw engineTask: position NOT OK (sp=%d pp=%d), stopped\n", sp, pp);
        wasOK = false;
      }
      claw->updateStatusLed(false);
      claw->setEngineTaskStatus(false);
      claw->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    Quaternion position = claw->imu.quaternion.load();
    Quaternion wristPosition = claw->wrist.imu.quaternion.load();

    if (!position.isValid() || !wristPosition.isValid()) {
      if (wasOK) {
        LogQueue::Log("claw engineTask: quaternion invalid (imu=%d wrist=%d), stopped\n", position.isValid(), wristPosition.isValid());
        wasOK = false;
      }
      claw->updateStatusLed(false);
      claw->setEngineTaskStatus(false);
      claw->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    LogQueue::Log("[DIAG] claw engineTask: position OK -> entering calibrateLoop (physicalX=%.2f physicalY=%.2f)\n", claw->clawX.getPhysicalAngle(), claw->clawY.getPhysicalAngle());
    wasOK = true;
    claw->setEngineTaskStatus(true);
    // Only proceed on a successful calibration - engineLoop() would otherwise retarget to stale values.
    if (claw->calibrateLoop()) {
      claw->engineLoop();
      LogQueue::Log("[DIAG] claw engineTask: engineLoop returned, back to top\n");
    }
    claw->setEngineTaskStatus(false);
    claw->updateStatuses();
  }
}

ArmClaw::ArmClaw(const uint8_t detectorsSdaPin, const uint8_t detectorsSclPin,
                 const uint8_t engineXPin, const uint8_t engineYPin,
                 const uint8_t engineGripperPin, const uint8_t canRxPin,
                 const uint8_t canTxPin, const uint8_t memsRxPin,
                 const uint8_t memsTxPin, const uint8_t memsRstPin,
                 const uint8_t memsIntPin, const uint8_t shortDetectorShutPin,
                 const uint8_t longDetectorShutPin)
    : ArmPart(canRxPin, canTxPin),
      // clawX's real range is 0-270 (135 is the midpoint) - was Range(0,180), inherited from the
      // old dead code, which made the Servo class's PWM-to-degree mapping 1.5x too coarse
      // (confirmed live: gravity-measured rotation was consistently ~1.44-1.5x the commanded
      // degrees for the same PWM change).
      clawX(engineXPin, Range(0, 270), CLAW_X_HOME_POSITION, 100),
      clawY(engineYPin, Range(0, 180), CLAW_Y_HOME_POSITION, 100),
      clawGripper(engineGripperPin, Range(0, 180), CLAW_GRIPPER_HOME_POSITION,
                  100),
      imu(this, memsRxPin, memsTxPin, memsRstPin),
      rangeDetector(this, i2c1, longDetectorShutPin, shortDetectorShutPin) {
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(detectorsSdaPin, GPIO_FUNC_I2C);
  gpio_set_function(detectorsSclPin, GPIO_FUNC_I2C);
  gpio_pull_up(detectorsSdaPin);
  gpio_pull_up(detectorsSclPin);
  bi_decl(bi_2pins_with_func(DETECTORS_SDA_PIN, DETECTORS_SCL_PIN, GPIO_FUNC_I2C));

  if (taskHandle == NULL) {
    if (xTaskCreateAffinitySet(ArmClaw::engineTask, "ArmClaw::engineTask",
                               4096, this, 4, ARM_CORE,
                               &taskHandle) == pdFAIL) {
      printf("Failed to create ArmClaw engine task\n");
    } else {
    }
  }
}

int ArmClaw::updateQuaternion(IMUBase *position) {
  Quaternion baseQ = base.load();
  if (!baseQ.isValid()) {
    return ERROR_PART_IS_NOT_CALIBRATED;
  }
  Quaternion quat = baseQ * correctedQuat();
  return ArmPart::updateQuaternion(quat);
}

int ArmClaw::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmClaw::updateAccelerometer(IMUBase *position) {
  Accelerometer acc = position->accelerometer.load();
  return ArmPart::updateAccelerometer(acc);
}

int ArmClaw::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

int ArmClaw::updateHeight(IMUBase *position) {
  return ArmPart::updateHeight(position->height, position->temperature);
}

void ArmClaw::busReceiveCallback(can2040_msg frame) {
  wrist.dispatchMessage(frame);

  if (frame.id == CAN_CLAW_SET_XYG_DEGREE) {
    uint32_t raw1 = frame.data32[0];
    uint32_t raw2 = frame.data32[1];
    uint16_t angleYS = raw1 & 0xFFFF;
    uint16_t angleXS = (raw1 >> 16) & 0xFFFF;
    uint16_t angleGS = raw2 & 0xFFFF;
    uint16_t timeMS = (raw2 >> 16) & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    // Was /100.0f - host's getAngle() encodes at x10 (same convention as shoulder/elbow/wrist),
    // so this silently produced a 10x-too-small angle. Never noticed because nothing called
    // tick() yet to make it observable.
    float angleX = (angleXS == PARAMETER_IS_NAN) ? NAN : angleXS / 10.0f;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
    float angleG = (angleGS == PARAMETER_IS_NAN) ? NAN : angleGS / 10.0f;

    if (!isnan(angleX)) {
      clawX.setTargetAngle(angleX, timeMS, CLAW_DEAD_ZONE);
    }
    if (!isnan(angleY)) {
      clawY.setTargetAngle(angleY, timeMS, CLAW_DEAD_ZONE);
    }
    if (!isnan(angleG)) {
      clawGripper.setTargetAngle(angleG, timeMS, CLAW_DEAD_ZONE);
    }
  }

  if (frame.id == CAN_TARE) {
    uint32_t raw = frame.data32[0];
    uint16_t clearMask = raw & 0xFFFF;
    uint16_t tareMask = (raw >> 16) & 0xFFFF;

    if (clearMask & ARM_CLAW) {
      // this->imu.clearTare();
    }
    if (tareMask & ARM_CLAW) {
      this->imu.tare();
      // this->imu.saveTare();
    }
  }

  if (frame.id == CAN_USE_IMU) {
    uint8_t mode = frame.data32[0] & 0xFF;
    useIMUMode.store(mode);
    setUseIMUStatus(mode);
  }

  if (frame.id == CAN_CLAW_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}

Vector3 ArmClaw::getIMUAngles() {
  Quaternion qm = base.load() * correctedQuat();

  float pitchX, pitchY;
  if (!qm.isValid()) {
    // Glitched reading - hold the last value instead of integrating NaN permanently.
    pitchX = accumulatedPitchX;
    pitchY = accumulatedPitchY;
  } else if (!pitchIntegrationValid) {
    pitchX = 0.0f;
    pitchY = 0.0f;
    accumulatedPitchX = 0.0f;
    accumulatedPitchY = 0.0f;
    pitchIntegrationValid = true;
    lastOrientation = qm;
  } else {
    // Delta-integrated rather than re-derived absolutely each tick - matches
    // ArmShoulder/ArmElbow/ArmWrist's fix for the yaw-decomposition-order bug (a sequential
    // strip-one-axis-then-extract-the-other decomposition is order-sensitive and breaks under a
    // large single-tick change, but is safe for the small delta between consecutive ticks).
    Quaternion qDelta = lastOrientation.invert() * qm;
    float deltaPitchX = qDelta.twistAngle({1.0f, 0.0f, 0.0f});
    Quaternion qDeltaX = Quaternion::AngleAxis(deltaPitchX, 1.0f, 0.0f, 0.0f);
    Quaternion qDeltaSwing = qDeltaX.invert() * qDelta;
    float deltaPitchY = qDeltaSwing.twistAngle({0.0f, 1.0f, 0.0f});
    accumulatedPitchX += deltaPitchX;
    accumulatedPitchY += deltaPitchY;

    // Gravity-vector-based, not a decomposition of qm itself - see ArmShoulder::getIMUAngles()'s
    // identical fix for why (order-sensitive, breaks under a large orientation change).
    constexpr float driftCorrectionAlpha = 0.01f; // ~1s time constant at 5ms/tick
    Vector3 gAbs = qm.getGravityVector();
    float pitchXAbs = atan2(gAbs.y, gAbs.z);
    float pitchYAbs = atan2(-gAbs.x, gAbs.z);
    accumulatedPitchX += driftCorrectionAlpha * (pitchXAbs - accumulatedPitchX);
    accumulatedPitchY += driftCorrectionAlpha * (pitchYAbs - accumulatedPitchY);

    pitchX = accumulatedPitchX;
    pitchY = accumulatedPitchY;
    lastOrientation = qm;
  }

  static Periodic axisPrinter(pdMS_TO_TICKS(300));
  axisPrinter.interval([&]() {
    LogQueue::Log("[DIAG] claw pitchX=%.2f pitchY=%.2f\n", pitchX * RAD_TO_DEG, pitchY * RAD_TO_DEG);
  });

  return {pitchX, pitchY, 0};
}

Vector3 ArmClaw::getPhysicalAngles(Vector3 &imuAngles) {
  return { (imuAngles.x * RAD_TO_DEG + clawXHomeAngle), (imuAngles.y * RAD_TO_DEG + clawYHomeAngle), 0 };
}

// Uses the quaternion's gravity vector (yaw-invariant) rather than a strip-then-extract
// decomposition of the absolute orientation - see getIMUAngles()'s comment for why. Sign/axis
// pairing verified live (2026-08-14): commanding clawX from 45->90 produced a positive change
// here, matching increasing physical angle.
float ArmClaw::angleFromGravityX() {
  Quaternion q = correctedQuat();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  float raw = atan2(g.y, g.z) * RAD_TO_DEG;

  static Periodic gravityPrinter(pdMS_TO_TICKS(300));
  gravityPrinter.interval([&]() {
    LogQueue::Log("[DIAG] claw angleFromGravityX: raw=%.2f physical=%.2f\n", raw, clawX.getPhysicalAngle());
  });

  return raw;
}

// Sign/axis pairing verified live (2026-08-14): commanding clawY from 90->130 produced a
// positive change here, matching increasing physical angle.
float ArmClaw::angleFromGravityY() {
  Quaternion q = correctedQuat();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  float raw = atan2(-g.x, g.z) * RAD_TO_DEG;

  static Periodic gravityPrinter(pdMS_TO_TICKS(300));
  gravityPrinter.interval([&]() {
    LogQueue::Log("[DIAG] claw angleFromGravityY: raw=%.2f physical=%.2f\n", raw, clawY.getPhysicalAngle());
  });

  return raw;
}
