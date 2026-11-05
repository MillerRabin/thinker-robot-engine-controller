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

void ArmWrist::updateStatusLed(bool calibrating, bool ready) {
  bool enginesEnabled = platform.getEnginesPowerStatus();
  if (lastEnginesEnabled && !enginesEnabled) {
    onIMUReset();
  }
  lastEnginesEnabled = enginesEnabled;
  // The one place servos go limp: everywhere else reset() keeps holding (see Servo::reset()). Not on
  // the edge above - that also fires when platform status just stops arriving.
  if (platform.enginesOffEdge()) {
    wristY.release();
    wristZ.release();
  }

  if (!enginesEnabled) {
    statusLed.setState(LedState::EnginesDisabled);
  } else if (calibrating) {
    statusLed.setState(LedState::Calibrating);
  } else if (ready) {
    statusLed.setState(LedState::Ready);
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
    // Matching ArmShoulder::calibrateYLoop()'s identical fix: elbow.isPositionOK() alone doesn't
    // reliably/promptly reflect engines-off, so a drop here went undetected until the settle loop
    // below, by which point setDegreeDirect() (an unramped, instant PWM command) may already have
    // committed to a stale/mid-transition seed.
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("Wrist calibrating Y aborted: engines powered off (seeding)\n");
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

  // Reuse lastSeedSample (guaranteed non-NaN and stable) instead of a fresh angleFromGravityY()
  // call - matching ArmShoulder::calibrateYLoop()'s identical fix. A fresh call can race into the
  // glitch filter's own rejection and return NaN, and std::clamp() on NaN is unspecified/
  // implementation-defined rather than being caught.
  // Mapped into PWM-space through wristYHomeAngle (the PWM the last calibration actually held at
  // gravity's 90), not used as-is: the servo holds vertical at PWM ~134-140, so seeding PWM = the
  // gravity reading commanded it ~45deg away from where it physically was - a real jerk at every
  // seed (confirmed live 2026-10-31). Clamped to the safe range so setDegreeDirect() can't reject it.
  // A servo still holding (engines never went off) is seeded where it already is instead.
  bool holding = wristY.isHolding();
  float seedY = holding ? wristY.getPhysicalAngle()
                        : std::clamp(wristYHomeAngle + (lastSeedSample - WRIST_Y_HOME_POSITION), wristY.safeMinDegree, wristY.safeMaxDegree);
  LogQueue::Log("[DIAG] wrist calibrateYLoop: seed physicalY=%.2f (raw gravityY=%.2f, home=%.2f, holding=%d)\n", seedY, lastSeedSample, wristYHomeAngle, holding);
  wristY.setDegreeDirect(seedY);

  // Same power check during the post-seed settle delay - matching ArmShoulder::calibrateYLoop().
  for (int i = 0; i < 20; i++) {
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("Wrist calibrating Y aborted: engines powered off (post-seed delay)\n");
      setYCalibrating(false);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Recovery time scaled to the actual seed error, not the fixed WRIST_Y_RECOVERY_TIME_MS constant -
  // matching ArmShoulder::calibrateYLoop()'s identical fix. A fixed short recovery time makes the
  // speed formula (error/timeLeft) demand near-maxAngularSpeed almost immediately for a large seed
  // error. Capped at 18deg/s (vs the normal 60deg/s ceiling) - conservative on purpose for wrist's
  // first-ever live test, where the seed-to-target error could be large and completely unverified.
  constexpr float WRIST_Y_MAX_RECOVERY_SPEED = 18.0f; // deg/s
  // Gravity-space, like the target - seedY is PWM-space and offset from it.
  float seedError = fabsf(WRIST_Y_HOME_POSITION - lastSeedSample);
  uint16_t recoveryTimeMs = (uint16_t)std::clamp(
      (seedError / WRIST_Y_MAX_RECOVERY_SPEED) * 1000.0f,
      (float)WRIST_Y_RECOVERY_TIME_MS, 10000.0f);
  LogQueue::Log("[DIAG] wrist calibrateYLoop: seedError=%.2f recoveryTimeMs=%u\n", seedError, recoveryTimeMs);
  wristY.setTargetAngle(WRIST_Y_HOME_POSITION, recoveryTimeMs, WRIST_DEAD_ZONE);
  TickType_t stableSince = 0;
  uint32_t iterations = 0;
  while (true) {
    updateStatusLed(true);
    if (!elbow.isPositionOK()) {
      LogQueue::Log("Wrist calibrating Y aborted: elbow lost position\n");
      setYCalibrating(false);
      return false;
    }
    // elbow.isPositionOK() doesn't reliably/promptly reflect engines-off (it only tracks elbow's
    // own calibrated bit, which lags a CAN round-trip behind platform's engines status) - without
    // this, tick() keeps advancing physicalAngle with no servo power, then snaps to that stale
    // value once power returns.
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] wrist calibrateYLoop: aborted, engines powered off\n");
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
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] wrist calibrateZLoop: aborted, engines powered off\n");
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
  } else {
    wristY.setIMUAngle(physicalAngles.y);
  }
  // Z always on the servo's own position, never yawZ - same unobservable-yaw windup as
  // ArmShoulder::trackTick()'s Z (seen here too, 2026-10-31: command 153 vs yaw-based 136).
  wristZ.setIMUAngle(wristZ.getPhysicalAngle());
  wristY.tick();
  wristZ.tick();
  // Trace a disturbance from the elbow - the wrist's own target doesn't change for one.
  if (fabsf(wristY.getTargetAngle() - wristY.getIMUAngle()) > 1.5f) {
    wristY.requestTrace();
  }
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
      if (!elbow.isPositionOK()) {
        LogQueue::Log("[DIAG] wrist calibrateLoop: aborted, elbow lost position during settle wait\n");
        ok = false;
        break;
      }
      if (!platform.getEnginesPowerStatus()) {
        LogQueue::Log("[DIAG] wrist calibrateLoop: aborted, engines powered off during settle wait\n");
        ok = false;
        break;
      }
      Vector3 physicalAngles = trackTick();
      iterations++;
      bool withinTolerance =
          fabsf(wristYHomeAngle - physicalAngles.y) <= CALIBRATION_SETTLE_TOLERANCE &&
          fabsf(WRIST_Z_HOME_POSITION - wristZ.getPhysicalAngle()) <= CALIBRATION_SETTLE_TOLERANCE; // matches trackTick()'s Z
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
    updateStatusLed(false, true);
    auto sp = imu.isPositionOK();
    auto pp = elbow.isPositionOK();
    auto bp = base.load().isValid();
    auto ep = platform.getEnginesPowerStatus();
    if (!sp || !pp || !bp || !ep) {
      LogQueue::Log("[DIAG] wrist engineLoop: exit (sp=%d pp=%d bp=%d ep=%d) physicalY=%.2f physicalZ=%.2f\n", sp, pp, bp, ep, wristY.getPhysicalAngle(), wristZ.getPhysicalAngle());
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

// TEMP (2026-08-24): sequential-calibration safety testing after a cascading-chaos incident (see
// project memory item 63) - flip to true only for the one module currently being verified in
// isolation, keep every other module false so it stays permanently idle (never calibrates/moves)
// regardless of its own IMU/parent status. Remove this whole mechanism once all four modules have
// been confirmed stable, one at a time, and can safely calibrate together again.
constexpr bool TEMP_SEQUENTIAL_CALIBRATION_ENABLED = true;

void ArmWrist::engineTask(void *instance) {
  auto *wrist = static_cast<ArmWrist *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool wasOK = false;

  // Window to attach a serial log reader before calibration's first seed happens.
  vTaskDelay(pdMS_TO_TICKS(3000));

  while (true) {
    if (!TEMP_SEQUENTIAL_CALIBRATION_ENABLED) {
      // TEMP DIAG (2026-08-24): unambiguous boot identity, repeated (not one-shot) so it doesn't
      // race a log reader attaching - needed after a flashing mixup risk (bootloader mode doesn't
      // expose per-board serial, so multiple boards in BOOTSEL at once could get crossed).
      static Periodic identityPrinter(pdMS_TO_TICKS(2000));
      identityPrinter.interval([&]() {
        LogQueue::Log("=== ArmWrist alive (calibration disabled) ===\n");
      });
      // TEMP DIAG (2026-08-31): passively observe angleFromGravityY() while calibration is disabled,
      // to measure the PWM offset (matching how ArmElbow's was measured) without needing to enable
      // engines/calibration first. Matches ArmShoulder's identical diagnostic call in this branch.
      wrist->angleFromGravityY();
      wrist->updateStatusLed(false);
      wrist->setEngineTaskStatus(false);
      wrist->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

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
    } else {
      // Not straight back into a calibration that fails at its first check - see ArmElbow::engineTask().
      vTaskDelay(pdMS_TO_TICKS(250));
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
  // Hold the wrist's own angle by its own IMU while the elbow moves (2026-11-03): with the defaults a
  // 15deg elbow move tilted it 17.7deg (anchor held for 1 s) and it took 4-6 s to come back. 3/s
  // (loop near 0.5 Hz) answered the elbow's ~1.1 Hz ringing hard enough to feed it: the elbow rang
  // +-3.5deg for 4-5 s after a move, the wrist +-6deg in antiphase, vs +-1deg with 0.5/s. 1.5/s
  // halves that; 40deg/s still covers the elbow's planned-move peak of ~28deg/s.
  wristY.setFineReleaseError(4.0f);
  wristY.setHoldResponse(1.5f, 40.0f);
  wristY.enableTrace();
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

  // The rotation about the joint's own axis since calibration, straight from the quaternion - see
  // ArmShoulder::getIMUAngles() for why not the gravity projection any more (undefined with the
  // joint axis near vertical, 2026-11-03) and why not per-tick deltas. Not qm's gravity either: that
  // is the calibration pose's body z-axis, horizontal here (this axis reads ~90 at vertical), and a
  // 15-20deg shoulder-z turn read through it as 4-6deg of fake pitch (2026-10-31). A turn about the
  // vertical gives no twist about a horizontal joint axis.
  // The joint axis isn't body y in this IMU's corrected frame (q_corr only got gravity right):
  // measured 2026-11-05 by moving the wrist alone 20deg and back with the platform still - a 25deg
  // turn about (-0.555, 0.778, -0.292), steady within 0.01 over the whole move. Twist about y read
  // 0.78 of it, so a 20deg command turned the wrist 25deg.
  constexpr Vector3 WRIST_Y_JOINT_AXIS = {-0.555f, 0.778f, -0.292f};
  float pitchY;
  float pitchYAbs = qm.twistAngle(WRIST_Y_JOINT_AXIS);
  if (!qm.isValid() || isnan(pitchYAbs)) {
    // Glitched reading - hold the last value instead of feeding NaN/garbage forward.
    pitchY = accumulatedPitchY;
  } else if (!pitchIntegrationValid) {
    pitchY = pitchYAbs;
    accumulatedPitchY = pitchYAbs;
    pitchIntegrationValid = true;
  } else {
    // Sanity clamp, not a control filter: real per-tick motion is bounded by servo speed, so an
    // implausibly large single-sample jump is treated as a glitch and capped rather than trusted
    // outright - matching ArmShoulder::getIMUAngles()'s guard.
    constexpr float maxPlausibleDeltaRad = 10.0f * DEG_TO_RAD;
    float delta = std::clamp(pitchYAbs - accumulatedPitchY, -maxPlausibleDeltaRad, maxPlausibleDeltaRad);
    accumulatedPitchY += delta;
    pitchY = accumulatedPitchY;
  }

  static Periodic axisPrinter(pdMS_TO_TICKS(300));
  axisPrinter.interval([&]() {
    LogQueue::Log("[DIAG] wrist pitchY=%.2f yawZ=%.2f\n", pitchY * RAD_TO_DEG, yawZ * RAD_TO_DEG);
    // The absolute quaternion, to check the twist against the gravity projection offline (2026-11-03).
    Quaternion raw = imu.quaternion.load();
    LogQueue::Log("[DIAG] wrist rawQuat i=%.4f j=%.4f k=%.4f real=%.4f\n", raw.i, raw.j, raw.k, raw.real);
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
// Glitch filter added 2026-08-31, matching ArmShoulder::angleFromGravityY()'s identical fix - a
// tick-to-tick jump filter that's time-aware (called from both a tight tracking loop and a slower
// idle-diagnostic loop) rather than a flat per-call threshold. No absolute MIN/MAX range check yet
// (unlike shoulder) - that requires hand-verified reference points across wrist's real range, which
// haven't been measured (wrist has never been live-tested). Add one once that measurement is done.
constexpr float WRIST_GRAVITY_Y_MAX_DEG_PER_SEC = 120.0f;
constexpr float WRIST_GRAVITY_Y_MIN_JUMP_FLOOR = 5.0f; // deg, for back-to-back calls
// res is a difference of two atan2 angles, so it can land on the wrong side of the +/-180 wrap even
// when both inputs are fine - confirmed live (2026-10-31): the wrist, folded ~183deg relative to the
// elbow, read -176.91, which clamped to a PWM 0 seed ~230deg from where it physically was. Split the
// circle inside the servo's unreachable arc instead: res = PWM + (90 - WRIST_Y_PWM_AT_VERTICAL), and
// PWM 0..270 leaves 270..360 unreachable, centered on PWM 315 (= -45).
constexpr float WRIST_GRAVITY_Y_WRAP_LOW = -45.0f + (WRIST_Y_HOME_POSITION - WRIST_Y_PWM_AT_VERTICAL);

float ArmWrist::angleFromGravityY() {
  Quaternion q = imu.quaternion.load();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  float raw = atan2(-g.x, g.z) * RAD_TO_DEG;
  float elbowCorr = elbowYAngleDeg();
  float res = raw - elbowCorr;
  if (res < WRIST_GRAVITY_Y_WRAP_LOW) {
    res += 360.0f;
  } else if (res >= WRIST_GRAVITY_Y_WRAP_LOW + 360.0f) {
    res -= 360.0f;
  }

  static Periodic gravityPrinter(pdMS_TO_TICKS(300));
  gravityPrinter.interval([&]() {
    LogQueue::Log("[DIAG] wrist angleFromGravityY: raw=%.2f elbowCorr=%.2f res=%.2f physical=%.2f\n",
                  raw, elbowCorr, res, wristY.getPhysicalAngle());
  });

  absolute_time_t now = get_absolute_time();
  if (!isnan(lastGoodGravityY)) {
    float elapsedSec = absolute_time_diff_us(lastGoodGravityYTime, now) / 1000000.0f;
    float maxPlausibleJump = std::max(WRIST_GRAVITY_Y_MAX_DEG_PER_SEC * elapsedSec, WRIST_GRAVITY_Y_MIN_JUMP_FLOOR);
    if (fabsf(res - lastGoodGravityY) > maxPlausibleJump) {
      return NAN;
    }
  }

  lastGoodGravityY = res;
  lastGoodGravityYTime = now;
  return res;
}
