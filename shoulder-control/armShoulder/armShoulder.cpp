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
  // The one place servos go limp: everywhere else reset() keeps holding (see Servo::reset()). Not on
  // the edge above - that also fires when platform status just stops arriving (see enginesOffEdge()).
  if (platform.enginesOffEdge()) {
    shoulderY.release();
    shoulderZ.release();
  }

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
    // Confirmed live (2026-08-25): a power cut mid-seed used to go undetected here (only the settle
    // loop below checked engines status) - the arm can sag under gravity while this loop keeps
    // measuring, and an inconsistent/stale seed captured across that window fed a real incident.
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] calibrateYLoop: aborted, engines powered off during seeding\n");
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

  // Reuse lastSeedSample (guaranteed non-NaN and stable - that's why the loop above just exited)
  // instead of a fresh angleFromGravityY() call here. Confirmed live (2026-08-31): a fresh call can
  // race into the glitch filter's own rejection (e.g. right after a real, fast recovery move that
  // legitimately jumped far from the last accepted reading) and return NaN - std::clamp() on NaN is
  // unspecified/implementation-defined, so it silently passed the NaN through to setDegreeDirect()
  // instead of being caught, leaving physicalAngle desynced from the real position.
  // A servo still holding (recalibration after a guard trip, engines never went off) is seeded where
  // it already is - re-seeding from gravity jumped by the ~6deg backlash near vertical.
  bool holding = shoulderY.isHolding();
  float imuY = holding ? shoulderY.getPhysicalAngle() : std::clamp(lastSeedSample, 0.0f, 180.0f);
  LogQueue::Log("[DIAG] calibrateYLoop: seed physicalY=%.2f (raw gravityY=%.2f, holding=%d)\n", imuY, lastSeedSample, holding);
  if (!holding && fabsf(imuY - lastSeedSample) > 1.0f) {
    LogQueue::Log("[DIAG] calibrateYLoop: arm is outside the servo's 0-180 range, seed clamped - expect a %.1fdeg jump\n",
                  fabsf(imuY - lastSeedSample));
  }
  shoulderY.setDegreeDirect(imuY);

  // Same power check during the post-seed settle delay - a cut here left the seed just as stale.
  for (int i = 0; i < 20; i++) {
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] calibrateYLoop: aborted, engines powered off during post-seed delay\n");
      setYCalibrating(false);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  LogQueue::Log("[DIAG] calibrateYLoop: post-seed delay done, starting settle loop\n");

  // Recovery time scaled to the actual seed error, not a fixed constant. Confirmed live
  // (2026-08-25): recovering from a large gravity-sagged start with the fixed 4s recovery time
  // makes the speed formula (error/timeLeft) demand near-maxAngularSpeed almost immediately, which
  // looked and felt like an uncontrolled snap. Capped at 18deg/s here (vs the normal 60deg/s
  // ceiling) - deliberately conservative for a start position that hasn't been verified stable yet.
  // setTimeMS() silently ignores values over 10000ms, so this stays clamped under that.
  constexpr float SHOULDER_Y_MAX_RECOVERY_SPEED = 18.0f; // deg/s
  float seedError = fabsf(SHOULDER_Y_HOME_POSITION - lastSeedSample); // gravity-space, like the target
  uint16_t recoveryTimeMs = (uint16_t)std::clamp(
      (seedError / SHOULDER_Y_MAX_RECOVERY_SPEED) * 1000.0f,
      (float)SHOULDER_Y_RECOVERY_TIME_MS, 10000.0f);
  LogQueue::Log("[DIAG] calibrateYLoop: seedError=%.2f recoveryTimeMs=%u\n", seedError, recoveryTimeMs);
  shoulderY.setTargetAngle(SHOULDER_Y_HOME_POSITION, recoveryTimeMs, SHOULDER_DEAD_ZONE);
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
  // Calibrated vertical is where gravity stops loading the joint - see Servo::zoneOf().
  shoulderY.setGravityNeutral(shoulderYHomeAngle);
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
  } else {
    shoulderY.setIMUAngle(physicalAngles.y);
  }
  // Z always runs on the servo's own position, never on yawZ: rotation about the vertical axis
  // doesn't change gravity, and the game rotation vector absorbs a slow one into its gyro-bias
  // estimate - confirmed live (2026-10-31): the arm turned ~20deg about Z while yawZ stayed at
  // 133+/-2.5, so closing the loop on it wound the command 135->177. Calibration's Z pass already
  // ran this way.
  shoulderZ.setIMUAngle(shoulderZ.getPhysicalAngle());
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
      // Z by the servo's own position, matching trackTick() - physicalAngles.z is yaw, unobservable.
      bool withinTolerance =
          fabsf(shoulderYHomeAngle - physicalAngles.y) <= CALIBRATION_SETTLE_TOLERANCE &&
          fabsf(SHOULDER_Z_HOME_POSITION - shoulderZ.getPhysicalAngle()) <= CALIBRATION_SETTLE_TOLERANCE;
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
    // TEMP DIAG (2026-08-24): unambiguous boot identity, repeated (not one-shot) so it doesn't race
    // a log reader attaching - needed after a flashing mixup risk (bootloader mode doesn't expose
    // per-board serial, so multiple boards in BOOTSEL at once could get crossed).
    static Periodic identityPrinter(pdMS_TO_TICKS(2000));
    identityPrinter.interval([&]() {
      LogQueue::Log("=== ArmShoulder alive ===\n");
    });

    auto sp = shoulder->imu.isPositionOK();
    auto pp = shoulder->platform.isPositionOK();

    if (!sp || !pp) {
      if (wasOK) {
        LogQueue::Log("engineTask: position NOT OK (sp=%d pp=%d), stopped\n", sp, pp);
        wasOK = false;
      }
      // TEMP DIAG (2026-08-26): passively observe angleFromGravityY() while idle/engines-off, to
      // check the seed-vs-real-position mismatch hypothesis without needing an engine-enable cycle
      // to trigger a seed measurement. Matches ArmElbow's identical diagnostic call in this branch.
      shoulder->angleFromGravityY();
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
    // PWM-space and gravity-space verified to coincide on this axis: calibration settles with PWM
    // within ~1deg of gravity's 90 (worst seen 5.6), and clean live lifts at ~9deg/s tracked within
    // 1.5deg. 8deg leaves margin for that while bounding how far the command can run ahead.
    shoulderY.setMaxLead(8.0f);
    // ~7deg of backlash at vertical (2026-11-02): with the default 3deg budget the command stopped
    // 1.1deg short of crossing it, and calibrateYLoop never settled (looped >50 s until engines off).
    shoulderY.setFineCorrectionBudget(8.0f);
    // Learn the resting offset only once the joint has stopped - see Servo::setPlanSettle().
    shoulderY.setPlanSettle(3000);
    // Sway diagnosis: does the command swing with the IMU, and which one lags (2026-11-02).
    shoulderY.enableTrace();
}

// Logged every 250 ms whatever the state, calibration included - the shake happened there. Fast
// rotation is what the gyro-fused quaternion does see, unlike a slow yaw drift.
void ArmShoulder::monitorShake(const Quaternion &q) {
  const absolute_time_t now = get_absolute_time();
  if (shakeLastQ.isValid()) {
    const float dt = absolute_time_diff_us(shakeLastTime, now) / 1000000.0f;
    if (dt > 0.0005f && dt < 0.05f) {
      // Body-frame rotation between samples; for a small one, rate = 2 * vector part / dt.
      const Quaternion dq = shakeLastQ.invert() * q;
      const float scale = 2.0f * RAD_TO_DEG / dt * (dq.real < 0.0f ? -1.0f : 1.0f);
      shakePeak.x = fmaxf(shakePeak.x, fabsf(dq.i * scale));
      shakePeak.y = fmaxf(shakePeak.y, fabsf(dq.j * scale));
      shakePeak.z = fmaxf(shakePeak.z, fabsf(dq.k * scale));
    }
  }
  shakeLastQ = q;
  shakeLastTime = now;
  if (absolute_time_diff_us(shakeWindowStart, now) >= 250000) {
    LogQueue::Log("[SHAKE] peak deg/s x %.1f y %.1f z %.1f\n", shakePeak.x, shakePeak.y, shakePeak.z);
    shakePeak = {0.0f, 0.0f, 0.0f};
    shakeWindowStart = now;
  }
}

int ArmShoulder::updateQuaternion(IMUBase *position) {
  monitorShake(position->quaternion.load());
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
      shoulderY.planMove(angleY, timeMS, SHOULDER_DEAD_ZONE);
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

  // pitchX used to be delta-integrated via a per-tick yaw-strip-then-swing decomposition of the
  // quaternion delta (qDeltaSwing.twistAngle({0,1,0})). That decomposition is order-sensitive and
  // breaks under a large rotation between ticks - confirmed live (2026-08-24) as the actual root
  // cause of a real runaway: with Y and Z both moving in engineLoop, a bad per-tick delta got
  // integrated, producing a target that ran away from the true angle, growing the servo error
  // exponentially (5.56 -> 71.79deg) until shoulderY hit its 180deg PWM limit and the diverging
  // guard froze it. Same failure pattern already documented as the cause of ArmElbow::shoulderYAngleDeg()
  // getting stuck after a 30deg shoulder-z move. Fixed by using the same gravity-vector-based
  // absolute formula already trusted for calibration (angleFromGravityY(), calibrateYLoop()) and
  // for this function's own former drift-correction term - getGravityVector() is yaw-invariant, so
  // it can't suffer this order-sensitivity, and it needs no delta/integration state at all.
  // Taken from the ABSOLUTE quaternion's gravity as a delta from calibration, not from qm's - qm's
  // "gravity" is the calibration pose's body z-axis, which only stands for gravity if that was
  // vertical. Close to true here (raw ~-2deg at vertical), but not on the wrist, where a shoulder-z
  // turn read as fake pitch live (2026-10-31) - kept identical across modules.
  float pitchX;
  Quaternion qAbs = imu.quaternion.load();
  Vector3 gAbs = qAbs.getGravityVector();
  float gravityPitch = atan2(-gAbs.x, gAbs.z);
  float pitchXAbs = NAN;
  if (qAbs.isValid() && !isnan(gravityPitch)) {
    if (!pitchIntegrationValid) {
      gravityPitchAtBase = gravityPitch;
    }
    float d = gravityPitch - gravityPitchAtBase;
    pitchXAbs = atan2f(sinf(d), cosf(d));
  }
  if (!qm.isValid() || isnan(pitchXAbs)) {
    // Glitched reading - hold the last value instead of feeding NaN/garbage forward.
    pitchX = accumulatedPitchX;
  } else if (!pitchIntegrationValid) {
    pitchX = pitchXAbs;
    accumulatedPitchX = pitchXAbs;
    pitchIntegrationValid = true;
  } else {
    // Sanity clamp, not a control filter: real per-tick motion is bounded by servo speed, so a
    // implausibly large single-sample jump is treated as a glitch and capped rather than trusted
    // outright - a second line of defense against the same class of bad reading, independent of
    // whatever the growing-error guard catches downstream.
    constexpr float maxPlausibleDeltaRad = 10.0f * DEG_TO_RAD;
    float delta = std::clamp(pitchXAbs - accumulatedPitchX, -maxPlausibleDeltaRad, maxPlausibleDeltaRad);
    accumulatedPitchX += delta;
    pitchX = accumulatedPitchX;
  }

  static Periodic axisPrinter(pdMS_TO_TICKS(300));
  axisPrinter.interval([&]() {
    LogQueue::Log("[DIAG] pitchX=%.2f yawZ=%.2f\n", pitchX * RAD_TO_DEG, yawZ * RAD_TO_DEG);
    // Raw local sensor reading, before any base transform - to distinguish a genuine sensor
    // glitch (raw i/j/k/real jumps) from a computation issue (raw looks smooth, yawZ doesn't).
    Quaternion raw = imu.quaternion.load();
    LogQueue::Log("[DIAG] rawQuat i=%.4f j=%.4f k=%.4f real=%.4f\n", raw.i, raw.j, raw.k, raw.real);
  });

  return {0, pitchX, yawZ};
}

Vector3 ArmShoulder::getPhysicalAngles(Vector3 &imuAngles) {
  return { 0, (imuAngles.y * RAD_TO_DEG + shoulderYHomeAngle), (imuAngles.z * RAD_TO_DEG + SHOULDER_Z_HOME_POSITION) };
}

// Quaternion-based (getGravityVector()), not raw accelerometer - a raw accelerometer can't tell
// gravity apart from real linear acceleration, so it picks up any actual vibration/shake as a
// (wrong) tilt change. Calibration is exactly when the servo moves fastest/hardest (large error,
// max commanded speed), so any real mechanical vibration there would corrupt this reading and
// feed a bogus angle back into the control loop - this was the actual root cause of the repeating
// chaotic-oscillation-during-calibration incidents (confirmed live: zero jumps/chaos with this
// formula under real hand-driven motion, vs. constant chaos with the old raw-accelerometer one).
// PWM_OFFSET: unlike ArmWrist::angleFromGravityY(), this axis's raw atan2 zero-crossing does NOT
// land at PWM=SHOULDER_Y_HOME_POSITION - measured live (engines off, arm hand-positioned at true
// vertical): raw read ~-2deg where PWM-space expects 90deg. Baked in here so the function's return
// value is directly PWM-comparable everywhere else in this file, matching every other module's convention.
constexpr float SHOULDER_GRAVITY_Y_PWM_OFFSET = 92.0f;

// Revised 2026-08-31: hand-verified at three known reference points with engines off (0deg, 90deg
// vertical, 180deg - the true mechanical stops) that this formula tracks close to 1:1 across the
// FULL range (res read 0.90, ~89, 178.51 respectively) - there's no real geometric wrap or
// non-monotonic region in the static relationship. The chaotic negative/jumpy readings seen live
// during fast calibration moves were tick-to-tick glitches (matching the same class of issue
// getIMUAngles() already guards against), not a position-dependent formula breakdown - a prior
// version of this comment misdiagnosed that jump as landing in "wrapped garbage" when the jump
// destination was actually close to the correct value and the readings before it were the glitch.
// MIN/MAX now cover the true valid range with margin, not a guessed "safe middle."
// MIN widened from -15 (2026-10-31): unpowered, the arm's weight back-drives the servo past its
// commandable 0 down to the joint's real mechanical stop at ~-23 (confirmed live by the user) - with
// -15 every sample there was rejected and calibration silently never started. The seed still clamps
// to 0, so starting from there is one deliberate jump up to PWM 0 (logged in calibrateYLoop()).
constexpr float SHOULDER_GRAVITY_Y_MIN = -35.0f;
constexpr float SHOULDER_GRAVITY_Y_MAX = 195.0f;
// Tick-to-tick glitch filter, mirroring getIMUAngles()'s sanity clamp - time-aware rather than a
// flat per-call threshold, since this function is called from both a tight ~5ms tracking loop and
// a slower ~250ms idle-diagnostic loop. Rate is more generous than maxAngularSpeed (60deg/s)
// because free, unpowered motion (e.g. drooping under gravity with engines off) isn't bounded by
// the servo's own commanded speed.
constexpr float SHOULDER_GRAVITY_Y_MAX_DEG_PER_SEC = 120.0f;
constexpr float SHOULDER_GRAVITY_Y_MIN_JUMP_FLOOR = 5.0f; // deg, for back-to-back calls

float ArmShoulder::angleFromGravityY() {
  Quaternion q = imu.quaternion.load();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  float res = atan2(-g.x, g.z) * RAD_TO_DEG + SHOULDER_GRAVITY_Y_PWM_OFFSET;

  static Periodic gravityPrinter(pdMS_TO_TICKS(300));
  gravityPrinter.interval([&]() {
    LogQueue::Log("[DIAG] shoulder angleFromGravityY: res=%.2f physical=%.2f\n", res, shoulderY.getPhysicalAngle());
  });

  if (res < SHOULDER_GRAVITY_Y_MIN || res > SHOULDER_GRAVITY_Y_MAX) {
    return NAN;
  }

  absolute_time_t now = get_absolute_time();
  if (!isnan(lastGoodGravityY)) {
    float elapsedSec = absolute_time_diff_us(lastGoodGravityYTime, now) / 1000000.0f;
    float maxPlausibleJump = std::max(SHOULDER_GRAVITY_Y_MAX_DEG_PER_SEC * elapsedSec, SHOULDER_GRAVITY_Y_MIN_JUMP_FLOOR);
    if (fabsf(res - lastGoodGravityY) > maxPlausibleJump) {
      return NAN;
    }
  }

  lastGoodGravityY = res;
  lastGoodGravityYTime = now;
  return res;
}
