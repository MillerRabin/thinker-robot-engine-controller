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

void ArmElbow::onIMUReset() {
  LogQueue::Log("elbow onIMUReset: invalidating base\n");
  base = Quaternion();
  pitchIntegrationValid = false;
}

void ArmElbow::updateStatusLed(bool calibrating, bool ready) {
  bool enginesEnabled = platform.getEnginesPowerStatus();
  if (lastEnginesEnabled && !enginesEnabled) {
    onIMUReset();
    // The one place the servo goes limp: everywhere else reset() keeps holding (see Servo::reset()).
    elbowY.release();
  }
  lastEnginesEnabled = enginesEnabled;

  constexpr TickType_t guardTripHoldTicks = pdMS_TO_TICKS(1500);
  if (elbowY.isDiverging()) {
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

  // Require stable readings before seeding - a transient first sample caused an uncontrolled jerk.
  constexpr int seedStableSamples = 5;
  constexpr float seedStableTolerance = 5.0f; // deg
  float lastSeedSample = NAN;
  int seedStableCount = 0;
  while (true) {
    updateStatusLed(true);
    if (!shoulder.isPositionOK()) {
      LogQueue::Log("Calibrating Y aborted: shoulder lost position (seeding)\n");
      setYCalibrating(false);
      return false;
    }
    // Matching ArmShoulder::calibrateYLoop()'s identical fix: this loop used to only check
    // shoulder's status, not engines power directly - a drop here went undetected until the settle
    // loop below, by which point setDegreeDirect() (an unramped, instant PWM command) may already
    // have committed to a stale/mid-transition seed.
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("Calibrating Y aborted: engines powered off (seeding)\n");
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

  // Seed from the freshly-measured gravity reading, mapped into PWM-space through elbowYHomeAngle
  // (the PWM the last calibration actually held at gravity's 90). Two earlier versions were each
  // wrong in a different way, both confirmed live as a visible jerk at calibration start:
  // - elbowYHomeAngle alone (2026-08-25): stale whenever the arm moved since the last calibration
  //   (engines-off droop, a fresh boot, manual repositioning).
  // - the gravity reading alone (2026-10-31): angleFromGravityY() is NOT PWM-comparable -
  //   ELBOW_GRAVITY_Y_PWM_OFFSET only makes 90 mean vertical, while the servo holds vertical at
  //   PWM ~138-146 - so every seed commanded the servo ~50deg below where it physically was.
  // Clamped to the servo's actual safe range (8-270), not the full mechanical range (0-270).
  // Confirmed live (2026-08-25 and again 2026-08-26) as a real bug: when the arm's real position put
  // lastSeedSample below 8 (e.g. -35.63), clamping only to 0 let seedY through as 0.0, and
  // setDegreeDirect() silently rejects anything below safeMinDegree (returns an error code that
  // wasn't checked) - physicalAngle was left at its stale prior value instead of the seed, so the
  // very next tick computed error against the wrong baseline and drove a visibly too-fast initial
  // move once the real IMU reading came in far from that stale value.
  // A servo still holding (engines never went off, e.g. it froze while shoulder recalibrated) is
  // seeded where it already is instead.
  bool holding = elbowY.isHolding();
  float seedY = holding ? elbowY.getPhysicalAngle()
                        : std::clamp(elbowYHomeAngle + (lastSeedSample - 90.0f), elbowY.safeMinDegree, elbowY.safeMaxDegree);
  LogQueue::Log("[DIAG] calibrateYLoop: seed physicalY=%.2f (raw gravityY=%.2f, home=%.2f, holding=%d)\n", seedY, lastSeedSample, elbowYHomeAngle, holding);
  elbowY.setDegreeDirect(seedY);

  // Same power check during the post-seed settle delay - matching ArmShoulder::calibrateYLoop()'s
  // identical fix. A drop here left the just-committed seed just as stale as one during seeding.
  for (int i = 0; i < 20; i++) {
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("Calibrating Y aborted: engines powered off (post-seed delay)\n");
      setYCalibrating(false);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Recovery time scaled to the actual seed error, not the fixed 1000ms constant - matching
  // ArmShoulder::calibrateYLoop()'s identical fix. 1000ms was even more aggressive than shoulder's
  // old 4000ms, so a large seed error here demanded near-max speed even sooner. Capped at 18deg/s
  // (vs the normal 60deg/s ceiling) - conservative on purpose for a not-yet-verified start position.
  constexpr float ELBOW_Y_MAX_RECOVERY_SPEED = 18.0f; // deg/s
  constexpr uint16_t ELBOW_Y_BASE_RECOVERY_TIME_MS = 1000;
  // Gravity-space, like the target - seedY is PWM-space and offset from it.
  float seedError = fabsf(90.0f - lastSeedSample);
  uint16_t recoveryTimeMs = (uint16_t)std::clamp(
      (seedError / ELBOW_Y_MAX_RECOVERY_SPEED) * 1000.0f,
      (float)ELBOW_Y_BASE_RECOVERY_TIME_MS, 10000.0f);
  LogQueue::Log("[DIAG] calibrateYLoop: seedError=%.2f recoveryTimeMs=%u\n", seedError, recoveryTimeMs);
  elbowY.setTargetAngle(90.0f, recoveryTimeMs, ELBOW_DEAD_ZONE);
  TickType_t stableSince = 0;
  bool ok = true;
  while (true) {
    updateStatusLed(true);
    if (!shoulder.isPositionOK()) {
      LogQueue::Log("Calibrating Y aborted: shoulder lost position\n");
      ok = false;
      break;
    }
    // A frozen servo can never satisfy settled(), so this loop needs its own way out.
    if (elbowY.isDiverging()) {
      LogQueue::Log("Calibrating Y aborted: elbowY diverging\n");
      ok = false;
      break;
    }
    // shoulder.isPositionOK() doesn't reliably/promptly reflect engines-off (it only tracks
    // shoulder's own calibrated bit, which lags a CAN round-trip behind platform's engines status) -
    // without this, tick() keeps advancing physicalAngle with no servo power, then snaps to that
    // stale value once power returns.
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("Calibrating Y aborted: engines powered off\n");
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


// TEMP (2026-08-24): sequential-calibration safety testing after a cascading-chaos incident (see
// project memory item 63) - flip to true only for the one module currently being verified in
// isolation, keep every other module false so it stays permanently idle (never calibrates/moves)
// regardless of its own IMU/parent status. Remove this whole mechanism once all four modules have
// been confirmed stable, one at a time, and can safely calibrate together again.
constexpr bool TEMP_SEQUENTIAL_CALIBRATION_ENABLED = true;

void ArmElbow::engineTask(void *instance) {
  auto *elbow = static_cast<ArmElbow *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    if (!TEMP_SEQUENTIAL_CALIBRATION_ENABLED) {
      // TEMP DIAG (2026-08-24): unambiguous boot identity, repeated (not one-shot) so it doesn't
      // race a log reader attaching - needed after a flashing mixup risk (bootloader mode doesn't
      // expose per-board serial, so multiple boards in BOOTSEL at once could get crossed).
      static Periodic identityPrinter(pdMS_TO_TICKS(2000));
      identityPrinter.interval([&]() {
        LogQueue::Log("=== ArmElbow alive (calibration disabled) ===\n");
      });
      elbow->updateStatusLed(false);
      elbow->setEngineTaskStatus(false);
      elbow->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    auto sp = elbow->imu.isPositionOK();
    auto pp = elbow->shoulder.isPositionOK();
    if (!sp || !pp ) {
      elbow->angleFromGravityY(); // DIAG: measure PWM/gravity offset by hand-positioning at true vertical, remove after
      elbow->updateStatusLed(false);
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
    // Only proceed on a successful calibration - engineLoop() would otherwise retarget to stale values.
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
    updateStatusLed(false, true);
    auto sp = imu.isPositionOK();
    auto pp = shoulder.isPositionOK();
    auto ep = platform.getEnginesPowerStatus();
    if (!sp || !pp || !ep || elbowY.isDiverging()) {
      LogQueue::Log("Engine is turned off (sp=%d pp=%d ep=%d diverging=%d)\n", sp, pp, ep, elbowY.isDiverging());
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
                   // Safe range's floor is 8deg, not 0 - physical low limit is ~5deg (motor ran hot
                   // sitting at commanded 0, straining against the real mechanical stop), 8deg keeps
                   // clear margin. Range(0,270) itself must stay the true calibrated span (it sets
                   // the degree-to-PWM scale); the safe range clamps commands without touching that.
                   const uint canTxPin) : ArmPart(canRxPin, canTxPin),
                                          elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100, 0.0005f, 0.0025f, Range(8, 270)),
                                          imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {
  // Mounting correction: maps raw IMU axes onto ArmShoulder's Y/Z convention.
  Quaternion q_corr = {-0.314134f, -0.584772f, -0.348743f, 0.661619f};
  // Tried (2026-08-24) composing a small residual correction directly into q_corr instead of as a
  // formula-level offset, on the theory that the residual is a real mounting-alignment error and
  // belongs here. Reverted: live-tested, and composing an extra rotation on top of q_corr's own
  // already-large rotation did not behave as a simple, predictable local-axis adjustment (measured
  // result went to roughly -94deg instead of the intended ~0deg) - q_corr's own rotation is large
  // enough that "local Y after q_corr" isn't intuitive to reason about by hand. Not safe to keep
  // guessing signs/axes on live hardware for a mounting-correction quaternion used everywhere -
  // back to the plain formula-level offset (see ELBOW_GRAVITY_Y_PWM_OFFSET below), which is simple,
  // already verified working, and only affects this one function.
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
  if (!base.isValid()) {
    return ERROR_PART_IS_NOT_CALIBRATED;
  }
  Quaternion quat = base * position->quaternion.load();
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

// Uses the gravity vector rather than a yaw-strip-then-pitch-extract decomposition of sq -
// that decomposition is order-sensitive and gives wildly wrong results under a large yaw
// change (confirmed live: a 30deg shoulder-z move made this read a stuck ~-121deg instead of
// a small correction, corrupting elbow's target and causing a real, escalating oscillation).
// getGravityVector() is yaw-invariant, so this can't happen.
float ArmElbow::shoulderYAngleDeg() {
  Quaternion sq = shoulder.imu.quaternion.load();
  if (!sq.isValid()) {
    return 0.0f;
  }
  Vector3 g = sq.getGravityVector();
  return atan2(-g.x, g.z) * RAD_TO_DEG;
}

// Re-measured live for the quaternion-based `raw` (2026-08-14): a full hand-swing to both physical
// hard stops (engines off) observed raw (pre-offset) spanning -128.15 to +135.40 - a ~263deg span
// that fits entirely within atan2's natural +/-180 output without ever needing to wrap. Threshold/
// MIN/MAX below are shifted by ELBOW_GRAVITY_Y_PWM_OFFSET from that measurement so they still
// correctly bracket the (now offset) real range - confirmed live this produced a ~357deg jump in
// `res` for a ~3deg real movement crossing the wrap boundary before it was pushed safely below the
// true minimum; keep it there if the offset ever changes.
// PWM_OFFSET: calibrateYLoop() targets a hardcoded 90.0f in this function's output space, intended
// to mean "true vertical" (matching ArmShoulder's convention) - measured live (engines off, elbow
// hand-positioned at true vertical) that raw read ~7.3deg there, not 90, so calibration was
// converging to a real but wrong pose (confirmed live: LED green/stable, but physically ~180deg
// off). Offset baked in here (not in q_corr - see the constructor comment on why that was tried
// and reverted) to keep "90 = vertical" consistent with every other module's convention.
// Despite the name, this does NOT make the output PWM-comparable (the servo holds vertical at PWM
// ~138-146, not 90) - calibrateYLoop()'s seed maps it through elbowYHomeAngle for that.
constexpr float ELBOW_GRAVITY_Y_PWM_OFFSET = 82.65f;
constexpr float ELBOW_GRAVITY_UNWRAP_THRESHOLD = -150.0f + ELBOW_GRAVITY_Y_PWM_OFFSET;
constexpr float ELBOW_GRAVITY_Y_MIN = -132.0f + ELBOW_GRAVITY_Y_PWM_OFFSET;
constexpr float ELBOW_GRAVITY_Y_MAX = 139.0f + ELBOW_GRAVITY_Y_PWM_OFFSET;

// raw is quaternion-based (getGravityVector()), not raw accelerometer - a raw accelerometer can't
// tell gravity apart from real linear acceleration, so any actual vibration/shake during a fast
// calibration move gets picked up as a (wrong) tilt change, same class of issue fixed for
// shoulderYAngleDeg() above and ArmShoulder::angleFromGravityY(). Also: LocalBNO's setRotate()
// mounting correction (q_corr) only applies to the quaternion, never to raw accelerometer readings
// (see item 20 in project memory) - so the old raw-accelerometer value here was in an uncorrected
// frame entirely, a second, independent reason to prefer this.
float ArmElbow::angleFromGravityY() {
  Quaternion q = imu.quaternion.load();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  float raw = atan2(-g.x, g.z) * RAD_TO_DEG + ELBOW_GRAVITY_Y_PWM_OFFSET;
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

  // pitchY used to be delta-integrated via a per-tick yaw-strip-then-swing decomposition of the
  // quaternion delta (qDeltaSwing.twistAngle({0,1,0})) - order-sensitive, breaks under a large
  // rotation between ticks. A comment here used to claim this already matched
  // ArmShoulder::getIMUAngles()'s fix for the identical bug, but the fix was never actually applied
  // to this function - only the comment was added. Confirmed live (2026-08-25) as a real incident:
  // elbow reached vertical correctly, then suddenly snapped down and hit the table, repeatedly.
  // Fixed the same way as shoulder: use the absolute, yaw-invariant gravity-vector formula directly
  // (this was already computed as the drift-correction term), no delta/integration state needed.
  // Measured from the ABSOLUTE quaternion's gravity, as a delta from its value at calibration - not
  // from qm's, whose "gravity" is really the calibration pose's body z-axis and only stands for
  // gravity if that was vertical. Here it's ~7deg off (raw reads ~7.35 at vertical), enough to turn
  // a rotation about the vertical into fake pitch - confirmed live on the wrist (2026-10-31, body z
  // horizontal there): a 15-20deg shoulder-z turn read as 4-6deg and tracking tilted it for real.
  float pitchY;
  Quaternion qAbs = imu.quaternion.load();
  Vector3 gAbs = qAbs.getGravityVector();
  float gravityPitch = atan2(-gAbs.x, gAbs.z);
  float pitchYAbs = NAN;
  if (qAbs.isValid() && !isnan(gravityPitch)) {
    if (!pitchIntegrationValid) {
      gravityPitchAtBase = gravityPitch;
    }
    float d = gravityPitch - gravityPitchAtBase;
    pitchYAbs = atan2f(sinf(d), cosf(d));
  }
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

  return {0, pitchY, 0};
}
