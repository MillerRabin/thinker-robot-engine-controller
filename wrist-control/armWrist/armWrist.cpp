#include "armWrist.h"
#include "../common/periodic/periodic.h"
#include "../common/quaternion/quaternion.h"

// Joint axes in this IMU's corrected frame (q_corr only got gravity right), each measured by moving
// that joint alone 20deg with the platform still. Y (2026-11-05): a 25deg turn about it, steady
// within 0.01 over the whole move; twist about body y read 0.78 of it, so a 20deg command turned
// the wrist 25deg. Z (2026-11-06): 19.6deg about it for +20 PWM, 89.7deg from Y - the IMU turns
// with Z too. With the arm upright both are horizontal, so gravity calibrates both.
constexpr Vector3 WRIST_Y_JOINT_AXIS = {-0.555f, 0.778f, -0.292f};
constexpr Vector3 WRIST_Z_JOINT_AXIS = {-0.221f, 0.208f, 0.953f};
// Z x Y: the body direction that points up when the wrist stands upright - normal to both axes,
// so it is the "vertical" for both joints.
constexpr Vector3 WRIST_UP_AXIS = {-0.8026f, -0.5938f, -0.0565f};

constexpr Vector3 cross(const Vector3 &a, const Vector3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline float dot(const Vector3 &a, const Vector3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static inline float wrapRad(float a) { return atan2f(sinf(a), cosf(a)); }

// up x Z: the Y axis made exactly normal to the other two (they're 0.3deg off normal as measured),
// so (Z, side, up) is an orthonormal frame - the arm's forward, side and up at calibration.
constexpr Vector3 WRIST_SIDE_AXIS = cross(WRIST_UP_AXIS, WRIST_Z_JOINT_AXIS);

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

// Wrist Z closed on gravity, the same steps as calibrateYLoop(). It used to just go to PWM
// WRIST_Z_HOME_POSITION, assumed - like the shoulder's Z - to turn about the vertical, which gravity
// can't see. It doesn't: with the arm upright its axis is horizontal (2026-11-06), and PWM 135 sat
// ~4deg off upright.
bool ArmWrist::calibrateZLoop() {
  LogQueue::Log("[DIAG] wrist calibrateZLoop: enter\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setZCalibrating(true);
  constexpr int seedStableSamples = 5;
  constexpr float seedStableTolerance = 5.0f; // deg
  float lastSeedSample = NAN;
  int seedStableCount = 0;
  while (seedStableCount < seedStableSamples) {
    updateStatusLed(true);
    if (!elbow.isPositionOK()) {
      LogQueue::Log("Wrist calibrating Z aborted: elbow lost position (seeding)\n");
      setZCalibrating(false);
      return false;
    }
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("Wrist calibrating Z aborted: engines powered off (seeding)\n");
      setZCalibrating(false);
      return false;
    }
    float sample = angleFromGravityZ();
    if (!isnan(sample) && !isnan(lastSeedSample) &&
        fabsf(sample - lastSeedSample) <= seedStableTolerance) {
      seedStableCount++;
    } else {
      seedStableCount = isnan(sample) ? 0 : 1;
    }
    lastSeedSample = sample;
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }

  // Through wristZHomeAngle into PWM-space, or where the servo already holds - see calibrateYLoop().
  bool holding = wristZ.isHolding();
  float seedZ = holding ? wristZ.getPhysicalAngle()
                        : std::clamp(wristZHomeAngle + (lastSeedSample - WRIST_Z_HOME_POSITION), wristZ.safeMinDegree, wristZ.safeMaxDegree);
  LogQueue::Log("[DIAG] wrist calibrateZLoop: seed physicalZ=%.2f (raw gravityZ=%.2f, home=%.2f, holding=%d)\n", seedZ, lastSeedSample, wristZHomeAngle, holding);
  wristZ.setDegreeDirect(seedZ);

  for (int i = 0; i < 20; i++) {
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("Wrist calibrating Z aborted: engines powered off (post-seed delay)\n");
      setZCalibrating(false);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  constexpr float WRIST_Z_MAX_RECOVERY_SPEED = 18.0f; // deg/s
  float seedError = fabsf(WRIST_Z_HOME_POSITION - lastSeedSample);
  uint16_t recoveryTimeMs = (uint16_t)std::clamp(
      (seedError / WRIST_Z_MAX_RECOVERY_SPEED) * 1000.0f,
      (float)WRIST_Y_RECOVERY_TIME_MS, 10000.0f);
  LogQueue::Log("[DIAG] wrist calibrateZLoop: seedError=%.2f recoveryTimeMs=%u\n", seedError, recoveryTimeMs);
  wristZ.setTargetAngle(WRIST_Z_HOME_POSITION, recoveryTimeMs, WRIST_DEAD_ZONE);
  TickType_t stableSince = 0;
  uint32_t iterations = 0;
  while (true) {
    updateStatusLed(true);
    if (!elbow.isPositionOK()) {
      LogQueue::Log("Wrist calibrating Z aborted: elbow lost position\n");
      setZCalibrating(false);
      return false;
    }
    if (!platform.getEnginesPowerStatus()) {
      LogQueue::Log("[DIAG] wrist calibrateZLoop: aborted, engines powered off\n");
      setZCalibrating(false);
      return false;
    }
    float gravityZ = angleFromGravityZ();
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] wrist calibrateZLoop: iter=%lu gravityZ=%.3f physicalZ=%.3f\n", (unsigned long)iterations, gravityZ, wristZ.getPhysicalAngle());
    });
    wristZ.setIMUAngle(gravityZ);
    wristZ.tick();
    updateStatuses();
    if (settled(fabsf(WRIST_Z_HOME_POSITION - gravityZ) <= CALIBRATION_SETTLE_TOLERANCE, stableSince)) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  wristZHomeAngle = wristZ.getPhysicalAngle();
  setZCalibrating(false);
  LogQueue::Log("[DIAG] wrist calibrateZLoop: exit after %lu iterations, wristZHomeAngle=%.2f\n", (unsigned long)iterations, wristZHomeAngle);
  return true;
}

Vector3 ArmWrist::trackTick() {
  Vector3 imuAngles = getIMUAngles();
  Vector3 physicalAngles = getPhysicalAngles(imuAngles);
  // Z holds its own angle by the IMU too since 2026-11-06 - it keeps the claw upright when the
  // platform rolls sideways, which the Y joints can't. It ran on the servo's own position before,
  // taken for an unobservable yaw like the shoulder's Z (2026-10-31: command 153 vs IMU 136).
  const float rate = pitchRate.update(imuAngles.y * RAD_TO_DEG, imu.quaternion.age());
  if (useIMUMode.load() == USE_IMU_NOT_USE) {
    wristY.setIMUAngle(wristY.getPhysicalAngle());
    wristZ.setIMUAngle(wristZ.getPhysicalAngle());
  } else {
    wristY.setIMUAngle(physicalAngles.y);
    wristZ.setIMUAngle(physicalAngles.z);
    wristY.setLinkRate(rate);
  }
  wristY.tick();
  wristZ.tick();
  dampingMonitor.update(rate, wristY.getDampingOffset());
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
    pitchIntegrationValid = false; // re-anchor pitchY and twistZ to 0 at the new base
    twistZIntegrationValid = false;
    // Retarget to wristYHomeAngle (PWM-space) - feedback is PWM-space from here on, target must match.
    wristY.setTargetAngle(wristYHomeAngle, WRIST_Y_RECOVERY_TIME_MS, WRIST_DEAD_ZONE);
    wristZ.setTargetAngle(wristZHomeAngle, WRIST_Y_RECOVERY_TIME_MS, WRIST_DEAD_ZONE);
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
          fabsf(wristZHomeAngle - physicalAngles.z) <= CALIBRATION_SETTLE_TOLERANCE;
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
  twistZIntegrationValid = false;
}

int ArmWrist::updateStatuses() {
  setUseIMUStatus(useIMUMode.load());
  setHomeAngles(wristYHomeAngle, wristZHomeAngle);
  return ArmPart::updateStatuses();
}

void ArmWrist::engineLoop() {
  LogQueue::Log("[DIAG] wrist engineLoop: enter\n");
  wristY.setTargetAngle(wristYHomeAngle, WRIST_Y_RECOVERY_TIME_MS, WRIST_DEAD_ZONE);
  wristZ.setTargetAngle(wristZHomeAngle, WRIST_Y_RECOVERY_TIME_MS, WRIST_DEAD_ZONE);
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
  // The same hold for Z. Its backlash: a 20deg move there and back left 3-4deg (2026-11-06), more
  // than the default 3deg fine budget lets the command cross.
  wristZ.setFineReleaseError(4.0f);
  wristZ.setHoldResponse(1.5f, 40.0f);
  wristZ.setFineCorrectionBudget(5.0f);
  wristY.setHoldHysteresis(HOLD_WAKE_DEG, HOLD_WAKE_MS, HOLD_WAKE_NOW_DEG);
  // Active damping of Y, like the shoulder's (see ArmShoulder's) - with it and the elbow's the whole
  // arm answers a push (2026-11-06). Its IMU is on I2C at 10 ms, so the rate is coarser.
  wristY.setDamping(0.05f, 3.0f, DAMPING_DEADBAND_DEG_PER_SEC);
  wristZ.setHoldHysteresis(HOLD_WAKE_DEG, HOLD_WAKE_MS, HOLD_WAKE_NOW_DEG);
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
  shoulder.dispatchMessage(frame);

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

// The arm's turn about the vertical since calibration, off the shoulder's broadcast: the shoulder's
// link turns only about the vertical and its own y, so its twist about z is that turn exactly. Held
// at the last value while the shoulder isn't heard.
float ArmWrist::armYaw() {
  Quaternion sq = shoulder.imu.quaternion.load();
  if (shoulder.imu.quaternion.isFresh() && sq.isValid()) {
    lastArmYaw = sq.twistAngle({0.0f, 0.0f, 1.0f});
  }
  return lastArmYaw;
}

Vector3 ArmWrist::getIMUAngles() {
  Quaternion qm = base.load() * imu.quaternion.load();
  float yaw = armYaw();

  // The wrist's rotation since calibration is the arm's turn about the vertical, then a pitch about
  // Y, then a side bend about Z: R = Rz(yaw) Ry(pitch) Rx(bend), with x = the Z axis, y = Y, z = up.
  // Undoing the turn (read off the shoulder) leaves Ry(pitch) Rx(bend), and its matrix gives both
  // angles exactly: R00 = cos pitch, R20 = -sin pitch, R11 = cos bend, R12 = -sin bend.
  // Was the twist of qm about each axis (2026-11-05), exact only with the arm unturned. Turned, each
  // leaked into the other: shoulder-z 37deg with the wrist pitched 54deg read a -19deg side bend
  // that wasn't there, and Z bent the claw sideways to "fix" it. Pointing straight down (pitch 180),
  // the twist about Z was undefined.
  // Still absolute, still since calibration: a platform tilt shows up in both, and that's what the
  // two joints hold the claw against. Not qm's gravity: see ArmShoulder::getIMUAngles().
  float pitchYAbs = NAN;
  float twistZAbs = NAN;
  if (qm.isValid()) {
    Quaternion q = Quaternion::AngleAxis(-yaw, WRIST_UP_AXIS.x, WRIST_UP_AXIS.y, WRIST_UP_AXIS.z) * qm;
    const Vector3 rx = q.rotate(WRIST_Z_JOINT_AXIS);
    const Vector3 ry = q.rotate(WRIST_SIDE_AXIS);
    const Vector3 rz = q.rotate(WRIST_UP_AXIS);
    pitchYAbs = atan2f(-dot(WRIST_UP_AXIS, rx), dot(WRIST_Z_JOINT_AXIS, rx));
    twistZAbs = atan2f(-dot(WRIST_SIDE_AXIS, rz), dot(WRIST_SIDE_AXIS, ry));
  }

  float pitchY;
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
    // outright - matching ArmShoulder::getIMUAngles()'s guard. The step is taken across the +-180
    // seam (atan2 jumps there while the wrist turns on through it).
    constexpr float maxPlausibleDeltaRad = 10.0f * DEG_TO_RAD;
    float delta = std::clamp(wrapRad(pitchYAbs - accumulatedPitchY), -maxPlausibleDeltaRad, maxPlausibleDeltaRad);
    accumulatedPitchY += delta;
    pitchY = accumulatedPitchY;
  }

  // Z the same way, with the same glitch clamp.
  float twistZ;
  if (!qm.isValid() || isnan(twistZAbs)) {
    twistZ = accumulatedTwistZ;
  } else if (!twistZIntegrationValid) {
    twistZ = twistZAbs;
    accumulatedTwistZ = twistZAbs;
    twistZIntegrationValid = true;
  } else {
    constexpr float maxPlausibleDeltaRad = 10.0f * DEG_TO_RAD;
    float delta = std::clamp(wrapRad(twistZAbs - accumulatedTwistZ), -maxPlausibleDeltaRad, maxPlausibleDeltaRad);
    accumulatedTwistZ += delta;
    twistZ = accumulatedTwistZ;
  }

  static Periodic axisPrinter(pdMS_TO_TICKS(300));
  axisPrinter.interval([&]() {
    LogQueue::Log("[DIAG] wrist pitchY=%.2f twistZ=%.2f yaw=%.2f\n", pitchY * RAD_TO_DEG, twistZ * RAD_TO_DEG, yaw * RAD_TO_DEG);
    // The absolute quaternion, to check the twist against the gravity projection offline (2026-11-03).
    Quaternion raw = imu.quaternion.load();
    LogQueue::Log("[DIAG] wrist rawQuat i=%.4f j=%.4f k=%.4f real=%.4f\n", raw.i, raw.j, raw.k, raw.real);
  });

  return {0, pitchY, twistZ};
}

Vector3 ArmWrist::getPhysicalAngles(Vector3 &imuAngles) {
  return { 0, (imuAngles.y * RAD_TO_DEG + wristYHomeAngle), (imuAngles.z * RAD_TO_DEG + wristZHomeAngle) };
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
// Gravity's angle about the real joint axis, 0 along WRIST_UP_AXIS, plus WRIST_Y_HOME_POSITION - the
// same way as angleFromGravityZ(). It was atan2(-g.x, g.z) about body y until 2026-11-06: 94 at
// upright, 1.2-1.4x the real turn, and a Z turn leaked in, so calibration stopped ~4deg short of
// upright.
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
  const Vector3 &u = WRIST_UP_AXIS;
  const Vector3 &y = WRIST_Y_JOINT_AXIS;
  Vector3 side = {y.y * u.z - y.z * u.y, y.z * u.x - y.x * u.z, y.x * u.y - y.y * u.x}; // Y x up
  float alongUp = g.x * u.x + g.y * u.y + g.z * u.z;
  float alongSide = g.x * side.x + g.y * side.y + g.z * side.z;
  float raw = atan2f(-alongSide, alongUp) * RAD_TO_DEG + WRIST_Y_HOME_POSITION;
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

// Gravity's angle about the Z axis, 0 when it lies along WRIST_UP_AXIS, in the same space as
// angleFromGravityY(): WRIST_Z_HOME_POSITION at upright. A body turn of +a about Z turns gravity -a
// in the body frame, hence the sign. No elbow correction: the parents only pitch, about axes
// parallel to Y, and that leaves this angle at 0 near upright.
constexpr float WRIST_GRAVITY_Z_MAX_DEG_PER_SEC = 120.0f;
constexpr float WRIST_GRAVITY_Z_MIN_JUMP_FLOOR = 5.0f; // deg, for back-to-back calls
// Split the circle inside the servo's unreachable arc, as for Y: PWM 0..270 leaves 270..360.
constexpr float WRIST_GRAVITY_Z_WRAP_LOW = -45.0f;

float ArmWrist::angleFromGravityZ() {
  Quaternion q = imu.quaternion.load();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  const Vector3 &u = WRIST_UP_AXIS;
  const Vector3 &z = WRIST_Z_JOINT_AXIS;
  Vector3 side = {z.y * u.z - z.z * u.y, z.z * u.x - z.x * u.z, z.x * u.y - z.y * u.x}; // Z x up
  float alongUp = g.x * u.x + g.y * u.y + g.z * u.z;
  float alongSide = g.x * side.x + g.y * side.y + g.z * side.z;
  float res = atan2f(-alongSide, alongUp) * RAD_TO_DEG + WRIST_Z_HOME_POSITION;
  if (res < WRIST_GRAVITY_Z_WRAP_LOW) {
    res += 360.0f;
  } else if (res >= WRIST_GRAVITY_Z_WRAP_LOW + 360.0f) {
    res -= 360.0f;
  }

  static Periodic gravityPrinter(pdMS_TO_TICKS(300));
  gravityPrinter.interval([&]() {
    LogQueue::Log("[DIAG] wrist angleFromGravityZ: res=%.2f physical=%.2f\n", res, wristZ.getPhysicalAngle());
  });

  absolute_time_t now = get_absolute_time();
  if (!isnan(lastGoodGravityZ)) {
    float elapsedSec = absolute_time_diff_us(lastGoodGravityZTime, now) / 1000000.0f;
    float maxPlausibleJump = std::max(WRIST_GRAVITY_Z_MAX_DEG_PER_SEC * elapsedSec, WRIST_GRAVITY_Z_MIN_JUMP_FLOOR);
    if (fabsf(res - lastGoodGravityZ) > maxPlausibleJump) {
      return NAN;
    }
  }

  lastGoodGravityZ = res;
  lastGoodGravityZTime = now;
  return res;
}
