#include "armElbow.h"
#include "../../common/tuning/tuning.h"
#include "../common/periodic/periodic.h"

int ArmElbow::updateStatuses() {
  setUseIMUStatus(useIMUMode.load());
  setHomeAngles(elbowYHomeAngle, NAN);
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
  // With the engines on (a BNO reset mid-run - the engines-off edge calls this too, with them off),
  // rebuild base where the arm is instead of a recalibration's drive to vertical (see recoverInPlace()).
  recoverPending = platform.getEnginesPowerStatus() && !isnan(elbowRawAtCal);
}

// Measured live (2026-11-17): the elbow's BNO under-read its rotation through a move down (20deg short
// in 2.3 s), the fusion check reset it, and the recalibration that followed drove the forearm 45deg
// back up to vertical at the work pose. The link's pitch from gravity tracks the IMU angle one to one
// (IMU angle - gravity pitch held at 48.0 +-0.3deg from vertical to the work pose at rest), so base
// can be rebuilt where the arm stands: the IMU then reads home plus the gravity pitch's change since
// the calibration. The command holds meanwhile. Called each engine tick while base is invalid; true
// while it's still at it, false once done or given up (then the engine loop recalibrates as before).
bool ArmElbow::recoverInPlace() {
  constexpr TickType_t RECOVERY_WINDOW = pdMS_TO_TICKS(300);
  constexpr TickType_t RECOVERY_TIMEOUT = pdMS_TO_TICKS(3000);
  constexpr float RECOVERY_STILL_DEG = 0.5f;
  const TickType_t now = xTaskGetTickCount();
  if (!recovering) {
    recovering = true;
    recoveryStart = now;
    recoveryWindowStart = now;
    recoveryMin = recoveryMax = NAN;
    recoverySum = 0.0f;
    recoveryCount = 0;
    elbowY.reset();  // holds the command; the next tick() takes the damping/dither offsets off
    setIMURecovering(true);
    LogQueue::Log("elbow: IMU reset with the engines on - holding the command, re-anchoring in place\n");
  }
  elbowY.tick();
  updateStatuses();
  if (now - recoveryStart > RECOVERY_TIMEOUT) {
    LogQueue::Log("elbow: in-place re-anchoring gave up (the link didn't keep still) - recalibrating\n");
    recovering = false;
    recoverPending = false;
    setIMURecovering(false);
    return false;
  }
  const float raw = imu.isPositionOK() ? rawGravityYDeg() : NAN;
  if (isnan(raw)) {
    recoveryWindowStart = now;
    recoveryMin = recoveryMax = NAN;
    recoverySum = 0.0f;
    recoveryCount = 0;
    return true;
  }
  recoveryMin = isnan(recoveryMin) ? raw : std::min(recoveryMin, raw);
  recoveryMax = isnan(recoveryMax) ? raw : std::max(recoveryMax, raw);
  recoverySum += raw;
  recoveryCount++;
  if (now - recoveryWindowStart < RECOVERY_WINDOW) {
    return true;
  }
  if (recoveryMax - recoveryMin > RECOVERY_STILL_DEG) {
    recoveryWindowStart = now;
    recoveryMin = recoveryMax = NAN;
    recoverySum = 0.0f;
    recoveryCount = 0;
    return true;
  }
  float delta = recoverySum / recoveryCount - elbowRawAtCal;
  delta -= 360.0f * roundf(delta / 360.0f);
  const Quaternion q = imu.quaternion.load();
  base = (q * Quaternion::AngleAxis(-delta * DEG_TO_RAD, 0.0f, 1.0f, 0.0f)).invert();
  pitchIntegrationValid = false;
  Vector3 imuAngles = getIMUAngles();
  const float angle = getPhysicalAngles(imuAngles).y;
  elbowY.setIMUAngle(angle);
  elbowY.setTargetAngle(angle, 500, ELBOW_DEAD_ZONE);
  LogQueue::Log("elbow: re-anchored in place in %lu ms: gravity pitch %.2f (%.2f at the calibration), IMU %.2f, "
                "command %.2f\n",
                (unsigned long)((now - recoveryStart) * portTICK_PERIOD_MS), recoverySum / recoveryCount,
                elbowRawAtCal, angle, elbowY.getPhysicalAngle());
  recovering = false;
  recoverPending = false;
  setIMURecovering(false);
  return false;
}

void ArmElbow::updateStatusLed(bool calibrating, bool ready) {
  bool enginesEnabled = platform.getEnginesPowerStatus();
  if (lastEnginesEnabled && !enginesEnabled) {
    onIMUReset();
  }
  lastEnginesEnabled = enginesEnabled;
  // The one place the servo goes limp: everywhere else reset() keeps holding (see Servo::reset()).
  // Not on the edge above - that also fires when platform status just stops arriving.
  if (platform.enginesOffEdge()) {
    elbowY.release();
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
    // Calibrated vertical is where gravity stops loading the joint - see Servo::zoneOf().
    elbowY.setGravityNeutral(elbowYHomeAngle);
    base = imu.quaternion.load().invert();
    pitchIntegrationValid = false; // re-anchor pitchY to 0 at the new base
    elbowRawAtCal = rawGravityYDeg();
    recoverPending = false;
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
    } else {
      // A calibration that fails at its first check came straight back here: with the engines off
      // while the shoulder still reported calibrated, hundreds of retries a second, each logging its
      // abort. The flood stalled the log output, the log task starved the BNO reads on the same core,
      // and the IMU went silent 795 ms and was restarted (2026-11-06). Wait before the next try.
      vTaskDelay(pdMS_TO_TICKS(250));
    }

    elbow->setEngineTaskStatus(false);
    elbow->updateStatuses();
  }
}

// The elbow's pitch from the raw accelerometer, in getIMUAngles()'s frame and PWM-space mapping - a
// trace channel only (2026-11-03). In some sessions the game-rotation-vector pitch crawled at
// 4.5-8deg/s while the command moved at 28deg/s, and an elbow reboot made it fast again; this follows
// the real tilt at once (plus the arm's own acceleration), so a trace shows which one is slow.
float ArmElbow::accelPitchDeg() {
  if (!pitchIntegrationValid) {
    return NAN;
  }
  const Accelerometer a = imu.accelerometer.load();
  // The stored quaternion is quat * rotate, so its gravity is rotate^-1 applied to the raw-frame one.
  const Vector3 g = imu.getRotate().invert().rotate({a.x, a.y, a.z});
  if (fabsf(g.x) + fabsf(g.z) < 1e-3f) {
    return NAN;
  }
  const float d = atan2f(-g.x, g.z) - gravityPitchAtBase;
  return atan2f(sinf(d), cosf(d)) * RAD_TO_DEG + elbowYHomeAngle;
}

// Active damping of the elbow, like the shoulder's (see ArmShoulder's): a push at the wrist bends it
// more than the shoulder, and with it damped the whole arm answers a push, not only the shoulder
// (2026-11-06). Half the shoulder's gain - the elbow servo is the weaker one.
// With the link rate filtered over 0.15 s, not 0.06 - see DAMPING_RATE_TAU_SEC (2026-11-17).
constexpr float ELBOW_DAMPING_GAIN_SEC = 0.05f;
constexpr float ELBOW_DAMPING_MAX_DEG = 3.0f;
// Dither while moving (see Servo::setDither()): +-0.5deg - the elbow stuck for ~1deg of command at the
// end of a host move - at 25 Hz, two of the servo's 100 Hz pulses each way. EXPERIMENT 2026-11-07,
// the elbow only.
constexpr float ELBOW_DITHER_DEG = 0.5f;
// Half STREAM_GAIN: the forearm holds its absolute tilt, so it takes the shoulder's tracking error for
// its own, and its correction, late, added to it. Measured live (2026-11-23, the shoulder swinging a
// straight arm -15 <-> +15deg in 5 s, x4 per setting): at 1/s the forearm's error spanned 5.1deg on
// average against 6.9 at 2 (its own share 2.6 against 4.2; the shoulder's gain made no difference);
// the elbow's own 45 <-> 15deg moves were about even, ending up to 1.25deg off where 2 left 0.55 -
// what the settle takes up.
constexpr float ELBOW_STREAM_GAIN = 1.0f;
// The sag feed-forward, see STREAM_SAG_TAU_SEC: the elbow's sag at rest grew 6.9deg per unit of load
// (6.1 under positive loads, 8.5 under negative, +-2-3deg by the side it came from) - a little under it,
// the correction taking the rest the same way. Not what leaves it short coming back from down to park:
// at 6, 3.5 and 2.5 (2026-11-26, x2 interleaved) it ended 2.6-3.5deg short all the same, the
// correction making up the difference mid-move.
constexpr float ELBOW_SAG_PER_LOAD_DEG = 6.0f;
constexpr uint16_t ELBOW_DITHER_HALF_PERIOD_MS = 20;

Vector3 ArmElbow::trackTick() {
  Vector3 imuAngles = getIMUAngles();
  Vector3 physicalAngles = getPhysicalAngles(imuAngles);
  const float rate = pitchRate.update(imuAngles.y * RAD_TO_DEG, imu.quaternion.age());
  if (useIMUMode.load() == USE_IMU_NOT_USE) {
    elbowY.setIMUAngle(elbowY.getPhysicalAngle());
    elbowY.setParentPlannedSpeed(0.0f); // not-use commands are relative - nothing to cancel
  } else {
    elbowY.setIMUAngle(physicalAngles.y);
    elbowY.setLinkRate(rate);
    elbowY.setFusionError(imu.getFusionDriftDeg());
    elbowY.allowDither();
    // The forearm rides on the shoulder: while the shoulder streams, turn against its planned speed.
    elbowY.setParentPlannedSpeed(shoulderPlan.speed());
    if (!isnan(rate) && fabsf(rate) > DAMPING_TRACE_RATE_DEG_PER_SEC && elbowY.isTraceIdle()) {
      elbowY.requestTrace();
    }
  }
  // The pulse's offset from the command, damping and dither - what the servo gets on top (2026-11-07;
  // was accelPitchDeg(), for the 2026-11-02 BNO-lag diagnosis).
  elbowY.setTraceAux(elbowY.getPulseOffset());
  // Settles only after the shoulder has (see Servo::setAncestorsSettling()).
  elbowY.setAncestorsSettling(shoulder.isOnline() && (shoulder.status.load() & ARM_SETTLING) != 0);
  elbowY.setParentSettleSpeed(shoulder.getSettleSpeed());
  elbowY.tick();
  dampingMonitor.update(rate, elbowY.getDampingOffset());
  setStreamWaiting(elbowY.isStreamWaiting());
  setSettling(elbowY.isStreamSettling());
  publishSettleSpeed(elbowY.getStreamSettleSpeed());
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
    // A BNO reset (see LocalBNO::fusionDiverged()) invalidates base: with the engines on it's rebuilt in
    // place (recoverInPlace()), the command held; failing that - and like the shoulder and wrist -
    // recalibrate.
    auto bp = base.isValid();
    if (!bp && recoverPending && pp && ep) {
      if (recoverInPlace()) {
        vTaskDelayUntil(&lastWakeTime, taskInterval);
        continue;
      }
      if (base.isValid()) {
        continue;
      }
    }
    if (!sp || !pp || !bp || !ep) {
      LogQueue::Log("Engine is turned off (sp=%d pp=%d bp=%d ep=%d)\n", sp, pp, bp, ep);
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

ArmElbow::ArmElbow(const uint memsSCKPin, const uint memsMISOPin,
                   const uint memsMOSIPin, const uint memsCSPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineYPin, const uint canRxPin,
                   const uint canTxPin) : 
      ArmPart(canRxPin, canTxPin),
      elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100, 0.0005f,
             0.0025f, Range(8, 270)),
      imu(this, memsSCKPin, memsMISOPin, memsMOSIPin, memsCSPin, memsRstPin,
          memsIntPin) {
  Quaternion q_corr = {-0.314134f, -0.584772f, -0.348743f, 0.661619f};
  imu.setRotate(q_corr);
  // Sway diagnosis: does the command swing with the IMU, and which one lags (2026-11-02).
  elbowY.enableTrace();
  // The elbow servo ran at 8-15deg/s on 2026-11-02 (27deg/s that morning) - see Servo::settleTick().
  elbowY.setPlanSettle(3000);
  elbowY.setHoldHysteresis(HOLD_WAKE_DEG, HOLD_WAKE_MS, HOLD_WAKE_NOW_DEG);
  elbowY.setBreakawayLead(ELBOW_LEAD_LIFT_DEG, ELBOW_LEAD_LOWER_DEG);
  elbowY.setDamping(ELBOW_DAMPING_GAIN_SEC, ELBOW_DAMPING_MAX_DEG, DAMPING_DEADBAND_DEG_PER_SEC);
  elbowY.setDither(ELBOW_DITHER_DEG, ELBOW_DITHER_HALF_PERIOD_MS);
  elbowY.setStreamGain(ELBOW_STREAM_GAIN);
  elbowY.setSagPerLoad(ELBOW_SAG_PER_LOAD_DEG);
  // Its servo's lag (~0.07 s fitted, see STREAM_VELOCITY_LEAD_TAU_SEC), a little under it. Measured
  // live (2026-11-26, fast host moves vertical -> park -> down -> park -> vertical, twice each,
  // interleaved): with the shoulder's sag uncompensated 0.09 did best - the wrist settled 2.3deg against
  // 3.9 at 0.06 - but only by leading the forearm over the shoulder's 3deg lag: the elbow's correction
  // took it up mid-move and still held it at the end, leaving the elbow short the more the higher the
  // lead (down -> park: 1.7-2.3deg at 0.06, 2.6-4.3 at 0.09, 4.7 at 0.12). With the shoulder's sag
  // fed forward (SHOULDER_SAG_PER_LOAD_DEG) 0.06 does better: the elbow's and the wrist's settles over a
  // cycle 15.6deg against 17.7 at 0.09 - all but vertical -> park, where the elbow ran 2.6deg past
  // against 0.9 (the arm leaves vertical only after calibration).
  // Then with the lift lead fading at the load crossing (2026-11-27, the same cycle, 0.06/0.09/0.12 x2,
  // then 0.12 against the sag gain): at 0.06 the joint lagged its plan by a further ~0.04 s x its
  // planned speed in every move (2.3deg at 55deg/s), so the servo's lag is ~0.1 s, not the 0.07 the
  // first fit gave; at 0.12 it tracked within ~0.5deg moving. The correction had been integrating that
  // lag and, frozen at the end, ran the elbow past: vertical -> park 2.2deg past and a 2.1 settle at
  // 0.06, 0.9 past and none at 0.12 (the wrist's settle 3.0 -> 1.3-2.1). Down -> park gets worse, 1.2
  // short and a 1.0 settle against none: there the windup had been cancelling the shoulder catching
  // up at the end (it lags ~2deg braking, the elbow's correction holds the forearm against that, then
  // the shoulder comes up). The elbow's and the wrist's settles over a cycle 8.6-9.6deg against 11.
  elbowY.setVelocityLead(0.12f);
  // The lift lead out as the load turns over (STREAM_LEAD_CROSS_LOAD): measured live (2026-11-27, host
  // moves vertical -> park -> down (6 s) -> park (6 s) -> vertical, the lift lead held at 4, 0 six
  // times, 0.2 and 0.4 twice each): down -> park the elbow ended 0.5-0.6deg short against 2.2-2.9 and
  // neither it nor the wrist settled (2.0-2.8 and 2.8-3.8 at 0); park -> down 0.4 ended 0.0-0.1 short
  // with no settle (0.2: 0.3-0.8). Park -> vertical the elbow ran 1.6-3.2deg past against 0.9-1.3 - the
  // lead goes as the load does there and its correction, frozen at the end, is the bigger for it - but
  // the arm goes vertical only for calibration and power-off. Breaking away was no worse.
  elbowY.setLeadCrossLoad(0.4f);
  // Same as the shoulder's (2026-11-05): with the defaults the elbow trailed vertical by a median 7deg,
  // p90 22deg, on a platform tilting at 10-30deg/s. 40deg/s matches the wrist and stays above the
  // elbow's planned-move peak of ~28deg/s.
  elbowY.setHoldResponse(1.5f, 40.0f);
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
    // Rebuilding base in place: the last one sent, so the wrist keeps its parent (see recoverInPlace()).
    if (recoverPending && lastSentQuat.isValid()) {
      return ArmPart::updateQuaternion(lastSentQuat);
    }
    return ERROR_PART_IS_NOT_CALIBRATED;
  }
  Quaternion quat = base * position->quaternion.load();
  lastSentQuat = quat;
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

  if (frame.id == CAN_TUNE) {
    const TuneRequest r = decodeTune(frame.data);
    if (r.joint == TUNE_JOINT_ELBOW_Y) {
      float value;
      if (r.param == TUNE_RESET_IMU) {
        if (r.set && r.value >= 0.5f) {
          imu.requestReset();
        }
        value = 0.0f;
      } else {
        value = r.param == TUNE_DAMPING_RATE_TAU ? pitchRate.tune(r.set, r.value)
                                                 : elbowY.tune(r.param, r.set, r.value);
      }
      replyTune(r.joint, r.param, r.set, value);
    }
    return;
  }

  if (frame.id == CAN_GRAVITY_LOADS) {
    const int16_t load = (int16_t)((frame.data32[0] >> 16) & 0xFFFF);
    if ((uint16_t)load != PARAMETER_IS_NAN) {
      elbowY.setGravityLoad(load / 1000.0f);
    }
    return;
  }

  // The shoulder's waypoints, for the planned speed the elbow turns against while both stream.
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {
    uint16_t angleYS = frame.data32[0] & 0xFFFF;
    uint16_t timeMS = frame.data32[1] & 0xFFFF;
    shoulderPlan.waypoint(angleYS == PARAMETER_IS_NAN ? NAN : angleYS / 10.0f,
                          timeMS == PARAMETER_IS_NAN ? 0 : timeMS);
  }

  if (frame.id == CAN_ELBOW_SET_Y_DEGREE) {

    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
    uint16_t timeMS = (raw >> 16) & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;
    // The host's end-of-stream shift, 0.01deg signed (see Servo::setStreamEndShift()).
    const uint16_t shiftYS = frame.data32[1] & 0xFFFF;

    LogQueue::Log("CAN_ELBOW_SET_Y_DEGREE received: angleY=%.2f timeMS=%u physicalAngle=%.2f\n", angleY, timeMS, elbowY.getPhysicalAngle());

    if (!isnan(angleY)) {
      elbowY.setStreamEndShift((shiftYS == PARAMETER_IS_NAN ? NAN : (int16_t)shiftYS / 100.0f));
      // A short timeMS is a waypoint of a stream (a straight-line move) - see Servo::streamTo().
      (timeMS > 0 && timeMS <= STREAM_MAX_MS) ? elbowY.streamTo(angleY, timeMS, ELBOW_DEAD_ZONE) : elbowY.planMove(angleY, timeMS, ELBOW_DEAD_ZONE);
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

float ArmElbow::rawGravityYDeg() {
  Quaternion q = imu.quaternion.load();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  return atan2(-g.x, g.z) * RAD_TO_DEG + ELBOW_GRAVITY_Y_PWM_OFFSET;
}

Vector3 ArmElbow::getIMUAngles() {
  Quaternion qm = base * imu.quaternion.load();

  // The rotation about the joint's own axis (body y) since calibration, straight from the
  // quaternion - see ArmShoulder::getIMUAngles() for why not the gravity projection any more
  // (undefined with the joint axis near vertical, 2026-11-03) and why not per-tick deltas (this
  // elbow snapped down and hit the table, 2026-08-25).
  float pitchY;
  float pitchYAbs = qm.twistAngle({0.0f, 1.0f, 0.0f});
  if (!pitchIntegrationValid) {
    // Only for accelPitchDeg()'s trace channel now.
    Vector3 g = imu.quaternion.load().getGravityVector();
    gravityPitchAtBase = atan2(-g.x, g.z);
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
