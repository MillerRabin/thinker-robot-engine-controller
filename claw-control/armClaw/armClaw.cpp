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
  // The one place servos go limp: everywhere else reset() keeps holding (see Servo::reset()). Not on
  // the edge above - that also fires when platform status just stops arriving.
  if (platform.enginesOffEdge()) {
    clawX.release();
    clawY.release();
    clawGripper.release();
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

// Common abort checks for every calibration phase below - engines power in each one (a cut mid-seed
// left a stale seed on the other modules), and losing the wrist (its recalibration moves the claw's
// base); the old loops only checked the wrist in their final settle wait.
// Which of RemoteWrist::isPositionOK()'s checks failed, and how stale each input was - the claw lost
// the wrist 10x in 46 s on 2026-11-03 while the wrist itself kept running (see Bus "CAN stats" gaps).
void ArmClaw::logWristLoss(const char *where) {
  LogQueue::Log("[DIAG] claw wrist lost (%s): online %d calibrated %d, status %lu ms ago, quaternion %lu ms ago\n",
                where, wrist.isOnline(), wrist.isCalibrated(),
                (unsigned long)(wrist.statusAge() * portTICK_PERIOD_MS),
                (unsigned long)(wrist.quaternionAge() * portTICK_PERIOD_MS));
}

bool ArmClaw::calibrationInterrupted(const char *phase) {
  if (!platform.getEnginesPowerStatus()) {
    LogQueue::Log("[DIAG] claw %s: aborted, engines powered off\n", phase);
    return true;
  }
  if (!wrist.isPositionOK()) {
    LogQueue::Log("[DIAG] claw %s: aborted, wrist lost position\n", phase);
    logWristLoss(phase);
    return true;
  }
  return false;
}

// One IMU-closed calibration pass for X or Y, the same steps every other module's calibrateYLoop()
// takes: a stable reading is seeded into PWM-space through homeAngle (the PWM at which the reading
// last equalled readingAtHome) - reused, never re-read, since a fresh read can return NaN - or from
// where the servo is already holding; recovery time scaled to the error at <= 18deg/s; then the loop
// closes on the reading until it settles at readingAtHome, and homeAngle takes the PWM it landed on.
bool ArmClaw::calibrateAxis(const char *phase, Servo &servo, float (ArmClaw::*reading)(),
                            float readingAtHome, float &homeAngle) {
  LogQueue::Log("[DIAG] claw %s: enter\n", phase);
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  constexpr int seedStableSamples = 5;
  constexpr float seedStableTolerance = 5.0f; // deg
  float lastSeedSample = NAN;
  int seedStableCount = 0;
  while (seedStableCount < seedStableSamples) {
    updateStatusLed(true);
    if (calibrationInterrupted(phase)) {
      return false;
    }
    float sample = (this->*reading)();
    if (!isnan(sample) && !isnan(lastSeedSample) &&
        fabsf(sample - lastSeedSample) <= seedStableTolerance) {
      seedStableCount++;
    } else {
      seedStableCount = isnan(sample) ? 0 : 1;
    }
    lastSeedSample = sample;
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }

  bool holding = servo.isHolding();
  float seed = holding ? servo.getPhysicalAngle()
                       : std::clamp(homeAngle + (lastSeedSample - readingAtHome), servo.safeMinDegree, servo.safeMaxDegree);
  LogQueue::Log("[DIAG] claw %s: seed physical=%.2f (reading=%.2f, home=%.2f, holding=%d)\n",
                phase, seed, lastSeedSample, homeAngle, holding);
  servo.setDegreeDirect(seed);

  for (int i = 0; i < 20; i++) {
    if (calibrationInterrupted(phase)) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  constexpr float CLAW_MAX_RECOVERY_SPEED = 18.0f; // deg/s
  float seedError = fabsf(readingAtHome - lastSeedSample);
  uint16_t recoveryTimeMs = (uint16_t)std::clamp((seedError / CLAW_MAX_RECOVERY_SPEED) * 1000.0f, 1000.0f, 10000.0f);
  servo.setTargetAngle(readingAtHome, recoveryTimeMs, CLAW_CALIBRATION_DEAD_ZONE);
  TickType_t stableSince = 0;
  uint32_t iterations = 0;
  while (true) {
    updateStatusLed(true);
    if (calibrationInterrupted(phase)) {
      return false;
    }
    float r = (this->*reading)();
    iterations++;
    printer.interval([&]() {
      LogQueue::Log("[DIAG] claw %s: iter=%lu reading=%.3f physical=%.3f\n", phase, (unsigned long)iterations, r, servo.getPhysicalAngle());
    });
    servo.setIMUAngle(r);
    servo.tick();
    updateStatuses();
    if (settled(fabsf(readingAtHome - r) <= CALIBRATION_SETTLE_TOLERANCE, stableSince)) {
      break;
    }
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  homeAngle = servo.getPhysicalAngle();
  LogQueue::Log("[DIAG] claw %s: exit after %lu iterations, home=%.2f\n", phase, (unsigned long)iterations, homeAngle);
  return true;
}

// Roll pass - only valid with the claw parked horizontal (see clawXReading()).
bool ArmClaw::calibrateXLoop() {
  setXCalibrating(true);
  bool ok = calibrateAxis("calibrateXLoop", clawX, &ArmClaw::clawXReading, CLAW_X_PWM_AT_LEVEL, clawXHomeAngle);
  setXCalibrating(false);
  return ok;
}

// Target 90 = vertical in clawYReading()'s space.
bool ArmClaw::calibrateYLoop() {
  setYCalibrating(true);
  bool ok = calibrateAxis("calibrateYLoop", clawY, &ArmClaw::clawYReading, 90.0f, clawYHomeAngle);
  setYCalibrating(false);
  return ok;
}

// Open-loop move of clawY on its own servo position - parks the claw horizontal for the X pass,
// where no Y reading is needed.
bool ArmClaw::moveYOpenLoop(float pwm) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  float distance = fabsf(pwm - clawY.getPhysicalAngle());
  uint16_t timeMs = (uint16_t)std::clamp((distance / 18.0f) * 1000.0f, 1000.0f, 10000.0f);
  clawY.setTargetAngle(pwm, timeMs, CLAW_OPEN_LOOP_DEAD_ZONE);
  while (!clawY.isPositioned()) {
    updateStatusLed(true);
    if (calibrationInterrupted("moveYOpenLoop")) {
      return false;
    }
    clawY.setIMUAngle(clawY.getPhysicalAngle());
    clawY.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  vTaskDelay(pdMS_TO_TICKS(1000)); // let it settle before the X pass reads gravity
  return true;
}

Vector3 ArmClaw::trackTick() {
  Vector3 imuAngles = getIMUAngles();
  Vector3 physicalAngles = getPhysicalAngles(imuAngles);
  // X, Y and the gripper all track open-loop on their own servo position - the IMU only calibrates.
  // Its frame turns with clawX, so once the host moves X off home a Y reading built from it is
  // wrong, and closing a loop on a reading another axis can corrupt is what tilted the wrist chasing
  // a fake pitch (2026-10-31). The claw is light and its servos hold position precisely.
  // The gripper isn't calibrated, so its first commanded move (fresh boot or after a release) starts
  // from mid-travel, exactly like manual mode's.
  if (!isnan(clawGripper.getTargetAngle()) && !clawGripper.isHolding()) {
    clawGripper.setDegreeDirect(CLAW_GRIPPER_HOME_POSITION);
  }
  for (Servo *servo : {&clawX, &clawY, &clawGripper}) {
    servo->setIMUAngle(servo->getPhysicalAngle());
    servo->tick();
  }
  updateStatuses();
  return physicalAngles;
}

bool ArmClaw::calibrateLoop() {
  LogQueue::Log("[DIAG] claw calibrateLoop: enter\n");
  setArmCalibrated(false);
  updateStatuses();
  // Y -> X -> Y (the user's plan, 2026-10-31): clawX rolls the jaws about the claw's own axis, which
  // gravity can't see while the claw points up - so Y first, then the claw parked horizontal where
  // the roll is measurable, then Y again, since clawYReading() is only exact with X home.
  bool ok = calibrateYLoop();
  if (ok) ok = moveYOpenLoop(CLAW_Y_PWM_HORIZONTAL);
  if (ok) ok = calibrateXLoop();
  if (ok) ok = calibrateYLoop();

  if (ok) {
    base.store(correctedQuat().invert());
    pitchIntegrationValid = false; // re-anchor pitchX/pitchY to 0 at the new base
    clawX.setTargetAngle(clawXHomeAngle, 1000, CLAW_OPEN_LOOP_DEAD_ZONE);
    clawY.setTargetAngle(clawYHomeAngle, 1000, CLAW_OPEN_LOOP_DEAD_ZONE);
    LogQueue::Log("[DIAG] claw calibrateLoop: done, clawXHomeAngle=%.2f clawYHomeAngle=%.2f\n", clawXHomeAngle, clawYHomeAngle);
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
  setHomeAngles(clawYHomeAngle, clawY.getPhysicalAngle());
  return ArmPart::updateStatuses();
}

void ArmClaw::engineLoop() {
  LogQueue::Log("[DIAG] claw engineLoop: enter\n");
  clawX.setTargetAngle(clawXHomeAngle, 1000, CLAW_OPEN_LOOP_DEAD_ZONE);
  clawY.setTargetAngle(clawYHomeAngle, 1000, CLAW_OPEN_LOOP_DEAD_ZONE);
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  while (true) {
    updateStatusLed(false, true);
    auto sp = imu.isPositionOK();
    auto pp = wrist.isPositionOK();
    auto bp = base.load().isValid();
    auto ep = platform.getEnginesPowerStatus();
    if (!sp || !pp || !bp || !ep) {
      LogQueue::Log("[DIAG] claw engineLoop: exit (sp=%d pp=%d bp=%d ep=%d) physicalX=%.2f physicalY=%.2f\n", sp, pp, bp, ep, clawX.getPhysicalAngle(), clawY.getPhysicalAngle());
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

// TEMP (2026-08-24): sequential-calibration safety testing after a cascading-chaos incident (see
// project memory item 63) - flip to true only for the one module currently being verified in
// isolation, keep every other module false so it stays permanently idle (never calibrates/moves)
// regardless of its own IMU/parent status. Remove this whole mechanism once all four modules have
// been confirmed stable, one at a time, and can safely calibrate together again.
constexpr bool TEMP_SEQUENTIAL_CALIBRATION_ENABLED = true;

void ArmClaw::engineTask(void *instance) {
  auto *claw = static_cast<ArmClaw *>(instance);
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
        LogQueue::Log("=== ArmClaw alive (calibration disabled) ===\n");
      });
      claw->updateStatusLed(false);
      claw->setEngineTaskStatus(false);
      claw->updateStatuses();
      // Manual mode: X, Y and the gripper follow explicit host commands (/set "claw-x", "claw-y",
      // "claw-gripper") open-loop, on their own servo position, with claw calibration disabled -
      // how the gripper's travel was found, and how X/Y's PWM-vs-gravity mapping gets measured
      // before their real calibration is trusted. An axis gets no pulse until it's commanded.
      // Runs at the normal tick rate while a move is in flight - tick() assumes ~5ms ticks (dt is
      // capped at 10ms), so the 250ms idle cadence would crawl.
      struct ManualAxis {
        Servo &servo;
        float start;
      };
      // First pulse goes to `start`: X and the gripper to their home (nothing knows where an
      // unpowered open-loop axis is), Y to where its gravity reading says it already is - the claw is
      // light enough to stay at any angle unpowered (user, 2026-10-31), so no fixed start is safe; an
      // earlier fixed 178 jumped ~100deg twice. NAN (no reading) means no pulse yet.
      float yReading = claw->clawYReading();
      float yStart = isnan(yReading) ? NAN
                                     : std::clamp(claw->clawYHomeAngle + (yReading - 90.0f),
                                                  claw->clawY.safeMinDegree, claw->clawY.safeMaxDegree);
      ManualAxis axes[] = {{claw->clawX, CLAW_X_PWM_AT_LEVEL},
                           {claw->clawY, yStart},
                           {claw->clawGripper, CLAW_GRIPPER_HOME_POSITION}};
      auto moving = [](Servo &s) {
        return s.isHolding() && !isnan(s.getTargetAngle()) && !s.isPositioned();
      };
      TickType_t manualWakeTime = xTaskGetTickCount();
      while (claw->platform.getEnginesPowerStatus()) {
        bool anyMoving = false;
        for (auto &axis : axes) {
          if (!isnan(axis.servo.getTargetAngle()) && !axis.servo.isHolding() && !isnan(axis.start)) {
            axis.servo.setDegreeDirect(axis.start);
          }
          if (moving(axis.servo)) {
            anyMoving = true;
            axis.servo.setIMUAngle(axis.servo.getPhysicalAngle());
            axis.servo.tick();
          }
        }
        if (!anyMoving) {
          break;
        }
        vTaskDelayUntil(&manualWakeTime, taskInterval);
      }
      // Both log raw gravity next to the axis's physicalAngle on their own 300ms printers - the
      // data the X/Y mapping is measured from.
      claw->angleFromGravityX();
      claw->angleFromGravityY();
      static Periodic gripperPrinter(pdMS_TO_TICKS(1000));
      gripperPrinter.interval([&]() {
        LogQueue::Log("[DIAG] claw gripper: target=%.2f physical=%.2f\n",
                      claw->clawGripper.getTargetAngle(), claw->clawGripper.getPhysicalAngle());
      });
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    auto sp = claw->imu.isPositionOK();
    auto pp = claw->wrist.isPositionOK();

    if (!sp || !pp) {
      if (wasOK) {
        LogQueue::Log("claw engineTask: position NOT OK (sp=%d pp=%d), stopped\n", sp, pp);
        if (!pp) {
          claw->logWristLoss("engineTask");
        }
        wasOK = false;
      }
      LogQueue::Log("[DIAG] claw engineTask: waiting sp=%d pp=%d\n", sp, pp);
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
    } else {
      // Not straight back into a calibration that fails at its first check - see ArmElbow::engineTask().
      vTaskDelay(pdMS_TO_TICKS(250));
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
      clawY(engineYPin, Range(0, 180), CLAW_Y_HOME_POSITION, 100, 0.0005f, 0.0025f, Range(CLAW_Y_SAFE_MIN, 180)),
      clawGripper(engineGripperPin, Range(0, 180), CLAW_GRIPPER_HOME_POSITION,
                  100, 0.0005f, 0.0025f, Range(CLAW_GRIPPER_SAFE_MIN, CLAW_GRIPPER_SAFE_MAX)),
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
    // X/Y track open-loop (manual mode and engineLoop alike), so the dead zone is stopping precision.
    float axisDeadZone = CLAW_OPEN_LOOP_DEAD_ZONE;

    if (!isnan(angleX)) {
      clawX.setTargetAngle(angleX, timeMS, axisDeadZone);
    }
    if (!isnan(angleY)) {
      clawY.setTargetAngle(angleY, timeMS, axisDeadZone);
    }
    if (!isnan(angleG)) {
      clawGripper.setTargetAngle(angleG, timeMS, CLAW_OPEN_LOOP_DEAD_ZONE);
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

// clawY in "gravity-space" - 90 = vertical, 180 = horizontal on the PWM-178 side, rising with PWM like
// every other joint's reading. Not atan2(-g.x, g.z) like angleFromGravityY(): in this IMU's frame
// gravity lies mostly along body y and clawY turns about ~(-0.074, 0.428, -0.901), so that formula
// roughly doubled the angle (live 2026-10-31: 10deg of claw-y read as 19.6). Measured instead as the
// gravity direction's angle about that axis, fitted from live sweeps with clawX home (1:1 with the
// command there). The axis turns with clawX (the IMU sits on the jaws), so it's only exact with clawX
// home - calibrateLoop() does Y again after X for that reason.
constexpr float CLAW_Y_REF[3] = {0.0322f, 0.9038f, 0.4268f};   // gravity at PWM 178 (horizontal), perpendicular to the axis
constexpr float CLAW_Y_PERP[3] = {0.9967f, 0.0026f, -0.0806f}; // axis x CLAW_Y_REF - gravity at vertical lies here

float ArmClaw::clawYReading() {
  Quaternion q = correctedQuat();
  if (!q.isValid()) {
    return NAN;
  }
  Vector3 g = q.getGravityVector();
  float alongRef = g.x * CLAW_Y_REF[0] + g.y * CLAW_Y_REF[1] + g.z * CLAW_Y_REF[2];
  float alongPerp = g.x * CLAW_Y_PERP[0] + g.y * CLAW_Y_PERP[1] + g.z * CLAW_Y_PERP[2];
  float fromHorizontal = atan2f(alongPerp, alongRef) * RAD_TO_DEG; // 0 horizontal, 90 vertical
  float res = 180.0f - fromHorizontal;

  static Periodic printer(pdMS_TO_TICKS(300));
  printer.interval([&]() {
    LogQueue::Log("[DIAG] claw clawYReading: res=%.2f physical=%.2f\n", res, clawY.getPhysicalAngle());
  });
  return res;
}

// clawX in PWM-comparable form: angleFromGravityX()'s atan2(g.y, g.z) is exactly the roll about
// clawX's axis (body -x, fitted live, 1:1 with the command), but only while the claw lies
// horizontal - pointing up, the roll doesn't move gravity at all. Shifted so the jaws-level roll
// (CLAW_X_ROLL_AT_LEVEL) reads CLAW_X_PWM_AT_LEVEL.
float ArmClaw::clawXReading() {
  float roll = angleFromGravityX();
  if (isnan(roll)) {
    return NAN;
  }
  return roll + (CLAW_X_PWM_AT_LEVEL - CLAW_X_ROLL_AT_LEVEL);
}
