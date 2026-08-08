#include "armShoulder.h"
#include "../common/periodic/periodic.h"
#include "../common/quaternion/quaternion.h"
#include "../common/speedBuffer/speedBuffer.h"

void ArmShoulder::calibrateYLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setYCalibrating(true);
  float imuY = angleFromGravityY();
  imuY = std::clamp(imuY, 0.0f, 180.0f);
  LogQueue::Log("[DIAG] calibrateYLoop: instant snap physicalY %.2f -> %.2f (accel raw before clamp)\n", shoulderY.getPhysicalAngle(), imuY);
  shoulderY.setDegreeDirect(imuY);
  vTaskDelay(pdMS_TO_TICKS(2000));
  // Close the loop on the accelerometer, not on the servo's own commanded
  // angle: SHOULDER_Y_HOME_POSITION (90) is meant to be vertical, but the
  // servo's PWM "90" doesn't always correspond to true vertical (mechanical
  // offset/tilt). Driving until the accelerometer itself reads 90 lands on
  // the real vertical regardless of that offset.
  shoulderY.setTargetAngle(SHOULDER_Y_HOME_POSITION, 1000, SHOULDER_DEAD_ZONE);
  while (!shoulderY.isPositioned()) {
    float gravityY = angleFromGravityY();
    printer.interval([&]() {
      LogQueue::Log("Calibrating Y... IMU Y: %.3f, Physical Y: %.3f\n", gravityY, shoulderY.getPhysicalAngle());
    });
    shoulderY.setIMUAngle(gravityY);
    shoulderY.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  // Record where this actually landed (accelerometer-verified vertical) so
  // downstream PWM<->base-relative-angle conversions use the real value
  // instead of assuming it landed exactly on SHOULDER_Y_HOME_POSITION.
  shoulderYHomeAngle = shoulderY.getPhysicalAngle();
  setYCalibrating(false);
}

void ArmShoulder::calibrateZLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));

  setZCalibrating(true);
  // Z (yaw) has no accelerometer reference, so unlike Y we can't verify
  // where it actually is - the best available guess is the last physical
  // angle this same power-on session commanded it to (engineLoop's break
  // now preserves this via stop() instead of reset()). Only a genuinely
  // fresh boot (no prior command this session) has no such guess, in which
  // case nominal home is the only option. Either way, drive there with a
  // speed-limited ramp (self-tracking, physicalAngle as its own feedback -
  // same pattern elbow's engineLoop uses) instead of an instant PWM jump:
  // if Z was left far from home when power was cut, an unramped
  // setDegreeDirect() would snap the servo at full mechanical speed.
  float startZ = shoulderZ.getPhysicalAngle();
  if (isnan(startZ)) {
    startZ = SHOULDER_Z_HOME_POSITION;
  }
  LogQueue::Log("[DIAG] calibrateZLoop: ramping physicalZ %.2f -> %.2f (home)\n", startZ, (float)SHOULDER_Z_HOME_POSITION);
  shoulderZ.setDegreeDirect(startZ);
  shoulderZ.setTargetAngle(SHOULDER_Z_HOME_POSITION, 1000, SHOULDER_DEAD_ZONE);
  while (!shoulderZ.isPositioned()) {
    printer.interval([&]() {
      LogQueue::Log("Calibrating Z... Physical Z: %.3f\n", shoulderZ.getPhysicalAngle());
    });
    shoulderZ.setIMUAngle(shoulderZ.getPhysicalAngle());
    shoulderZ.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
  setZCalibrating(false);
}

void ArmShoulder::calibrateLoop() {
  setArmCalibrated(false);
  updateStatuses();
  calibrateYLoop();
  calibrateZLoop();
  vTaskDelay(pdMS_TO_TICKS(2000));
  // Re-derive base on every calibration, not just the first time: BNO085 yaw
  // (Game Rotation Vector, no magnetometer) has no absolute reference and
  // drifts continuously, so a base kept from a much earlier calibration goes
  // stale across repeated engine power-cycles. Y has just been driven to
  // accelerometer-verified vertical and Z has just been snapped to its
  // nominal home PWM - this is the most accurate moment to declare "zero",
  // and refreshing it here eliminates the accumulated drift that was
  // driving Y/Z into a diverging, full-speed runaway.
  base.store(imu.quaternion.load().invert());
  setArmCalibrated(true);
  updateStatuses();
}

void ArmShoulder::onIMUReset() {
  LogQueue::Log("[DIAG] onIMUReset: invalidating base\n");
  base.store(Quaternion());
}

int ArmShoulder::updateStatuses() {
  // Sync the reported useIMU bits from the actual live useIMUMode every
  // time, not just when a CAN_USE_IMU command happens to arrive - otherwise
  // a freshly booted/reflashed board reports "not-use" (the register's
  // zero default) even though useIMUMode's own constructor default is
  // USE_IMU_USE, until the first command ever shows up.
  setUseIMUStatus(useIMUMode.load());
  return ArmPart::updateStatuses();
}

void ArmShoulder::engineLoop() { 
  LogQueue::Log("Starting Engine Loop\n"); 
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));      
  while (true) {
    auto sp = imu.isPositionOK();
    auto pp = platform.isPositionOK();
    auto bp = base.load().isValid();
    if (!sp || !pp || !bp) {
      LogQueue::Log("[DIAG] Engine is turned off (sp=%d pp=%d bp=%d) physicalY=%.2f physicalZ=%.2f\n", sp, pp, bp, shoulderY.getPhysicalAngle(), shoulderZ.getPhysicalAngle());
      shoulderY.reset();
      // Y always re-seeds physicalAngle from the accelerometer at the start
      // of calibrateYLoop() regardless, so wiping it here is harmless. Z has
      // no such external reference - stop() (clears targetAngle only) keeps
      // physicalAngle as calibrateZLoop()'s best-known starting point for a
      // smooth ramp back to home, instead of losing that memory entirely.
      shoulderZ.stop();
      break;
    }

    Vector3 imuAngles = getIMUAngles();
    Vector3 physicalAngles = getPhysicalAngles(imuAngles);    
    printer.interval([&]() {
      LogQueue::Log("Y: %.2f %.2f, Z: %.2f %.2f\n",
                    physicalAngles.y, imuAngles.y * RAD_TO_DEG,
                    physicalAngles.z, imuAngles.z * RAD_TO_DEG);
    });

    uint8_t imuMode = useIMUMode.load();
    if (imuMode == USE_IMU_NOT_USE) {
      shoulderY.setIMUAngle(shoulderY.getPhysicalAngle());
      shoulderZ.setIMUAngle(shoulderZ.getPhysicalAngle());
    } else {
      // USE_IMU_USE and USE_IMU_AUTO currently behave identically.
      shoulderY.setIMUAngle(physicalAngles.y);
      shoulderZ.setIMUAngle(physicalAngles.z);
    }
    shoulderY.tick();
    shoulderZ.tick();

    updateStatuses();    
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool wasOK = false;

  while (true) {
    auto sp = shoulder->imu.isPositionOK();
    auto pp = shoulder->platform.isPositionOK();

    if (!sp || !pp) {
      if (wasOK) {
        LogQueue::Log("[DIAG] engineTask: position NOT OK (sp=%d pp=%d), stopped\n", sp, pp);
        wasOK = false;
      }
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    auto position = shoulder->imu.quaternion.load();
    auto platformPosition = shoulder->platform.imu.quaternion.load();

    if (!position.isValid() || !platformPosition.isValid()) {
      if (wasOK) {
        LogQueue::Log("[DIAG] engineTask: quaternion invalid (imu=%d platform=%d), stopped\n", position.isValid(), platformPosition.isValid());
        wasOK = false;
      }
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    LogQueue::Log("[DIAG] engineTask: position OK -> entering calibrateLoop (physicalY=%.2f physicalZ=%.2f)\n", shoulder->shoulderY.getPhysicalAngle(), shoulder->shoulderZ.getPhysicalAngle());
    wasOK = true;
    shoulder->setEngineTaskStatus(true);
    shoulder->calibrateLoop();
    shoulder->engineLoop();
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
  Quaternion qZ = Quaternion::AngleAxis(yawZ, 0.0f, 0.0f, 1.0f);
  Quaternion qSwing = qZ.invert() * qm;
  float pitchX = qSwing.twistAngle({0.0f, 1.0f, 0.0f});  
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
