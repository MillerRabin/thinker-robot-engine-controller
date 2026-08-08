#include "armElbow.h"
#include "../common/periodic/periodic.h"

void ArmElbow::calibrateLoop() {
  setArmCalibrated(false);
  updateStatuses();
  calibrateYLoop();
  setArmCalibrated(true);
  updateStatuses();
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

void ArmElbow::calibrateYLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  setYCalibrating(true);
  elbowY.reset();
  elbowY.setTargetAngle(90.0f, 1000, ELBOW_DEAD_ZONE);
  TickType_t stableSince = 0;
  while (true) {
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
  elbowYHomeAngle = elbowY.getPhysicalAngle();
  setYCalibrating(false);
}


void ArmElbow::engineTask(void *instance) {
  auto *elbow = static_cast<ArmElbow *>(instance);  
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    auto sp = elbow->imu.isPositionOK();
    auto pp = elbow->shoulder.isPositionOK();        
    if (!sp || !pp ) {
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
    elbow->calibrateLoop();
    elbow->engineLoop();

    elbow->setEngineTaskStatus(false);
    elbow->updateStatuses();    
  }
}

void ArmElbow::engineLoop() {
  LogQueue::Log("Starting Engine Loop\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  base = shoulder.imu.quaternion.load().invert();
  elbowY.setTargetAngle(elbowYHomeAngle, 500, ELBOW_DEAD_ZONE);
  while (true) {
    auto sp = imu.isPositionOK();
    auto pp = shoulder.isPositionOK();
    if (!sp || !pp) {
      LogQueue::Log("Engine is turned off\n");
      elbowY.reset();
      break;
    }

    elbowY.setIMUAngle(elbowY.getPhysicalAngle());
    elbowY.tick();
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

ArmElbow::ArmElbow(const uint memsSdaPin, const uint memsSclPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineYPin, const uint canRxPin,
                   const uint canTxPin) : ArmPart(canRxPin, canTxPin), elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100),
                                          imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {
  // Mounting correction: maps raw IMU axes onto ArmShoulder's Y/Z convention.
  Quaternion q_corr = {-0.314134f, -0.584772f, -0.348743f, 0.661619f};
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
  Quaternion quat = position->quaternion.load();
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

float ArmElbow::shoulderYAngleDeg() {
  Quaternion sq = shoulder.imu.quaternion.load();
  if (!sq.isValid()) {
    return 0.0f;
  }
  float yawZ = sq.twistAngle({0.0f, 0.0f, 1.0f});
  Quaternion qZ = Quaternion::AngleAxis(yawZ, 0.0f, 0.0f, 1.0f);
  Quaternion qSwing = qZ.invert() * sq;
  float pitchY = qSwing.twistAngle({0.0f, 1.0f, 0.0f});
  return pitchY * RAD_TO_DEG;
}

// Monotonic across the full 0-270 PWM range once the atan2 +/-180 wrap is undone (confirmed via full-range sweep).
constexpr float ELBOW_GRAVITY_UNWRAP_THRESHOLD = -90.0f;
constexpr float ELBOW_GRAVITY_Y_MIN = -35.0f;
constexpr float ELBOW_GRAVITY_Y_MAX = 220.0f;

float ArmElbow::angleFromGravityY() {
  Accelerometer acc = imu.accelerometer.load();
  float res = atan2(acc.y, acc.z) * RAD_TO_DEG;
  res -= shoulderYAngleDeg();
  if (res < ELBOW_GRAVITY_UNWRAP_THRESHOLD) {
    res += 360.0f;
  }
  if (res < ELBOW_GRAVITY_Y_MIN || res > ELBOW_GRAVITY_Y_MAX) {
    return NAN;
  }
  return res;
}

Vector3 ArmElbow::getIMUAngles() {
  Quaternion sq = shoulder.imu.quaternion.load();
  Quaternion qm = imu.quaternion.load();
  float pitchX = qm.twistAngle({1.0f, 0.0f, 0.0f});
  /*float pitchY = qm.twistAngle({0.0f, 1.0f, 0.0f});
  float pitchZ = qm.twistAngle({0.0f, 0.0f, 1.0f});*/  
  Quaternion qZ = Quaternion::AngleAxis(pitchX, 0.0f, 0.0f, 1.0f);
  Quaternion qSwing = qZ.invert() * qm;
  float yawX = qSwing.twistAngle({1.0f, 1.0f, 0.0f});
  float yawY = qSwing.twistAngle({0.0f, 1.0f, 0.0f});
  float yawZ = qSwing.twistAngle({0.0f, 0.0f, 1.0f});
  LogQueue::Log("Raw IMU angles: X: %.2f, pitchX: %.2f, pitchY: %.2f, pitchZ: %.2f\n", pitchX * RAD_TO_DEG, yawX * RAD_TO_DEG, yawY * RAD_TO_DEG, yawZ * RAD_TO_DEG);
  return {0, pitchX, yawX};
}
