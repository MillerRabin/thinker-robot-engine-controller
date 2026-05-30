#include "armWrist.h"
#include "../common/periodic/periodic.h"

void ArmWrist::calibrateYLoop() {
  LogQueue::Log("ArmClaw::calibrateYLoop started\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  setYCalibrating(true);
  float guessAngle = WRIST_Y_HOME_POSITION;
  float increment = 90.0f;
  int maxCounter = 40;
  Quaternion position = imu.quaternion.load();
  Quaternion origin = position.invert();
  for (int i = 0; i < maxCounter; i++) {
    wristY.setDegreeDirect(guessAngle);
    LogQueue::Log("\nIteration %d, prevGuessAngle: %.3f, ", i, guessAngle);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(40));
    position = imu.quaternion.load();

    Quaternion delta = origin * position;
    Vector3 g = delta.getGravityVector();

    if (fabs(g.y) < 0.001f) {
      updateStatuses();
      continue;
    }

    origin = position.invert();
    float dir = (g.y > 0) ? -1.0f : 1.0f;
    increment = (increment > 0.1f) ? increment / 2.0f : 0.0f;
    guessAngle += dir * increment;
    LogQueue::Log("currGuessAngle: %.3f, increment: %.3f, y: %.3f", guessAngle, increment, g.y);
    updateStatuses();
  }
  setYCalibrating(false);
}

void ArmWrist::calibrateZLoop() {
  //LogQueue::Log("ArmWrist::calibrateZLoop started\n");
  TickType_t lastWakeTime = xTaskGetTickCount();  
  setZCalibrating(true);
  float guessAngle = WRIST_Z_HOME_POSITION;
  float increment = 90.0f;
  int maxCounter = 40;
  Quaternion position = imu.quaternion.load();
  Quaternion origin = position.invert();
  for (int i = 0; i < maxCounter; i++) {
    wristZ.setDegreeDirect(guessAngle);
    //LogQueue::Log("\nIteration %d, prevGuessAngle: %.3f, ", i, guessAngle);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(40));
    position = imu.quaternion.load();

    Quaternion delta = origin * position;
    auto ea = delta.swingTwistToAngles();

    if (fabs(ea.yaw) < 0.001f) {
      updateStatuses();
      continue;
    }

    origin = position.invert();
    float dir = (ea.yaw > 0) ? -1.0f : 1.0f;
    increment = (increment > 0.1f) ? increment / 2.0f : 0.0f;
    guessAngle += dir * increment;
    //LogQueue::Log("currGuessAngle: %.3f, increment: %.3f, pitch: %.3f", guessAngle, increment, ea.pitch);
    updateStatuses();
  }
  setZCalibrating(false);
}

void ArmWrist::calibrateLoop() {
  setArmCalibrated(false);
  updateStatuses();
  //calibrateZLoop();
  //calibrateYLoop();  
  //setArmCalibrated(true);
  updateStatuses();
}

void ArmWrist::engineLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    /*Quaternion delta = positionOrigin.invert() * position;
    auto euler = delta.swingTwistToAngles();
    //shoulderY.setIMUAngle(physY);
    //shoulderZ.setIMUAngle(physZ);
    shoulderY.tick();
    shoulderZ.tick();
    updateStatuses();*/
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

void ArmWrist::engineTask(void *instance) {
  auto *wrist = static_cast<ArmWrist *>(instance);
  Periodic printer(pdMS_TO_TICKS(1000));
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    auto wp = wrist->imu.isPositionOK();
    auto pp = wrist->platform.isPositionOK();
    auto ec = wrist->claw.isCalibrated();
    
    if (!wp || !pp || !ec) {
      wrist->setEngineTaskStatus(false);
      wrist->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    Quaternion eQuat = wrist->imu.quaternion.load();
    Quaternion pQuat = wrist->platform.imu.quaternion.load();

    if (!eQuat.isValid() || !eQuat.isValid()) {
      wrist->setEngineTaskStatus(false);
      wrist->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    wrist->setEngineTaskStatus(true);
    wrist->updateStatuses();
    wrist->calibrateLoop();
    wrist->engineLoop();

    wrist->setEngineTaskStatus(false);
    wrist->updateStatuses();
    vTaskDelete(NULL);
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
  if (taskHandle == NULL) {
    if (xTaskCreateAffinitySet(ArmWrist::engineTask,
                               "ArmWrist::engineTask", 4096, this, 4,
                               ARM_CORE, &taskHandle) == pdFAIL) {
      printf("Failed to create ArmWrist engine task\n");    
    } else {}
  }
}

int ArmWrist::updateQuaternion(IMUBase *position) {
  Quaternion quat = position->quaternion.load();
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
      printf("Set target angle Y %f\n", angleY);
      wristY.setTargetAngle(angleY, timeMS, WRIST_DEAD_ZONE);
    }
    if (!isnan(angleZ)) {
      printf("Set target angle Z %f\n", angleZ);
      wristZ.setTargetAngle(angleZ, timeMS, WRIST_DEAD_ZONE);
    }
  }
  
  if (frame.id == CAN_WRIST_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}