#include "armShoulder.h"
#include "../common/periodic/periodic.h"
#include "../common/speedBuffer/speedBuffer.h"

Quaternion ArmShoulder::rotationQuaternion =
    Quaternion(0.0f, 0.0f, 0.0f, 1.0f); // East North Up

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);

  Periodic printer(pdMS_TO_TICKS(1000));
  TickType_t lastWakeTime = xTaskGetTickCount();

  shoulder->imu.begin();

  while (true) {
    if (shoulder->getTareError() || !shoulder->platform.isPositionOK()) {
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    auto sQuat = shoulder->imu.quaternion;
    auto pQuat = shoulder->platform.imu.quaternion;

    if (!sQuat.isValid() || !pQuat.isValid()) {
      // shoulder->shoulderY.stop();
      // shoulder->shoulderZ.stop();
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    auto sRel = pQuat.invert() * sQuat;
    auto sDiff = shoulder->offset.invert() * sRel;
    auto ea = sDiff.swingTwistToAngles();
    auto pitchDeg = ArmPart::NormalizeAngle(ea.pitch * 180.0f / (float)M_PI);
    shoulder->tick(pQuat, sQuat);

    shoulder->shoulderY.setIMUAngle(pitchDeg);
    shoulder->shoulderZ.setIMUAngle(shoulder->shoulderZ.getPhysicalAngle());

    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();

    shoulder->setEngineTaskStatus(true);
    shoulder->updateStatuses();
    vTaskDelayUntil(&lastWakeTime, shoulder->taskInterval);
  }
}

ArmShoulder::ArmShoulder(uint memsSCKPin, uint memsMISOPin, uint memsMOSIPin,
                         uint memsCSPin, uint memsIntPin, uint memsRstPin,
                         uint engineZPin, uint engineYPin, uint canRxPin,
                         uint canTxPin)
    : ArmPart(canRxPin, canTxPin),
      shoulderZ(engineZPin, Range(0, 180), SHOULDER_Z_HOME_POSITION, 100),
      shoulderY(engineYPin, Range(0, 180), NAN, 100, 0.00055),
      imu(this, memsSCKPin, memsMISOPin, memsMOSIPin, memsCSPin, memsRstPin,
          memsIntPin) {}

int ArmShoulder::updateQuaternion(IMUBase *position) {
  auto resQuat = ArmShoulder::rotationQuaternion * position->quaternion;
  return ArmPart::updateQuaternion(resQuat);
}

int ArmShoulder::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmShoulder::updateAccelerometer(IMUBase *position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
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
      printf("Received shoulder Y angle: %f\n", angleY);
      shoulderY.setTargetAngle(angleY, timeMS, SHOULDER_DEAD_ZONE);
    }
    if (!isnan(angleZ)) {
      shoulderZ.setTargetAngle(angleZ, timeMS, SHOULDER_DEAD_ZONE);
    }
  }
  if (frame.id == CAN_TARE) {
    uint32_t raw = frame.data32[0];
    uint16_t clearMask = raw & 0xFFFF;
    uint16_t tareMask = (raw >> 16) & 0xFFFF;

    if (clearMask & ARM_SHOULDER) {
      this->imu.clearTare();
      this->imu.saveTare();
    }

    if (tareMask & ARM_SHOULDER) {
      this->imu.tare(TARE_AXIS_ALL);
      this->imu.saveTare();
      this->scheduleSave();
    }
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
  if (xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 4096,
                  this, 4, NULL) == pdFAIL) {
    printf("Failed to create ArmShoulder engine task\n");
    return ERROR_ENGINE_TASK_CREATION_FAILED;
  }
  return 0;
}