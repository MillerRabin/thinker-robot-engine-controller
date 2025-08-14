#include "armShoulder.h"

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  while (true)
  {
    Quaternion diff = shoulder->difference(shoulder->imu.quaternion);
    Euler euler = diff.getEuler();
    shoulder->shoulderY.setIMUAngle(SHOULDER_Y_HOME_POSITION - euler.getYawAngle());
    shoulder->shoulderZ.setIMUAngle(SHOULDER_Z_HOME_POSITION - euler.getPitchAngle());
    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();

    shoulder->setYCalibrating(shoulder->shoulderY.isCalibrating());
    shoulder->setZCalibrating(shoulder->shoulderZ.isCalibrating());
    
    if (shoulder->shoulderY.isCalibrating() && shoulder->shoulderZ.isCalibrating()) {      
      shoulder->setHomeQuaternion(shoulder->imu.quaternion, shoulder->platform.imu.quaternion);
      shoulder->saveHomeQuaternionsToEEPROM();
      printf("Shoulder home quaternion set and saved to EEPROM\n");
    }
    //printf("Shoulder Y target: %f, current: %f, IMU: %f\n", shoulder->shoulderY.getTargetAngle(), shoulder->shoulderY.getCurrentAngle(), shoulder->shoulderY.getIMUAngle());
    //printf("Shoulder Z target: %f, current: %f, IMU: %f\n", shoulder->shoulderZ.getTargetAngle(), shoulder->shoulderZ.getCurrentAngle(), shoulder->shoulderZ.getIMUAngle());

    //printf("Shoulder roll: %f, pitch: %f, yaw: %f\n", euler.getRollAngle(), euler.getPitchAngle(), euler.getYawAngle());
    shoulder->setEngineTaskStatus(true);
    
    shoulder->updateStatuses();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT));
  }
}

ArmShoulder::ArmShoulder(
    const uint memsSdaPin,
    const uint memsSclPin,
    const uint memsIntPin,
    const uint memsRstPin,
    const uint engineZPin,
    const uint engineYPin,
    const uint canRxPin,
    const uint canTxPin) : ArmPart(canRxPin, canTxPin),
                           shoulderZ(engineZPin, Range(0, 270), SHOULDER_Z_HOME_POSITION, 100),
                           shoulderY(engineYPin, Range(0, 180), SHOULDER_Y_HOME_POSITION, 100),
                           imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)                           
{  
  if (loadHomeQuaternionsFromEEPROM()) {
    printf("Shoulder home quaternions loaded from EEPROM\n");
  }
  else {
    printf("No valid quaternion data found in EEPROM, using defaults\n");
    offsetQuaternion = getRotationQuaternion();
  }
  if (!xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 1024, this, 5, NULL)) {
    setEngineTaskStatus(false);
  }
  else {
    setEngineTaskStatus(true);
  }
}

int ArmShoulder::updateQuaternion(IMUBase *position)
{  
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmShoulder::updateGyroscope(IMUBase *position)
{
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmShoulder::updateAccelerometer(IMUBase *position)
{
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmShoulder::updateAccuracy(IMUBase *position)
{
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmShoulder::busReceiveCallback(can2040_msg frame)
{
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE)
  {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;
    float angleY = angleYS / 100.0f;
    float angleZ = angleZS / 100.0f;
    uint32_t raw1 = frame.data32[1];
    uint16_t timeMS = raw1 & 0xFFFF;

    if (!isnan(angleY)) {
      shoulderY.setTargetAngle(angleY, timeMS, SHOULDER_DEAD_ZONE);
    }
    if (!isnan(angleZ)) {
      shoulderZ.setTargetAngle(angleZ, timeMS, SHOULDER_DEAD_ZONE);
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

Quaternion ArmShoulder::getRotationQuaternion() {
  float rollOffset = -90.0f * (M_PI / 180.0f);
  float pitchOffset = 90.0f * (M_PI / 180.0f);
  float yawOffset = 0.0f * (M_PI / 180.0f);
  Quaternion errorQuat = Quaternion::FromEuler(rollOffset, pitchOffset, yawOffset);
  Quaternion correctionQuat = Quaternion::Conjugate(errorQuat);
  return Quaternion::Multiply(correctionQuat, {0, 0, 0, 1});
};