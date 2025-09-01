#include "armWrist.h"

void ArmWrist::engineTask(void *instance)
{
  auto *wrist = static_cast<ArmWrist *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true)
  {    
    wrist->wristY.setIMUAngle(wrist->wristY.getPhysicalAngle());
    wrist->wristZ.setIMUAngle(wrist->wristZ.getPhysicalAngle());
    wrist->setYCalibrating(wrist ->wristY.isCalibrating());
    wrist->setZCalibrating(wrist->wristZ.isCalibrating());

    wrist->wristY.tick();
    wrist->wristZ.tick();

    if (wrist->wristY.isCalibrating() && wrist->wristZ.isCalibrating()) {
      wrist->setHomeQuaternion(wrist->imu.quaternion, wrist->platform.imu.quaternion);
      wrist->saveHomeQuaternionsToEEPROM();
      printf("Wrist home quaternion set and saved to EEPROM\n");
    }

    wrist->setEngineTaskStatus(true);
    wrist->updateStatuses();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT));
  }
}

ArmWrist::ArmWrist(
    const uint memsSdaPin,
    const uint memsSclPin,
    const uint memsIntPin,
    const uint memsRstPin,
    const uint engineZPin,
    const uint engineYPin,
    const uint canRxPin,
    const uint canTxPin) : ArmPart(canRxPin, canTxPin),
                           wristZ(engineZPin, Range(0, 270), WRIST_Z_HOME_POSITION, 100),
                           wristY(engineYPin, Range(0, 180), WRIST_Y_HOME_POSITION, 100),
                           imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
{
  if (!xTaskCreate(ArmWrist::engineTask, "ArmWrist::engineTask", 1024, this, 5, NULL))
  {
    setEngineTaskStatus(false);
  }
  else
  {
    setEngineTaskStatus(true);
  }
}

int ArmWrist::updateQuaternion(IMUBase *position)
{
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmWrist::updateGyroscope(IMUBase *position)
{
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmWrist::updateAccelerometer(IMUBase *position)
{
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmWrist::updateAccuracy(IMUBase *position)
{
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmWrist::busReceiveCallback(can2040_msg frame)
{
  if (frame.id == CAN_WRIST_SET_YZ_DEGREE)
  {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;    
    
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 100.0f;
    float angleZ = (angleZS == PARAMETER_IS_NAN) ? NAN : angleZS / 100.0f;
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
  if (frame.id == CAN_WRIST_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}