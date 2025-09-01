#include "armElbow.h"

void ArmElbow::engineTask(void *instance)
{
  auto *elbow = static_cast<ArmElbow *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true)
  {   
    elbow->elbowY.setIMUAngle(elbow->elbowY.getPhysicalAngle());    
    elbow->elbowY.tick();

    elbow->setYCalibrating(elbow->elbowY.isCalibrating());
    
    if (elbow->elbowY.isCalibrating())
    {
      elbow->setHomeQuaternion(elbow->imu.quaternion, elbow->platform.imu.quaternion);
      elbow->saveHomeQuaternionsToEEPROM();
      printf("Shoulder home quaternion set and saved to EEPROM\n");
    }

    elbow->setEngineTaskStatus(true);

    elbow->updateStatuses();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT));
  }
}

ArmElbow::ArmElbow(
    const uint memsSdaPin,
    const uint memsSclPin,
    const uint memsIntPin,
    const uint memsRstPin,
    const uint engineYPin,
    const uint canRxPin,
    const uint canTxPin) : ArmPart(canRxPin, canTxPin),
                           elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100),
                           imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
{
  if (!xTaskCreate(ArmElbow::engineTask, "ArmElbow::engineTask", 1024, this, 5, NULL))
  {
    setEngineTaskStatus(false);
  }
  else
  {
    setEngineTaskStatus(true);
  }
}

int ArmElbow::updateQuaternion(IMUBase *position)
{  
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmElbow::updateGyroscope(IMUBase *position)
{
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmElbow::updateAccelerometer(IMUBase *position)
{
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmElbow::updateAccuracy(IMUBase *position)
{
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmElbow::busReceiveCallback(can2040_msg frame)
{
  if (frame.id == CAN_ELBOW_SET_Y_DEGREE) {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 100.0f;

    uint16_t timeMS = (raw >> 16) & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    if (!isnan(angleY)) {
      elbowY.setTargetAngle(angleY, timeMS, ELBOW_DEAD_ZONE);
    }
  }
  if (frame.id == CAN_ELBOW_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}