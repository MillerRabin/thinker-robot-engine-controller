#include "armWrist.h"

void ArmWrist::engineTask(void *instance)
{
  auto *wrist = static_cast<ArmWrist *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true)
  {
    wrist->wristY.tick();
    wrist->wristZ.tick();    
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
                           wristZ(engineZPin, Range(0, 270), Range(-180, 180), IMU_USE_YAW, WRIST_Z_HOME_POSITION, 100),
                           wristY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, WRIST_Y_HOME_POSITION, 100),
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
    float angleY = angleYS / 100.0f;
    float angleZ = angleZS / 100.0f;

    if (!isnan(angleY))
    {
      wristY.setTargetAngle(angleY);
    }
    if (!isnan(angleZ))
    {
      wristZ.setTargetAngle(angleZ);
    }
  }
  if (frame.id == CAN_WRIST_FIRMWARE_UPGRADE)
  {
    rebootInBootMode();
  }
}