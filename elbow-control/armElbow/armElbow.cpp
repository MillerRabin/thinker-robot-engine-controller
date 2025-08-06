#include "armElbow.h"

void ArmElbow::engineTask(void *instance)
{
  auto *elbow = static_cast<ArmElbow *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true)
  {
    elbow->elbowY.tick();

    /*Euler rEuler = elbow->platform.imu.quaternion.getEuler();
    Euler sEuler = elbow->imu.quaternion.getEuler();
    printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    printf("Shoulder roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle());*/
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
                           elbowY(engineYPin, Range(0, 270), Range(-90, 90), IMU_USE_PITCH, ELBOW_Y_HOME_POSITION, 100),
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
  if (frame.id == CAN_ELBOW_SET_Y_DEGREE)
  {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    float angleY = angleYS / 100.0f;

    if (!isnan(angleY))
    {
      elbowY.setTargetAngle(angleY);
    }
  }
  if (frame.id == CAN_ELBOW_FIRMWARE_UPGRADE)
  {
    rebootInBootMode();
  }
}