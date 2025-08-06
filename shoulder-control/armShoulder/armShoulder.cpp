#include "armShoulder.h"

void ArmShoulder::engineTask(void *instance)
{
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true)
  {
    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();

    if (shoulder->shoulderY.atHomePosition() && shoulder->shoulderZ.atHomePosition()) {
      shoulder->setHomeQuaternion(shoulder->imu.quaternion, shoulder->platform.imu.quaternion);
      printf("Shoulder home position set\n");
    }
    
    Quaternion diff = shoulder->difference(shoulder->imu.quaternion);
    Euler euler = diff.getEuler();
    printf("Shoulder roll: %f, pitch: %f, yaw: %f\n", euler.getRollAngle(), euler.getPitchAngle(), euler.getYawAngle());
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
                           shoulderZ(engineZPin, Range(0, 270), Range(-180, 180), IMU_USE_YAW, SHOULDER_Z_HOME_POSITION, 100),
                           shoulderY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, SHOULDER_Y_HOME_POSITION, 100),
                           imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
{
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

    if (!isnan(angleY))
    {
      shoulderY.setTargetAngle(angleY);
    }
    if (!isnan(angleZ))
    {
      shoulderZ.setTargetAngle(angleZ);
    }
  }
  if (frame.id == CAN_SHOULDER_FIRMWARE_UPGRADE)
  {
    rebootInBootMode();
  }
}