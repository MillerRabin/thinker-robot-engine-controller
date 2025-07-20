#include "armShoulder.h"

void ArmShoulder::engineTask(void *instance) {
  auto* shoulder = static_cast<ArmShoulder*>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {    
    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();
    
    Euler rEuler = shoulder->platform.bno.quaternion.getEuler();
    Euler sEuler = shoulder->position.quaternion.getEuler();
    printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    printf("Shoulder roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle());
    printf("Dropped frames %d\n", Bus::droppedFrames);
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
                           shoulderZ(engineZPin, Range(0, 270), Range(-180, 180), IMU_USE_YAW, 100),
                           shoulderY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
                           position(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
{
  if (!xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 1024, this, 5, NULL)) {
    setEngineTaskStatus(false);
  } else {
    setEngineTaskStatus(true);
  }
}

int ArmShoulder::updateQuaternion(BasePosition *position)
{
  Euler euler = position->quaternion.getEuler();
  // printf("Euler get pitch angle: %f\n", euler.getPitchAngle());
  shoulderY.euler = euler;
  shoulderZ.euler = euler;
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmShoulder::updateGyroscope(BasePosition *position)
{
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmShoulder::updateAccelerometer(BasePosition *position)
{
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmShoulder::updateAccuracy(BasePosition *position)
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

    if (!isnan(angleY)) {
      shoulderY.setTargetAngle(angleY);
    }
    if (!isnan(angleZ)) {
      shoulderZ.setTargetAngle(angleZ);
    }
  }
  if (frame.id == CAN_SHOULDER_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}