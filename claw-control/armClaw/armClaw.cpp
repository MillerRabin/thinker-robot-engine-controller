#include "armClaw.h"

volatile QueueHandle_t ArmClaw::queue;
int counter = 0;

void ArmClaw::engineTask(void *instance)
{
  auto *claw = static_cast<ArmClaw *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true)
  {
    claw->clawX.tick();
    claw->clawZ.tick();

    Euler rEuler = claw->platform.imu.quaternion.getEuler();
    Euler sEuler = claw->imu.quaternion.getEuler();
    printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    printf("Claw roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle());
    printf("Dropped frames %d\n", Bus::getDroppedFrames());
    claw->setEngineTaskStatus(true);
    claw->updateStatuses();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT));
  }
}

ArmClaw::ArmClaw(
    const uint8_t detectorsSdaPin,
    const uint8_t detectorsSclPin,
    const uint8_t engineXPin,
    const uint8_t engineZPin,
    const uint8_t engineGripperPin,
    const uint8_t canRxPin,
    const uint8_t canTxPin,
    const uint8_t memsRxPin,
    const uint8_t memsTxPin,
    const uint8_t memsRstPin,
    const uint8_t memsIntPin,
    const uint8_t shortDetectorShutPin,
    const uint8_t longDetectorShutPin) : ArmPart(canRxPin, canTxPin),
                                         clawX(engineXPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
                                         clawZ(engineZPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
                                         clawGripper(engineGripperPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
                                         imu(this, memsRxPin, memsTxPin, memsRstPin),
                                         rangeDetector(this, i2c1, longDetectorShutPin, shortDetectorShutPin)
{
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(detectorsSdaPin, GPIO_FUNC_I2C);
  gpio_set_function(detectorsSclPin, GPIO_FUNC_I2C);
  gpio_pull_up(detectorsSdaPin);
  gpio_pull_up(detectorsSclPin);
  bi_decl(bi_2pins_with_func(DETECTORS_SDA_PIN, DETECTORS_SCL_PIN, GPIO_FUNC_I2C));
  xTaskCreate(ArmClaw::engineTask, "ArmClaw::engineTask", 1024, this, 5, NULL);
}

int ArmClaw::updateQuaternion(IMUBase *position)
{
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmClaw::updateGyroscope(IMUBase *position)
{
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmClaw::updateAccelerometer(IMUBase *position)
{
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmClaw::updateAccuracy(IMUBase *position)
{
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmClaw::busReceiveCallback(can2040_msg frame)
{
  if (frame.id == CAN_CLAW_SET_XZG_DEGREE)
  {
    uint32_t raw1 = frame.data32[0];
    uint32_t raw2 = frame.data32[1];
    uint16_t angleZS = (raw1 >> 16) & 0xFFFF;
    uint16_t angleXS = raw1 & 0xFFFF;
    uint16_t angleGS = (raw1 >> 16) & 0xFFFF;

    float angleX = angleXS / 100.0f;
    float angleZ = angleZS / 100.0f;
    float angleG = angleGS / 100.0f;

    if (!isnan(angleX))
    {
      clawX.setTargetAngle(angleX);
    }
    if (!isnan(angleZ))
    {
      clawZ.setTargetAngle(angleZ);
    }
    if (!isnan(angleG))
    {
      clawGripper.setTargetAngle(angleG);
    }
  }
  if (frame.id == CAN_SHOULDER_FIRMWARE_UPGRADE)
  {
    rebootInBootMode();
  }
}