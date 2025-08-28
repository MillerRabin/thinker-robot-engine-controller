#include "armClaw.h"

volatile QueueHandle_t ArmClaw::queue;
int counter = 0;

void ArmClaw::engineTask(void *instance) {
  printf("ArmClaw::engineTask started\n");
  auto *claw = static_cast<ArmClaw *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {    
    claw->clawX.setIMUAngle(claw->clawX.getPhysicalAngle());
    claw->clawY.setIMUAngle(claw->clawY.getPhysicalAngle());

    claw->setXCalibrating(claw->clawX.isCalibrating());
    claw->setYCalibrating(claw->clawY.isCalibrating());

    claw->clawX.tick();
    claw->clawY.tick();
    claw->clawGripper.tick();

    if (claw->clawX.isCalibrating() && claw->clawY.isCalibrating()) {
      claw->setHomeQuaternion(claw->imu.quaternion, claw->platform.imu.quaternion);
      claw->saveHomeQuaternionsToEEPROM();
      printf("Claw home quaternion set and saved to EEPROM\n");
    }

    claw->setEngineTaskStatus(true);
    claw->updateStatuses();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT));
  }
}

ArmClaw::ArmClaw(
    const uint8_t detectorsSdaPin,
    const uint8_t detectorsSclPin,
    const uint8_t engineXPin,
    const uint8_t engineYPin,
    const uint8_t engineGripperPin,
    const uint8_t canRxPin,
    const uint8_t canTxPin,
    const uint8_t memsRxPin,
    const uint8_t memsTxPin,
    const uint8_t memsRstPin,
    const uint8_t memsIntPin,
    const uint8_t shortDetectorShutPin,
    const uint8_t longDetectorShutPin) : ArmPart(canRxPin, canTxPin),
                                         clawX(engineXPin, Range(0, 180), CLAW_X_HOME_POSITION, 100),
                                         clawY(engineYPin, Range(0, 180), CLAW_Y_HOME_POSITION, 100),
                                         clawGripper(engineGripperPin, Range(0, 180), CLAW_GRIPPER_HOME_POSITION, 100),
                                         imu(this, memsRxPin, memsTxPin, memsRstPin)
                                         //rangeDetector(this, i2c1, longDetectorShutPin, shortDetectorShutPin) 
                                         {  
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(detectorsSdaPin, GPIO_FUNC_I2C);
  gpio_set_function(detectorsSclPin, GPIO_FUNC_I2C);
  gpio_pull_up(detectorsSdaPin);
  gpio_pull_up(detectorsSclPin);
  bi_decl(bi_2pins_with_func(DETECTORS_SDA_PIN, DETECTORS_SCL_PIN, GPIO_FUNC_I2C));
 
  if (loadHomeQuaternionsFromEEPROM()) {
    printf("Claw home quaternions loaded from EEPROM\n");
  } else {
    printf("No valid quaternion data found in EEPROM, using defaults\n");
    offsetQuaternion = getRotationQuaternion();
  }

  if (!xTaskCreate(ArmClaw::engineTask, "ArmClaw::engineTask", 2048, this, 5, NULL)) {
    printf("Failed to create ArmClaw::engineTask\n");
    setEngineTaskStatus(false);
  } else {
    printf("ArmClaw::engineTask created successfully\n");
    setEngineTaskStatus(true);
  }
}

int ArmClaw::updateQuaternion(IMUBase *position) {
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmClaw::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmClaw::updateAccelerometer(IMUBase *position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmClaw::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmClaw::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_CLAW_SET_XYG_DEGREE) {
    uint32_t raw1 = frame.data32[0];
    uint32_t raw2 = frame.data32[1];
    uint16_t angleYS = raw1 & 0xFFFF;
    uint16_t angleXS = (raw1 >> 16) & 0xFFFF;
    uint16_t angleGS = raw2 & 0xFFFF;
    uint16_t timeMS = (raw2 >> 16) & 0xFFFF;

    float angleX = angleXS / 100.0f;
    float angleY = angleYS / 100.0f;
    float angleG = angleGS / 100.0f;

    if (!isnan(angleX)) {
      clawX.setTargetAngle(angleX, timeMS, CLAW_DEAD_ZONE);
    }
    if (!isnan(angleY)) {
      clawY.setTargetAngle(angleY, timeMS, CLAW_DEAD_ZONE);
    }
    if (!isnan(angleG)) {
      clawGripper.setTargetAngle(angleG, timeMS, CLAW_DEAD_ZONE);
    }
  }
  if (frame.id == CAN_CLAW_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}