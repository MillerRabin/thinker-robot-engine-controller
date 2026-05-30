#include "armClaw.h"

void ArmClaw::calibrateYLoop() {
  // LogQueue::Log("ArmClaw::calibrateYLoop started\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  setYCalibrating(true);
  float guessAngle = CLAW_Y_HOME_POSITION;
  float increment = 90.0f;
  int maxCounter = 40;
  Quaternion position = imu.quaternion.load();
  Quaternion origin = position.invert();
  for (int i = 0; i < maxCounter; i++) {
    clawY.setDegreeDirect(guessAngle);
    //LogQueue::Log("\nIteration %d, prevGuessAngle: %.3f, ", i, guessAngle);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(40));
    position = imu.quaternion.load();

    Quaternion delta = origin * position;
    Vector3 g = delta.getGravityVector();

    if (fabs(g.y) < 0.001f) {
      updateStatuses();
      continue;
    }

    origin = position.invert();
    float dir = (g.y > 0) ? -1.0f : 1.0f;
    increment = (increment > 0.1f) ? increment / 2.0f : 0.0f;
    guessAngle += dir * increment;
    //LogQueue::Log("currGuessAngle: %.3f, increment: %.3f, y: %.3f", guessAngle, increment, g.y);
    updateStatuses();
  }
  setYCalibrating(false);
}

void ArmClaw::calibrateXLoop() {    
  //LogQueue::Log("ArmClaw::calibrateXLoop started\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  setXCalibrating(true);
  float guessAngle = CLAW_X_HOME_POSITION;
  float increment = 90.0f;
  int maxCounter = 40;
  Quaternion position = imu.quaternion.load();
  Quaternion origin = position.invert();
  for (int i = 0; i < maxCounter; i++) {
    clawX.setDegreeDirect(guessAngle);        
    //LogQueue::Log("\nIteration %d, prevGuessAngle: %.3f, ", i, guessAngle);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(40));
    position = imu.quaternion.load();

    Quaternion delta = origin * position;
    Vector3 g = delta.getGravityVector();
    
    if (fabs(g.x) < 0.001f) {
      updateStatuses();
      continue;
    }

    origin = position.invert();
    float dir = (g.x < 0) ? -1.0f : 1.0f;
    increment = (increment > 0.1f) ? increment / 2.0f : 0.0f;
    guessAngle += dir * increment;
    //LogQueue::Log("currGuessAngle: %.3f, increment: %.3f, x: %.3f", guessAngle, increment, g.x);
    updateStatuses();
  }
  setXCalibrating(false);
}

void ArmClaw::calibrateLoop() {
  setArmCalibrated(false);
  LogQueue::Log("Callibration started\n");
  updateStatuses();
  //calibrateXLoop();
  //calibrateYLoop();  
  //setArmCalibrated(true);
  updateStatuses();
}

float ArmClaw::angleX(const Quaternion &q) {
  float s = std::fmax(-1.0f, std::fmin(1.0f, 2.0f * (q.real * q.j - q.k * q.i)));
  return std::asin(s);
}

float ArmClaw::angleY(const Quaternion &q) {
  float s = sqrt(q.i * q.i + q.j * q.j + q.k * q.k);
  return 2.0 * atan2(s, q.real);
}

Quaternion ArmClaw::makeRotationX(float angleX) {
  float h = angleX * 0.5f;
  return {std::cos(h), 0.0f, std::sin(h), 0.0f};
}

void ArmClaw::engineLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();

  vTaskDelay(pdMS_TO_TICKS(200));
  Quaternion origin = imu.quaternion.load().invert();
  vTaskDelay(pdMS_TO_TICKS(50));

  Quaternion position = imu.quaternion.load();
  
  static constexpr Vector3 X_AXIS = {1.0f, 0.0f, 0.0f};
  static constexpr Vector3 Y_AXIS = {0.0f, 1.0f, 0.0f};
  static constexpr Vector3 Z_AXIS = {0.0f, 0.0f, 1.0f};

  while (true) {
    position = imu.quaternion.load();
    Quaternion delta = origin * position;

    Vector3 g = delta.getGravityVector();
    Vector3 localX = delta.rotate({ 1.0f, 0.0f, 0.0f });
    //LogQueue::Log("Rotated: X: %.3f, Y: %.3f, Z: %.3f\n", localX.x, localX.y, localX.z);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void ArmClaw::engineTask(void *instance) {
  auto *claw = static_cast<ArmClaw *>(instance);
  Periodic printer(pdMS_TO_TICKS(1000));
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    auto wp = claw->imu.isPositionOK();
    auto pp = claw->platform.isPositionOK();    

    if (!wp || !pp) {
      claw->setEngineTaskStatus(false);
      claw->updateStatuses();      
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    Quaternion eQuat = claw->imu.quaternion.load();
    Quaternion pQuat = claw->platform.imu.quaternion.load();

    if (!eQuat.isValid() || !eQuat.isValid()) {
      claw->setEngineTaskStatus(false);
      claw->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    claw->setEngineTaskStatus(true);
    claw->updateStatuses();
    claw->calibrateLoop();
    claw->engineLoop();

    claw->setEngineTaskStatus(false);
    claw->updateStatuses();
    vTaskDelete(NULL);    
  }
}

ArmClaw::ArmClaw(const uint8_t detectorsSdaPin, const uint8_t detectorsSclPin,
                 const uint8_t engineXPin, const uint8_t engineYPin,
                 const uint8_t engineGripperPin, const uint8_t canRxPin,
                 const uint8_t canTxPin, const uint8_t memsRxPin,
                 const uint8_t memsTxPin, const uint8_t memsRstPin,
                 const uint8_t memsIntPin, const uint8_t shortDetectorShutPin,
                 const uint8_t longDetectorShutPin)
    : ArmPart(canRxPin, canTxPin),
      clawX(engineXPin, Range(0, 180), CLAW_X_HOME_POSITION, 100),
      clawY(engineYPin, Range(0, 180), CLAW_Y_HOME_POSITION, 100),
      clawGripper(engineGripperPin, Range(0, 180), CLAW_GRIPPER_HOME_POSITION,
                  100),
      imu(this, memsRxPin, memsTxPin, memsRstPin),
      rangeDetector(this, i2c1, longDetectorShutPin, shortDetectorShutPin) {
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(detectorsSdaPin, GPIO_FUNC_I2C);
  gpio_set_function(detectorsSclPin, GPIO_FUNC_I2C);
  gpio_pull_up(detectorsSdaPin);
  gpio_pull_up(detectorsSclPin);
  bi_decl(bi_2pins_with_func(DETECTORS_SDA_PIN, DETECTORS_SCL_PIN, GPIO_FUNC_I2C));

  if (taskHandle == NULL) {
    if (xTaskCreateAffinitySet(ArmClaw::engineTask, "ArmClaw::engineTask",
                               4096, this, 4, ARM_CORE,
                               &taskHandle) == pdFAIL) {
      printf("Failed to create ArmClaw engine task\n");
    } else {
    }
  }
}

int ArmClaw::updateQuaternion(IMUBase *position) {
  Quaternion quat = position->quaternion.load();
  return ArmPart::updateQuaternion(quat);
}

int ArmClaw::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmClaw::updateAccelerometer(IMUBase *position) {
  Accelerometer acc = position->accelerometer.load();
  return ArmPart::updateAccelerometer(acc);
}

int ArmClaw::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

int ArmClaw::updateHeight(IMUBase *position) {
  return ArmPart::updateHeight(position->height, position->temperature);
}

void ArmClaw::busReceiveCallback(can2040_msg frame) {  
  if (frame.id == CAN_CLAW_SET_XYG_DEGREE) {
    uint32_t raw1 = frame.data32[0];
    uint32_t raw2 = frame.data32[1];
    uint16_t angleYS = raw1 & 0xFFFF;
    uint16_t angleXS = (raw1 >> 16) & 0xFFFF;
    uint16_t angleGS = raw2 & 0xFFFF;
    uint16_t timeMS = (raw2 >> 16) & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    float angleX = (angleXS == PARAMETER_IS_NAN) ? NAN : angleXS / 100.0f;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 100.0f;
    float angleG = (angleGS == PARAMETER_IS_NAN) ? NAN : angleGS / 100.0f;

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

  if (frame.id == CAN_TARE) {
    uint32_t raw = frame.data32[0];
    uint16_t clearMask = raw & 0xFFFF;
    uint16_t tareMask = (raw >> 16) & 0xFFFF;

    if (clearMask & ARM_CLAW) {
      // this->imu.clearTare();
    }
    if (tareMask & ARM_CLAW) {    
      this->imu.tare();
      // this->imu.saveTare();
    }
  }

  if (frame.id == CAN_CLAW_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}