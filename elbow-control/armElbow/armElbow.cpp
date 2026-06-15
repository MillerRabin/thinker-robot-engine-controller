#include "armElbow.h"
#include "../common/periodic/periodic.h"

void ArmElbow::calibrateLoop() {  
  setArmCalibrated(false);
  updateStatuses();
  calibrateYLoop();  
  //setArmCalibrated(true);
  updateStatuses();
}

void ArmElbow::calibrateYLoop() {
  setYCalibrating(true);
  int maxCounter = 10;  
  float imuY = angleFromGravityY();
  imuY = std::clamp(imuY, 0.0f, 180.0f);      
  for (int i = 0; i < maxCounter; i++) {
    //elbowY.setDegreeDirect(imuY);
    vTaskDelay(pdMS_TO_TICKS(50));
    updateStatuses();
  }
  setYCalibrating(false);
}


void ArmElbow::engineTask(void *instance) {
  auto *elbow = static_cast<ArmElbow *>(instance);  
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    auto sp = elbow->imu.isPositionOK();
    auto pp = elbow->shoulder.isPositionOK();        
    if (!sp || !pp ) {
      elbow->setEngineTaskStatus(false);
      elbow->updateStatuses();
      LogQueue::Log("Engine task failed sp-%d, pp-%d\n", sp, pp);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    
    Quaternion eQuat = elbow->imu.quaternion.load();
    Quaternion sQuat = elbow->shoulder.imu.quaternion.load();

    if (!eQuat.isValid() || !sQuat.isValid()) {
      elbow->setEngineTaskStatus(false);
      elbow->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    elbow->setEngineTaskStatus(true);
    elbow->updateStatuses();
    elbow->calibrateLoop();
    elbow->engineLoop();

    elbow->setEngineTaskStatus(false);
    elbow->updateStatuses();    
  }
}

void ArmElbow::engineLoop() {
  LogQueue::Log("Starting Engine Loop\n");
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));
  base = shoulder.imu.quaternion.load().invert();
  while (true) {
    auto sp = imu.isPositionOK();
    auto pp = shoulder.isPositionOK();
    if (!sp || !pp) {
      LogQueue::Log("Engine is turned off\n");
      break;
    }

    Vector3 imuAngles = getIMUAngles();
    Vector3 physicalAngles = getPhysicalAngles(imuAngles);
    /*printer.interval([&]() {
      LogQueue::Log("Y: %.2f %.2f, Z: %.2f %.2f\n",
                    physicalAngles.y, imuAngles.y * RAD_TO_DEG,
                    physicalAngles.z, imuAngles.z * RAD_TO_DEG);
    });*/

    elbowY.setIMUAngle(physicalAngles.y);
    //shoulderZ.setIMUAngle(physicalAngles.z);    
    //elbowY.tick();    
    updateStatuses();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
    //vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

ArmElbow::ArmElbow(const uint memsSdaPin, const uint memsSclPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineYPin, const uint canRxPin,
                   const uint canTxPin) : ArmPart(canRxPin, canTxPin), elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100),
                                          imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {
  
  
  //Quaternion q_corr = {0.0f, 0.7071068f, 0.0f, 0.7071068f};
  //Quaternion q_corr = {0.7071068f, 0.0f, 0.0f, 0.7071068f};

  // Вариант 1: -90° Z (текущий)
  //Quaternion q_corr = {0.0f, 0.0f, -0.7071068f, 0.7071068f};

  // Вариант 2: +90° Z
  //Quaternion q_corr = {0.0f, 0.0f, 0.7071068f, 0.7071068f};

  // Вариант 3: +90° X
  //Quaternion q_corr = {0.7071068f, 0.0f, 0.0f, 0.7071068f};

  // Вариант 4: -90° X
  //Quaternion q_corr = {-0.7071068f, 0.0f, 0.0f, 0.7071068f};

  // Вариант 5: +90° Y
  //Quaternion q_corr = {0.0f, 0.7071068f, 0.0f, 0.7071068f};

  // Вариант 6: -90° Y
  //Quaternion q_corr = {0.0f, -0.7071068f, 0.0f, 0.7071068f};  
  //Quaternion q_corr = {0.0f, 0.0f, 0.7071068f, 0.7071068f};
  //imu.setRotate(q_corr);
}

int ArmElbow::begin() {
  if (xTaskCreateAffinitySet(ArmElbow::engineTask, "ArmElbow::engineTask",
                             4096, this, 4, ARM_CORE, &taskHandle) == pdFAIL) {
    printf("Failed to create ArmElbow engine task\n");
    return ERROR_ENGINE_TASK_CREATION_FAILED;
  } else {
  }
  return 0;
}

int ArmElbow::updateQuaternion(IMUBase *position) {
  Quaternion quat = position->quaternion.load();
  return ArmPart::updateQuaternion(quat);
}

int ArmElbow::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmElbow::updateAccelerometer(IMUBase *position) {  
  Accelerometer acc = position->accelerometer.load();
  return ArmPart::updateAccelerometer(acc);
}

int ArmElbow::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmElbow::busReceiveCallback(can2040_msg frame) {
  shoulder.dispatchMessage(frame);

  if (frame.id == CAN_ELBOW_SET_Y_DEGREE) {

    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
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

float ArmElbow::angleY() {
  Accelerometer acc = imu.accelerometer.load();  
  return std::atan2(acc.y, acc.z);    
}

Vector3 ArmElbow::getPhysicalAngles(Vector3 &imuAngles) {
  return {0, (imuAngles.y * RAD_TO_DEG) + 45, 0};
}

float ArmElbow::angleFromGravityY() {
  Accelerometer acc = imu.accelerometer.load();
  float res = atan2(acc.y, acc.z) * RAD_TO_DEG;
  /*if (res < -45) {
    res = 360 + res;
  }*/
  return res;
}

Vector3 ArmElbow::getIMUAngles() {
  Quaternion sq = shoulder.imu.quaternion.load();
  Quaternion qm = imu.quaternion.load();
  float yawX = qm.twistAngle({1.0f, 0.0f, 0.0f});
  float yawY = qm.twistAngle({0.0f, 1.0f, 0.0f});
  float yawZ = qm.twistAngle({0.0f, 0.0f, 1.0f});
  LogQueue::Log("Raw IMU angles: X: %.2f, Y: %.2f, Z: %.2f\n", yawX * RAD_TO_DEG, yawY * RAD_TO_DEG, yawZ * RAD_TO_DEG);
  Quaternion qZ = Quaternion::AngleAxis(yawZ, 0.0f, 0.0f, 1.0f);
  Quaternion qSwing = qZ.invert() * qm;
  float pitchY = qSwing.twistAngle({0.0f, 1.0f, 0.0f});
  return {0, pitchY, yawY};
}
