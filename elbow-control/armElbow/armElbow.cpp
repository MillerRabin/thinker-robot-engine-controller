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
  Periodic printer(pdMS_TO_TICKS(1000));
  setEngineTaskStatus(false);
  setYCalibrating(true);  
  Vector3 iv = { 0, angleY(), 0};
  Vector3 gv = getPhysicalAngles(iv);
  int maxCount = 10;    
  for (int i = 0; i < maxCount; i++) {    
    elbowY.setDegreeDirect(gv.y);
    updateStatuses();
    vTaskDelay(pdMS_TO_TICKS(50));
  }    
  setYCalibrating(false);
}

void ArmElbow::engineTask(void *instance) {
  auto *elbow = static_cast<ArmElbow *>(instance);
  Periodic printer(pdMS_TO_TICKS(1000));
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    auto sp = elbow->imu.isPositionOK();
    auto pp = elbow->platform.isPositionOK();
    //auto sc = elbow->shoulder.isCalibrated();
    bool sc = false;

    if (!sp || !pp || !sc) {
      elbow->setEngineTaskStatus(false);
      elbow->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    
    Quaternion eQuat = elbow->imu.quaternion.load();
    Quaternion pQuat = elbow->platform.imu.quaternion.load();

    if (!eQuat.isValid() || !eQuat.isValid()) {
      elbow->setEngineTaskStatus(false);
      elbow->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    elbow->setEngineTaskStatus(true);
    elbow->updateStatuses();
    elbow->calibrateLoop();
    Vector3 ia = elbow->getIMUAngles();
    Vector3 pa = elbow->getPhysicalAngles(ia);
    LogQueue::Log("Y angle is %.3f %.3f %.3f\n", elbow->elbowY.getPhysicalAngle(), ia.y * RAD_TO_DEG, pa.y);
    elbow->engineLoop();

    elbow->setEngineTaskStatus(false);
    elbow->updateStatuses();
    vTaskDelete(NULL);
  }
}

void ArmElbow::engineLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    /*Quaternion delta = positionOrigin.invert() * position;
    auto euler = delta.swingTwistToAngles();
    //shoulderY.setIMUAngle(physY);
    //shoulderZ.setIMUAngle(physZ);
    shoulderY.tick();
    shoulderZ.tick();
    updateStatuses();*/
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

ArmElbow::ArmElbow(const uint memsSdaPin, const uint memsSclPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineYPin, const uint canRxPin,
                   const uint canTxPin)
    : ArmPart(canRxPin, canTxPin), elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100),
      imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {}

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

Vector3 ArmElbow::getIMUAngles(float physicalX, float physicalY,
                               float physicalZ) {
  return {0, physicalY - 45, 0};
}

Vector3 ArmElbow::getIMUAngles() {
  Quaternion qm = base * imu.quaternion.load();
  float pitch = qm.twistAngle({0.0f, 1.0f, 0.0f});  
  return {0, pitch, 0};
}
