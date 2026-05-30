#include "armShoulder.h"
#include "../common/periodic/periodic.h"
#include "../common/quaternion/quaternion.h"
#include "../common/speedBuffer/speedBuffer.h"

void ArmShoulder::calibrateYLoop() {  
  setYCalibrating(true);  
  int maxCounter = 10;
  float imuY = angleFromGravityY();
  imuY = std::clamp(imuY, 0.0f, 180.0f);  
  for (int i = 0; i < maxCounter; i++) {
    shoulderY.setDegreeDirect(imuY);
    vTaskDelay(pdMS_TO_TICKS(50));
    updateStatuses();
  }
  setYCalibrating(false);
}

void ArmShoulder::calibrateZLoop() {
  setZCalibrating(true);
  shoulderZ.setDegreeDirect(SHOULDER_Z_HOME_POSITION);
  vTaskDelay(pdMS_TO_TICKS(2000));
  setZCalibrating(false);
}

void ArmShoulder::calibrateLoop() {
  setArmCalibrated(false);
  updateStatuses();
  calibrateYLoop();
  calibrateZLoop();
  vTaskDelay(pdMS_TO_TICKS(2000));
  Quaternion homeQ = getHomeQuaternion();
  base = homeQ.invert();
  setArmCalibrated(true);
  updateStatuses();
}

void ArmShoulder::engineLoop() {  
  TickType_t lastWakeTime = xTaskGetTickCount();
  Periodic printer(pdMS_TO_TICKS(500));      
  while (true) {
    auto sp = imu.isPositionOK();
    auto pp = platform.isPositionOK();
    if (!sp || !pp) {
      LogQueue::Log("Engine is turned off\n");
      break;
    }

    Vector3 imuAngles = getIMUAngles();
    Vector3 physicalAngles = getPhysicalAngles(imuAngles);
    Vector3 logicalAngles = getIMUAngles(0, shoulderY.getPhysicalAngle(), shoulderZ.getPhysicalAngle());
    /*printer.interval([&]() {
      LogQueue::Log("Y: %.3f %.3f %.3f, Z: %.3f %.3f %.3f\n", logicalAngles.y, imuAngles.y * RAD_TO_DEG, physicalAngles.y, logicalAngles.z, imuAngles.z * RAD_TO_DEG, physicalAngles.z);
    });*/

    shoulderY.setIMUAngle(physicalAngles.y);
    shoulderZ.setIMUAngle(physicalAngles.z);
    //shoulderY.setIMUAngle(shoulderY.getPhysicalAngle());
    //shoulderZ.setIMUAngle(shoulderZ.getPhysicalAngle());
    shoulderY.tick();
    shoulderZ.tick();

    updateStatuses();
    //vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
    vTaskDelayUntil(&lastWakeTime, taskInterval);
  }
}

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();


  while (true) {
    auto sp = shoulder->imu.isPositionOK();
    auto pp = shoulder->platform.isPositionOK();

    if (!sp || !pp) {
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    auto position = shoulder->imu.quaternion.load();
    auto platformPosition = shoulder->platform.imu.quaternion.load();

    if (!position.isValid() || !platformPosition.isValid()) {
      shoulder->setEngineTaskStatus(false);
      shoulder->updateStatuses();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    
    shoulder->setEngineTaskStatus(true);    
    shoulder->calibrateLoop();  
    shoulder->engineLoop();
    shoulder->setEngineTaskStatus(false);
    shoulder->updateStatuses();
    //LogQueue::Log("ArmShoulder engine task finished\n");
    //vTaskDelete(NULL);
  }
}

ArmShoulder::ArmShoulder(uint memsSCKPin, uint memsMISOPin, uint memsMOSIPin,
                         uint memsCSPin, uint memsIntPin, uint memsRstPin,
                         uint engineZPin, uint engineYPin, uint canRxPin,
                         uint canTxPin)
    : ArmPart(canRxPin, canTxPin),
      shoulderZ(engineZPin, Range(0, 270), SHOULDER_Z_HOME_POSITION, 330),
      shoulderY(engineYPin, Range(0, 180), SHOULDER_Y_HOME_POSITION, 330,
                0.00055),
      imu(this, memsSCKPin, memsMISOPin, memsMOSIPin, memsCSPin, memsRstPin,
          memsIntPin) {
    Quaternion q_corr = {0.7071068f, 0.0f, 0.0f, 0.7071068f};
    imu.setRotate(q_corr);
}

int ArmShoulder::updateQuaternion(IMUBase *position) {
  if (!base.isValid()) {
    return ERROR_PART_IS_NOT_CALIBRATED;
  }
  Quaternion quat = position->quaternion.load() * base;
  return ArmPart::updateQuaternion(quat);
}

int ArmShoulder::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmShoulder::updateAccelerometer(IMUBase *position) {
  Accelerometer acc = position->accelerometer.load();
  return ArmPart::updateAccelerometer(acc);
}

int ArmShoulder::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmShoulder::busReceiveCallback(can2040_msg frame) {

  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
    float angleZ = (angleZS == PARAMETER_IS_NAN) ? NAN : angleZS / 10.0f;
    uint32_t raw1 = frame.data32[1];
    uint16_t timeMS = raw1 & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    if (!isnan(angleY)) {
      shoulderY.setTargetAngle(angleY, timeMS, SHOULDER_DEAD_ZONE);
    }
    if (!isnan(angleZ)) {
      shoulderZ.setTargetAngle(angleZ, timeMS, SHOULDER_DEAD_ZONE);
    }
  }

  if (frame.id == CAN_SHOULDER_FIRMWARE_UPGRADE) {
    ArmPart::sendFirmwareUpgradeMessage();
    setUpgrading(true);
    updateStatuses();
    vTaskDelay(pdMS_TO_TICKS(1000));
    rebootInBootMode();
  }
}

int ArmShoulder::begin() {
  if (taskHandle == NULL) {
    if (xTaskCreateAffinitySet(ArmShoulder::engineTask,
                               "ArmShoulder::engineTask", 4096, this, 4,
                               ARM_CORE, &taskHandle) == pdFAIL) {
      printf("Failed to create ArmShoulder engine task\n");
      return ERROR_ENGINE_TASK_CREATION_FAILED;
    } else {}
  }
  return 0;
}

Vector3 ArmShoulder::getIMUAngles(float physicalX, float physicalY, float physicalZ) {
  return { 0, getIMUAngleY(physicalY), getIMUAngleZ(physicalZ) };  
}

Vector3 ArmShoulder::getIMUAngles() {
  Quaternion qm = imu.quaternion.load() * base;
  float yawZ = qm.twistAngle({0.0f, 0.0f, 1.0f});
  Quaternion qZ = Quaternion::AngleAxis(yawZ, 0.0f, 0.0f, 1.0f);
  Quaternion qSwing = qZ.invert() * qm;
  float pitchX = qSwing.twistAngle({yAxisSign, 0.0f, 0.0f});  
  return {0, pitchX, yawZ};
}

Vector3 ArmShoulder::getPhysicalAngles(Vector3 &imuAngles) {
  return { 0, (imuAngles.y * RAD_TO_DEG + SHOULDER_Y_HOME_POSITION), (imuAngles.z * RAD_TO_DEG + SHOULDER_Z_HOME_POSITION) };
}

float ArmShoulder::angleFromGravityY() {
  Accelerometer acc = imu.accelerometer.load();  
  float res = -1 * atan2(acc.y, acc.x) * RAD_TO_DEG;
  if (res < -45) {
    res = 360 + res;
  }
  return res;
}

float ArmShoulder::getIMUAngleY() {
  return shoulderY.getPhysicalAngle() - SHOULDER_Y_HOME_POSITION;
}

float ArmShoulder::getIMUAngleZ() {
  return shoulderZ.getPhysicalAngle() - SHOULDER_Z_HOME_POSITION;
}

float ArmShoulder::getIMUAngleY(float physicalY) {
  return physicalY - SHOULDER_Y_HOME_POSITION;
}

float ArmShoulder::getIMUAngleZ(float physicalZ) {
  return physicalZ - SHOULDER_Z_HOME_POSITION;
}

Quaternion ArmShoulder::getHomeQuaternion() {
    Quaternion imuQ = imu.quaternion.load();
    float dY = (shoulderY.getPhysicalAngle() - SHOULDER_Y_HOME_POSITION) * DEG_TO_RAD;
    float dZ = (shoulderZ.getPhysicalAngle() - SHOULDER_Z_HOME_POSITION) * DEG_TO_RAD;
    if (dY < 0.0f) {
      yAxisSign = -1.0f;
    }
    Quaternion qy = Quaternion::AngleAxis(dY, yAxisSign, 0.0f, 0.0f);
    Quaternion qz = Quaternion::AngleAxis(dZ, 0.0f, 0.0f, 1.0f);
    
    return (qz * qy).invert() * imuQ; 
}