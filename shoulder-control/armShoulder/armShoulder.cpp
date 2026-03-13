#include "armShoulder.h"

Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);                                  // East North Up

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool imuInitialized = false;

  auto imuRes = shoulder->imu.begin();
  if (!imuRes) {
    printf("Failed to initialize LocalBNO IMU with code %d\n", imuRes);
    imuInitialized = true;
  }

  while (true) {
    auto sQuat = Quaternion(shoulder->imu.quaternion);
    auto sEuler = sQuat.getEuler();
    float yAngle = 90 + (sEuler.pitch * 180.0f / M_PI);    
    shoulder->shoulderY.setIMUAngle(yAngle);    
    shoulder->shoulderZ.setIMUAngle(shoulder->shoulderZ.getPhysicalAngle());
    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();
    
    shoulder->setEngineTaskStatus(true);    
    //shoulder->updateStatuses();
    //printf("Shoulder engine task tick\n");
    
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT));
  }
}

ArmShoulder::ArmShoulder(
    uint memsSCKPin,
    uint memsMISOPin,
    uint memsMOSIPin,
    uint memsCSPin,
    uint memsIntPin,
    uint memsRstPin,
    uint engineZPin,
    uint engineYPin,
    uint canRxPin,
    uint canTxPin) : ArmPart(canRxPin, canTxPin),
                     shoulderZ(engineZPin, Range(0, 270), SHOULDER_Z_HOME_POSITION, 100),
                     shoulderY(engineYPin, Range(0, 180), NAN, 100, 0.00055),
                     imu(this, memsSCKPin, memsMISOPin, memsMOSIPin, memsCSPin, memsRstPin, memsIntPin)
{}

int ArmShoulder::updateQuaternion(IMUBase *position) {  
  auto resQuat = ArmShoulder::rotationQuaternion * position->quaternion;
  return ArmPart::updateQuaternion(resQuat); 
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

void ArmShoulder::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;
    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 100.0f;
    float angleZ = (angleZS == PARAMETER_IS_NAN) ? NAN : angleZS / 100.0f;
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
  if (frame.id == CAN_TARE) {
    uint32_t raw = frame.data32[0];
    uint16_t clearMask = raw & 0xFFFF;
    uint16_t tareMask = (raw >> 16) & 0xFFFF;
      
    if (clearMask & ARM_SHOULDER) {
      //this->imu.clearTare();
    }

    if (tareMask & ARM_SHOULDER) {
      //this->imu.tare(TARE_AXIS_ALL);
      //this->imu.saveTare();
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
  if (xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 4096, this, 5, NULL) == pdFAIL) {
    printf("Failed to create ArmShoulder engine task\n");
    return ERROR_ENGINE_TASK_CREATION_FAILED;
  }  
  return 0;
}