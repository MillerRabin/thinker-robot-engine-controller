#include "armShoulder.h"
#include "../common/periodic/periodic.h"

Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);                                  // East North Up

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  Periodic printer(pdMS_TO_TICKS(1000));
  TickType_t lastWakeTime = xTaskGetTickCount();  
  /*float maxGyro = 0;
  float maxAccel = 0;*/

  shoulder->imu.begin();
    
  while (true) {    
    if (!shoulder->getPositionStatus() || !shoulder->getTareError()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    
    auto sQuat = shoulder->imu.quaternion;
    auto accel = shoulder->imu.accelerometer;
    auto gyro = shoulder->imu.gyroscope;    
    auto pQuat = shoulder->platform.imu.quaternion;

    auto yAccel = accel.x;
    auto yGyro = gyro.y;
    auto zAccel = accel.z;
    auto zGyro = gyro.z;

    if (!sQuat.isValid() || !pQuat.isValid()) {      
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    auto sRel = pQuat.invert() * sQuat;
    auto sDiff = shoulder->offset.invert() * sRel;    
    auto ea = sDiff.swingTwistToAngles();
    //printf("Angles is x: %f, y: %f, z: %f\n", ea.roll * 180 / M_PI, ea.pitch * 180 / M_PI, ea.yaw * 180 / M_PI);
    /*auto sEuler = sQuat.getEuler();
    printf("Angles is x: %f, y: %f, z: %f\n", sEuler.roll * 180 / M_PI,
           sEuler.pitch * 180 / M_PI, sEuler.yaw * 180 / M_PI);*/

    /*if (fabs(cAccel) > fabs(maxAccel)) {
      maxAccel = cAccel;
    }

    if (fabs(cGyro) > fabs(maxGyro)) {
      maxGyro = cGyro;
    }*/

    /*printer.run([&maxGyro, &maxAccel]() { 
      printf("Max Gyro is %f, maxAccel is %f\n", maxGyro, maxAccel);
      maxGyro = 0.0f;
      maxAccel = 0.0f;
    });*/

    shoulder->tick(pQuat, sQuat);
    //shoulder->shoulderY.setIMUAngle(ea.pitch * 180 / M_PI);
    shoulder->shoulderY.setIMUAngle(ea.pitch * 180 / M_PI);
    shoulder->shoulderY.setAcceleration(yAccel);
    shoulder->shoulderY.setAngularSpeed(yGyro);
    shoulder->shoulderZ.setIMUAngle(shoulder->shoulderZ.getPhysicalAngle());
    shoulder->shoulderZ.setAcceleration(zAccel);
    shoulder->shoulderZ.setAngularSpeed(zGyro);
    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();
    
    shoulder->setEngineTaskStatus(true);
    shoulder->updateStatuses();
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
                     shoulderZ(engineZPin, Range(0, 180), SHOULDER_Z_HOME_POSITION, 100),
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
      this->imu.clearTare();
      this->imu.saveTare();
    }

    if (tareMask & ARM_SHOULDER) {
      this->imu.tare(TARE_AXIS_ALL);
      this->imu.saveTare();
      this->scheduleSave();      
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