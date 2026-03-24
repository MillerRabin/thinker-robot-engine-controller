#include "armElbow.h"
#include "../common/periodic/periodic.h"

void ArmElbow::engineTask(void *instance)
{
  auto *elbow = static_cast<ArmElbow *>(instance);      
  Periodic printer(pdMS_TO_TICKS(1000));
  elbow->imu.begin();
  TickType_t lastWakeTime = xTaskGetTickCount();

  
  while (true)
  {
    if (!elbow->getPositionStatus() || !elbow->getTareError()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    auto eQuat = elbow->imu.quaternion;
    auto pQuat = elbow->platform.imu.quaternion;
    
    if (!eQuat.isValid() || !pQuat.isValid()) {      
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    
    auto accel = elbow->imu.accelerometer;
    auto gyro = elbow->imu.gyroscope;
    auto yAccel = accel.x;
    auto yGyro = gyro.y;
    
    auto sRel = pQuat.invert() * eQuat;
    auto sDiff = elbow->offset.invert() * sRel;
    auto ea = sDiff.swingTwistToAngles();
    auto eEuler = eQuat.getEuler();
    float eaDeg = (ea.pitch * 180 / M_PI) + 45;
    
    elbow->tick(pQuat, eQuat);
    elbow->elbowY.setIMUAngle(eaDeg);
    elbow->elbowY.setAcceleration(yAccel);
    elbow->elbowY.setAngularSpeed(yGyro);
    elbow->elbowY.tick();
            
    elbow->setEngineTaskStatus(true);

    elbow->updateStatuses();    
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ENGINE_TASK_LOOP_TIMEOUT));
  }
}

ArmElbow::ArmElbow(
    const uint memsSdaPin,
    const uint memsSclPin,
    const uint memsIntPin,
    const uint memsRstPin,
    const uint engineYPin,
    const uint canRxPin,
    const uint canTxPin) : ArmPart(canRxPin, canTxPin),
                           elbowY(engineYPin, Range(0, 270), ELBOW_Y_HOME_POSITION, 100),
                           imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
{}

int ArmElbow::begin() {
  if (xTaskCreate(ArmElbow::engineTask, "ArmElbow::engineTask", 4096, this, 5, NULL) == pdFAIL) {
    printf("Failed to create ArmElbow engine task\n");
    return ERROR_ENGINE_TASK_CREATION_FAILED;
  }
  return 0;
}

int ArmElbow::updateQuaternion(IMUBase *position)
{  
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmElbow::updateGyroscope(IMUBase *position)
{
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmElbow::updateAccelerometer(IMUBase *position)
{
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmElbow::updateAccuracy(IMUBase *position)
{
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmElbow::busReceiveCallback(can2040_msg frame)
{
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

  if (frame.id == CAN_TARE) {
    uint32_t raw = frame.data32[0];
    uint16_t clearMask = raw & 0xFFFF;
    uint16_t tareMask = (raw >> 16) & 0xFFFF;
      
    if (clearMask & ARM_ELBOW) {
      this->imu.clearTare();
    }
    if (tareMask & ARM_ELBOW) {
      this->imu.tare(TARE_AXIS_ALL);
      this->imu.saveTare();
      this->scheduleSave();
    }    
  }

  if (frame.id == CAN_ELBOW_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}