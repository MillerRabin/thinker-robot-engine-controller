#include "armShoulder.h"

//Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f); // North Down West

//Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, -sqrt(2.0f) / 2.0f, 0.0f, 0.0f); // North East Down

//Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, 1.0f, 0.0f); // West South Up

// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, 1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);    // South Down East
//Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, 0.0f, -sqrt(2.0f) / 2.0f); // East Down North
//Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, -sqrt(2.0) / 2.0f, 0.0f); // West Down South

//Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, sqrt(2.0) / 2.0f, sqrt(2.0f) / 2.0f);         // North West Up



//Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f);    // North Up East

//Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, -sqrt(2.0f) / 2.0f, 0.0f);      // Down South West
//Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, 0.0f, -sqrt(2.0f) / 2.0);       // Up North West
//Quaternion ArmShoulder::rotationQuaternion = Quaternion(1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);    // South Up West

//Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f, 0.0f, 0.0f, 0.0f); // West North Down

Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);                                  // East North Up
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, sqrt(2.0) / 2.0f, sqrt(2.0f) / 2.0f);         // North West Up
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, 1.0f, 0.0f);                                  // West South Up
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, 0.0f, -sqrt(2.0f) / 2.0f, sqrt(2.0f) / 2.0f);       // South East Up
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, -1.0f, 0.0f, 0.0f);                                 // East South Down
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, -sqrt(2.0f) / 2.0f, 0.0f, 0.0f);      // North East Down
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f, 0.0f, 0.0f, 0.0f);                                 // West North Down
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0) / 2.0f, sqrt(2.0) / 2.0f, 0.0f, 0);            // South West Down
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, -sqrt(2.0) / 2.0f, sqrt(2.0) / 2.0f, 0);            // Up South East
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f);    // North Up East
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, 0.0f, sqrt(2.0f) / 2.0f);       // Down North East
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, 1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);    // South Down East
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, 0.0f, -sqrt(2.0f) / 2.0);       // Up North West
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f);  // North Down West
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, -sqrt(2.0f) / 2.0f, 0.0f);      // Down South West
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);    // South Up West
 //Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f, -1.0f / 2.0f);   // Up East North
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, sqrt(2.0f) / 2.0f, 0.0f);       // West Up North
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f);     // Down West North
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, 0.0f, -sqrt(2.0f) / 2.0f);      // East Down North
//Quaternion ArmShoulder::rotationQuaternion = Quaternion(1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f);     // Up West South
// Quaternion ArmShoulder::rotationQuaternion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, -sqrt(2.0) / 2.0f, 0.0f);       // West Down South
//Quaternion ArmShoulder::rotationQuaternion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);   // Down East South
//Quaternion ArmShoulder::rotationQuaternion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, 0.0f, sqrt(2.0f) / 2.0f);       // East Up South

float ArmShoulder::tiltAngle(Accelerometer acc) {
  //const float g = 9.80665;
  const float g = 9.5f;
  
  const float sign = acc.bnoY() >= -1 ? 1 : -1;

  float x = acc.bnoX();
  float det = x / g;
  if (det > 1.0f)
    det = 1.0f;
  if (det < -1.0f)
    det = -1.0f;
  const float angleRad = acos(det);
  if (!isfinite(angleRad))
    return 0;  
  return sign * angleRad * 180 / M_PI;
}

float ArmShoulder::getAccelerometerAngleY() {
  Accelerometer sAcc = this->imu.accelerometer;
  Accelerometer pAcc = this->platform.imu.accelerometer;

  float sAngle = ArmShoulder::tiltAngle(sAcc);
  float pAngle = 90 - ArmShoulder::tiltAngle(pAcc);
  return sAngle + pAngle;
}

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    float sAngle = shoulder->getAccelerometerAngleY();
        
    shoulder->shoulderY.setIMUAngle(sAngle);    
    shoulder->shoulderZ.setIMUAngle(shoulder->shoulderZ.getPhysicalAngle());
    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();

    if (shoulder->shoulderY.isCalibrating() && shoulder->shoulderZ.isCalibrating()) {
      shoulder->setHomeQuaternion(shoulder->imu.quaternion, shoulder->platform.imu.quaternion);
      shoulder->saveHomeQuaternionsToEEPROM();
      printf("Shoulder home quaternion set and saved to EEPROM\n");      
    }

    shoulder->setEngineTaskStatus(true);    
    //shoulder->updateStatuses();
    //vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
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
                           shoulderZ(engineZPin, Range(0, 270), SHOULDER_Z_HOME_POSITION, 100),
                           shoulderY(engineYPin, Range(0, 180), NAN, 100),
                           imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)                           
{  
  if (loadHomeQuaternionsFromEEPROM()) {
    printf("Shoulder home quaternions loaded from EEPROM\n");
  }
  else {
    printf("No valid quaternion data found in EEPROM, using defaults\n");
    //offsetQuaternion = getRotationQuaternion();
  }
  if (!xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 1024, this, 5, NULL)) {
    setEngineTaskStatus(false);
  }
  else {
    setEngineTaskStatus(true);
  }
}

int ArmShoulder::updateQuaternion(IMUBase *position) {
  auto resQuat = ArmShoulder::rotationQuaternion * position->quaternion;
  return ArmPart::updateQuaternion(resQuat);
  //return ArmPart::updateQuaternion(position->quaternion);
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

void ArmShoulder::busReceiveCallback(can2040_msg frame)
{
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {
    this->imu.tare(TARE_AXIS_X + TARE_AXIS_Y + TARE_AXIS_Z);
    this->imu.saveTare();
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
    }
    if (tareMask & ARM_SHOULDER) {
      this->imu.tare(TARE_AXIS_ALL);
      this->imu.saveTare();
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