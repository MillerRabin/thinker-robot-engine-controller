#include "armWrist.h"
#include "../common/periodic/periodic.h"

void ArmWrist::engineTask(void *instance) {
  auto *wrist = static_cast<ArmWrist *>(instance);
  Periodic printer(pdMS_TO_TICKS(1000));
  TickType_t lastWakeTime = xTaskGetTickCount();
  float gyroMax = 0;
  float accelMax = 0;
  const char *gyroAxis = "NONE";
  const char *accelAxis = "NONE";

  wrist->imu.begin();

  while (true) {
    if (!wrist->getPositionStatus() || !wrist->getTareError()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    auto sQuat = wrist->imu.quaternion;
    auto accel = wrist->imu.accelerometer;
    auto gyro = wrist->imu.gyroscope;
    auto pQuat = wrist->platform.imu.quaternion;

    auto yAccel = accel.y;
    auto yGyro = gyro.y;
    auto zAccel = accel.z;
    auto zGyro = gyro.z;

    if (!sQuat.isValid() || !pQuat.isValid()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    /*if (fabs(accelMax) < fabs(accel.x)) { accelMax = accel.x; accelAxis = "X";
    }
    //if (fabs(accelMax) < fabs(accel.y)) { accelMax = accel.y; accelAxis = "Y";
    }
    //if (fabs(accelMax) < fabs(accel.z)) { accelMax = accel.z; accelAxis = "Z";
    } if (fabs(gyroMax) < fabs(gyro.x)) { gyroMax = gyro.x; gyroAxis = "X"; } if
    (fabs(gyroMax) < fabs(gyro.y)) { gyroMax = gyro.y; gyroAxis = "Y"; } if
    (fabs(gyroMax) < fabs(gyro.z)) { gyroMax = gyro.z; gyroAxis = "Z"; }*/

    auto sRel = pQuat.invert() * sQuat;
    auto sDiff = wrist->offset.invert() * sRel;
    auto ea = sDiff.swingTwistToAngles();
    float eaDeg = (ea.pitch * 180 / M_PI) + 45;

    /*printer.run([ea]() {
      printf("Angle is roll: %f, pitch: %f, yaw: %f\n", ea.roll * 180 / M_PI,
    ea.pitch * 180 / M_PI, ea.yaw * 180 / M_PI);
    });*/

    wrist->tick(pQuat, sQuat);
    wrist->wristY.setIMUAngle(eaDeg);
    wrist->wristZ.setIMUAngle(wrist->wristZ.getPhysicalAngle());
    wrist->wristY.tick();
    wrist->wristZ.tick();

    wrist->setEngineTaskStatus(true);
    wrist->updateStatuses();

    vTaskDelayUntil(&lastWakeTime, wrist->taskInterval);
  }
}

ArmWrist::ArmWrist(const uint memsSdaPin, const uint memsSclPin,
                   const uint memsIntPin, const uint memsRstPin,
                   const uint engineZPin, const uint engineYPin,
                   const uint canRxPin, const uint canTxPin)
    : ArmPart(canRxPin, canTxPin),
      wristZ(engineZPin, Range(0, 270), WRIST_Z_HOME_POSITION, 100),
      wristY(engineYPin, Range(0, 180), NAN, 100),
      imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin) {
  if (!xTaskCreate(ArmWrist::engineTask, "ArmWrist::engineTask", 1024, this, 5,
                   NULL)) {
    setEngineTaskStatus(false);
  } else {
    setEngineTaskStatus(true);
  }
}

int ArmWrist::updateQuaternion(IMUBase *position) {
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmWrist::updateGyroscope(IMUBase *position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmWrist::updateAccelerometer(IMUBase *position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmWrist::updateAccuracy(IMUBase *position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmWrist::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_WRIST_SET_YZ_DEGREE) {
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;

    float angleY = (angleYS == PARAMETER_IS_NAN) ? NAN : angleYS / 10.0f;
    float angleZ = (angleZS == PARAMETER_IS_NAN) ? NAN : angleZS / 10.0f;
    uint32_t raw1 = frame.data32[1];
    uint16_t timeMS = raw1 & 0xFFFF;
    timeMS = (timeMS == PARAMETER_IS_NAN) ? 0 : timeMS;

    // printf("Wrist angle is set y: %f, z: %f, timeMS: %d\n", angleY, angleZ,
    // timeMS);

    if (!isnan(angleY)) {
      wristY.setTargetAngle(angleY, timeMS, WRIST_DEAD_ZONE);
    }
    if (!isnan(angleZ)) {
      wristZ.setTargetAngle(angleZ, timeMS, WRIST_DEAD_ZONE);
    }
  }

  if (frame.id == CAN_TARE) {
    uint32_t raw = frame.data32[0];
    uint16_t clearMask = raw & 0xFFFF;
    uint16_t tareMask = (raw >> 16) & 0xFFFF;

    if (clearMask & ARM_WRIST) {
      this->imu.clearTare();
      this->imu.saveTare();
    }
    if (tareMask & ARM_WRIST) {
      this->imu.tare(TARE_AXIS_ALL);
      this->imu.saveTare();
      this->scheduleSave();
    }
  }

  if (frame.id == CAN_WRIST_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}