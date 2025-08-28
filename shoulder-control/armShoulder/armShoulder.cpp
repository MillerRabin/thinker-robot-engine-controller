#include "armShoulder.h"

float ArmShoulder::getYawAngle(Euler euler) {
  float yaw = euler.getYawAngle();
  if (yaw > 90) yaw = yaw - 180;
  if (yaw < -90) yaw = yaw + 180;  
  return yaw;
}

struct Vec3
{
  double x, y, z;
};

static inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static inline double dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline double norm(const Vec3 &v) { return std::sqrt(dot(v, v)); }
static inline Vec3 normalize(Vec3 v)
{
  double n = norm(v);
  return (n > 1e-12) ? Vec3{v.x / n, v.y / n, v.z / n} : Vec3{0, 0, 0};
}

// Поворот вектора кватернионом (q — единичный)
static inline Vec3 rotateByQuat(const Quaternion &q, const Vec3 &v)
{
  Quaternion p{0, v.x, v.y, v.z};
  Quaternion qi = Quaternion::Conjugate(q);
  Quaternion r = Quaternion::Multiply(Quaternion::Multiply(q, p), qi);
  return {r.i, r.j, r.k};
}

// Строим базис (a, b, c), где b ⟂ a
static inline void buildBasis(const Vec3 &a_unit, Vec3 &b, Vec3 &c)
{
  Vec3 t = (std::fabs(a_unit.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  b = normalize(cross(a_unit, t));
  c = cross(a_unit, b);
}

// Подписанный угол вокруг оси a (ось в той же СК, что и qRel)
double signedAngleAroundAxis(const Quaternion &qHome, const Quaternion &qNow, double ax, double ay, double az)
{
  Vec3 a = normalize({ax, ay, az});
  Quaternion qRel = Quaternion::Normalize(Quaternion::Multiply(Quaternion::Conjugate(qHome), qNow));

  // опорный вектор в плоскости ⟂ a
  Vec3 b, c;
  buildBasis(a, b, c);

  // повернули b кватернионом qRel
  Vec3 b2 = rotateByQuat(qRel, b);

  // синус и косинус угла в плоскости ⟂ a
  double s = dot(a, cross(b, b2)); // подписанный «синус» (вдоль оси)
  double coss = dot(b, b2);        // косинус
  return std::atan2(s, coss);      // угол в (-π, π]
}

void ArmShoulder::engineTask(void *instance) {
  auto *shoulder = static_cast<ArmShoulder *>(instance);
  TickType_t lastWakeTime = xTaskGetTickCount();

  //Quaternion baseQuat = Quaternion::FromEuler(0, 0, 0);
  while (true) {
    /*Quaternion imuQuat = Quaternion(shoulder->imu.quaternion);
    Quaternion homeInv = Quaternion::Conjugate(shoulder->homeQuaternion);
    Quaternion homeQuat1 = Quaternion::Multiply(baseQuat, homeInv);

    Quaternion diff = shoulder->difference(imuQuat, shoulder->homeQuaternion);
    Euler euler = diff.getEuler();    
    printf("Diff: real: %f, i: %f, j: %f, k: %f\n", diff.real, diff.i, diff.j, diff.k);
    printf("Diff: roll: %f, pitch: %f, yaw: %f\n", euler.getRollAngle(), euler.getPitchAngle(), euler.getYawAngle());

    double angle = signedAngleAroundAxis(baseQuat, diff, 0, 0, 1);
    printf("zAngle: %f\n", angle * RAD_TO_DEG);*/
    //shoulder->shoulderY.maxDegree - shoulder->getYawAngle(euler) - SHOULDER_Y_HOME_POSITION
    shoulder->shoulderY.setIMUAngle(shoulder->shoulderY.getPhysicalAngle());
    shoulder->shoulderZ.setIMUAngle(shoulder->shoulderZ.getPhysicalAngle());
    shoulder->shoulderY.tick();
    shoulder->shoulderZ.tick();

    shoulder->setYCalibrating(shoulder->shoulderY.isCalibrating());
    shoulder->setZCalibrating(shoulder->shoulderZ.isCalibrating());

    if (shoulder->shoulderY.isCalibrating() && shoulder->shoulderZ.isCalibrating()) {
      shoulder->setHomeQuaternion(shoulder->imu.quaternion, shoulder->platform.imu.quaternion);
      shoulder->saveHomeQuaternionsToEEPROM();
      printf("Shoulder home quaternion set and saved to EEPROM\n");      
    }

    shoulder->setEngineTaskStatus(true);
    
    shoulder->updateStatuses();
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
                           shoulderY(engineYPin, Range(0, 180), SHOULDER_Y_HOME_POSITION, 100),
                           imu(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)                           
{  
  if (loadHomeQuaternionsFromEEPROM()) {
    printf("Shoulder home quaternions loaded from EEPROM\n");
  }
  else {
    printf("No valid quaternion data found in EEPROM, using defaults\n");
    offsetQuaternion = getRotationQuaternion();
  }
  if (!xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 1024, this, 5, NULL)) {
    setEngineTaskStatus(false);
  }
  else {
    setEngineTaskStatus(true);
  }
}

int ArmShoulder::updateQuaternion(IMUBase *position)
{  
  return ArmPart::updateQuaternion(position->quaternion);
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
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE)
  {    
    uint32_t raw = frame.data32[0];
    uint16_t angleYS = raw & 0xFFFF;
    uint16_t angleZS = (raw >> 16) & 0xFFFF;
    float angleY = angleYS / 100.0f;
    float angleZ = angleZS / 100.0f;
    uint32_t raw1 = frame.data32[1];
    uint16_t timeMS = raw1 & 0xFFFF;

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

Quaternion ArmShoulder::getRotationQuaternion() {
  float rollOffset = -90.0f * (M_PI / 180.0f);
  float pitchOffset = 90.0f * (M_PI / 180.0f);
  float yawOffset = 0.0f * (M_PI / 180.0f);
  Quaternion errorQuat = Quaternion::FromEuler(rollOffset, pitchOffset, yawOffset);
  Quaternion correctionQuat = Quaternion::Conjugate(errorQuat);
  return Quaternion::Multiply(correctionQuat, {0, 0, 0, 1});
};