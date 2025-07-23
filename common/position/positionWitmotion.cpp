#include "positionWitmotion.h"

uint32_t PositionWitMotion::notificationIndex = 1;
TaskHandle_t PositionWitMotion::compassTaskHandle;


void PositionWitMotion::compassTask(void* instance) {  
  PositionWitMotion* position = (PositionWitMotion*)instance;  
  WitMotion imu = position->imu;
  while (true) {        
    position->quaternion.fromWitmotion(imu.rawQuatI, imu.rawQuatJ, imu.rawQuatK, imu.rawQuatReal, 32768.0f);
    if (position->armPart->updateQuaternion(position) != 0) {
      printf("quat sending error\n");
    }

    position->updateAccelerometerData(imu.rawLinAccelX, imu.rawLinAccelY, imu.rawLinAccelZ);
    if (position->armPart->updateAccelerometer(position) != 0) {
      printf("Accelerometer sending error\n");
    }

    position->updateGyroscopeData(imu.rawGyroX, imu.rawGyroZ, imu.rawGyroZ);
    if (position->armPart->updateGyroscope(position) != 0) {
      printf("Gyro sending error\n");
    }

    position->updateHeightData(imu.rawHeight);
    if (position->armPart->updateHeight(position) != 0) {
      printf("Height sending error\n");
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);                
  }
}

bool PositionWitMotion::updateAccelerometerData(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ) {
  uint16_t xd = (rawAccX > accelerometer.x) ? rawAccX - accelerometer.x : accelerometer.x - rawAccX;
  uint16_t yd = (rawAccY > accelerometer.y) ? rawAccY - accelerometer.y : accelerometer.y - rawAccY;
  uint16_t zd = (rawAccZ > accelerometer.z) ? rawAccZ - accelerometer.z : accelerometer.z - rawAccZ;  
  accelerometer.x = rawAccX;
  accelerometer.y = rawAccY;
  accelerometer.z = rawAccZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool PositionWitMotion::updateGyroscopeData(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ) {
  uint16_t xd = (rawGyroX > gyroscope.x) ? rawGyroX - gyroscope.x : gyroscope.x - rawGyroX;
  uint16_t yd = (rawGyroY > gyroscope.y) ? rawGyroY - gyroscope.y : gyroscope.y - rawGyroY;
  uint16_t zd = (rawGyroZ > gyroscope.z) ? rawGyroZ - gyroscope.z : gyroscope.z - rawGyroZ;  
  gyroscope.x = rawGyroX;
  gyroscope.y = rawGyroY;
  gyroscope.z = rawGyroZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool PositionWitMotion::updateHeightData(uint32_t height) {
  bool bHeight = (this->height != height);
  this->height = height;  
  return bHeight;
}

PositionWitMotion::PositionWitMotion(ArmPart* armPart, const uint memsRxPin, const uint memsTxPin, const uint memsRstPin, const uint memsIntPin) :
  memsRxPin(memsRxPin),
  memsTxPin(memsTxPin),
  memsRstPin(memsRstPin),
  memsIntPin(memsIntPin),
  armPart(armPart),
  imu(memsRxPin, memsTxPin, memsRstPin, memsIntPin)
{
  printf("PositionWitmotion constructor\n");  
  xTaskCreate(PositionWitMotion::compassTask, "Position::compassTask", 1024, this, 5, NULL);  
}