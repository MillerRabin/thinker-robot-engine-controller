#include "positionBNO.h"

uint32_t Position::notificationIndex = 1;
TaskHandle_t Position::compassTaskHandle;


void Position::compassCallback(uint gpio, uint32_t events) {    
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(Position::compassTaskHandle, Position::notificationIndex, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Position::compassTask(void* instance) {  
  Position* position = (Position*)instance;
  BNO080 *imu = &(position->imu);
  uint32_t notificationValue;  
  imu->getReadings();  
  while (true) {
    position->armPart->setPositionTaskStatus(true);
    notificationValue = ulTaskNotifyTakeIndexed(notificationIndex, pdTRUE, pdMS_TO_TICKS(COMPASS_DATA_WAIT_TIMEOUT));
    if (notificationValue == 0) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    
    uint16_t datatype = imu->getReadings();
    if (datatype == SENSOR_REPORTID_ROTATION_VECTOR) {
      position->updateQuaternionData(imu->rawQuatI, imu->rawQuatJ, imu->rawQuatK, imu->rawQuatReal);      
      if (position->armPart->updateQuaternion(position) != 0) {
        printf("quat sending error\n");
      }      
    }

    if (datatype == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      position->updateQuaternionData(imu->rawQuatI, imu->rawQuatJ, imu->rawQuatK, imu->rawQuatReal);      
      if (position->armPart->updateQuaternion(position) != 0) {
        printf("quat sending error\n");
      }      
    }
    
    if (datatype == SENSOR_REPORTID_GYROSCOPE) {
      position->updateGyroscopeData(imu->rawGyroX, imu->rawGyroY, imu->rawGyroZ);      
      if (position->armPart->updateGyroscope(position) != 0) {
        printf("Gyro sending error\n");
      }    
    }

    if (datatype == SENSOR_REPORTID_LINEAR_ACCELERATION) {
      position->updateAccelerometerData(imu->rawLinAccelX, imu->rawLinAccelY, imu->rawLinAccelZ);
      if (position->armPart->updateAccelerometer(position) != 0) {
        printf("Accelerometer sending error\n");
      }
    }
          
    position->updateAccuracy(imu->rawQuatRadianAccuracy, imu->quatAccuracy, imu->gyroAccuracy, imu->accelLinAccuracy);
    if (position->armPart->updateAccuracy(position) != 0) {
      printf("Quaternion accuracy sending error\n");
    }
  }
}

bool Position::updateQuaternionData(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal) {
  uint16_t id = (rawQuatI > quaternion.rawI) ? rawQuatI - quaternion.rawI : quaternion.rawI - rawQuatI;
  uint16_t jd = (rawQuatJ > quaternion.rawJ) ? rawQuatJ - quaternion.rawJ : quaternion.rawJ - rawQuatJ;
  uint16_t kd = (rawQuatK > quaternion.rawK) ? rawQuatK - quaternion.rawK : quaternion.rawK - rawQuatK;
  uint16_t rd = (rawQuatReal > quaternion.rawReal) ? rawQuatReal - quaternion.rawReal : quaternion.rawReal - rawQuatReal;
  quaternion.rawI = rawQuatI;
  quaternion.rawJ = rawQuatJ;
  quaternion.rawK = rawQuatK;
  quaternion.rawReal = rawQuatReal;
  return ((id > 1) || (jd > 1) || (kd > 1) || (rd > 1));
}

bool Position::updateAccelerometerData(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ) {
  uint16_t xd = (rawAccX > accelerometer.x) ? rawAccX - accelerometer.x : accelerometer.x - rawAccX;
  uint16_t yd = (rawAccY > accelerometer.y) ? rawAccY - accelerometer.y : accelerometer.y - rawAccY;
  uint16_t zd = (rawAccZ > accelerometer.z) ? rawAccZ - accelerometer.z : accelerometer.z - rawAccZ;  
  accelerometer.x = rawAccX;
  accelerometer.y = rawAccY;
  accelerometer.z = rawAccZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool Position::updateGyroscopeData(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ) {
  uint16_t xd = (rawGyroX > gyroscope.x) ? rawGyroX - gyroscope.x : gyroscope.x - rawGyroX;
  uint16_t yd = (rawGyroY > gyroscope.y) ? rawGyroY - gyroscope.y : gyroscope.y - rawGyroY;
  uint16_t zd = (rawGyroZ > gyroscope.z) ? rawGyroZ - gyroscope.z : gyroscope.z - rawGyroZ;  
  gyroscope.x = rawGyroX;
  gyroscope.y = rawGyroY;
  gyroscope.z = rawGyroZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool Position::updateAccuracy(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy) {
  bool rd = (accuracy.quaternionRadAccuracy != quaternionRadianAccuracy);
  bool qd = (accuracy.quaternionAccuracy != quaternionAccuracy);
  bool gd = (accuracy.gyroscopeAccuracy != gyroscopeAccuracy);
  bool ad = (accuracy.accelerometerAccuracy != accelerometerAccuracy);
  accuracy.accelerometerAccuracy = accelerometerAccuracy;
  accuracy.quaternionAccuracy = quaternionAccuracy;
  accuracy.quaternionRadAccuracy = quaternionRadianAccuracy;  
  accuracy.gyroscopeAccuracy = gyroscopeAccuracy;
  return ad || rd || qd || gd;
}

Position::Position(ArmPart* armPart, const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin) :
  sdaPin(sdaPin),
  sclPin(sclPin),
  intPin(intPin),
  rstPin(rstPin),
  armPart(armPart)
{     
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(sdaPin, GPIO_FUNC_I2C);
  gpio_set_function(sclPin, GPIO_FUNC_I2C);
  gpio_pull_up(sdaPin);
  gpio_pull_up(sclPin);
  gpio_init(rstPin);
  gpio_set_dir(rstPin, GPIO_OUT);
  gpio_put(rstPin, 1);

  gpio_init(intPin);
  gpio_set_dir(intPin, GPIO_IN);
  gpio_pull_up(intPin);
  gpio_set_irq_enabled_with_callback(intPin, GPIO_IRQ_EDGE_FALL, true, &Position::compassCallback);
  
  xTaskCreate(Position::compassTask, "Position::compassTask", 1024, this, tskIDLE_PRIORITY, &Position::compassTaskHandle);  
  bi_decl(bi_2pins_with_func(sdaPin, sclPin, GPIO_FUNC_I2C));
    
  imu.begin(BNO080_DEFAULT_ADDRESS, i2c_default, intPin);  
  printf("Clearing tare\n");
  imu.clearTare();
  imu.calibrateAll();
  imu.enableRotationVector(50);  
  //imu.enableGameRotationVector(50);  
  imu.enableLinearAccelerometer(50);
  imu.enableGyro(50);
}