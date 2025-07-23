#include "localBNO.h"

uint32_t LocalBNO::notificationIndex = 1;
TaskHandle_t LocalBNO::compassTaskHandle;
Quaternion LocalBNO::lastQuaternion;

float rollOffset = 0.9f * (M_PI / 180.0f);
float pitchOffset = -1.680f * (M_PI / 180.0f);

Quaternion errorQuat = Quaternion::FromEuler(rollOffset, pitchOffset, 0.0f);
Quaternion correctionQuat = Quaternion::Conjugate(errorQuat);
Quaternion LocalBNO::rotateQuatenion = Quaternion::Multiply(correctionQuat, {-0.7071f, 0.7071f, 0.0f, 0.0f});

void LocalBNO::compassCallback(uint gpio, uint32_t events) {    
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(LocalBNO::compassTaskHandle, LocalBNO::notificationIndex, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LocalBNO::compassTask(void* instance) {  
  LocalBNO* bno = (LocalBNO*)instance;
  bool needCalibration = false;
  BNO080* imu = &(bno->imu);
  uint32_t notificationValue;  
  imu->getReadings();  
  while (true) {
    if (imu->hasReset()){
      printf("IMU was reset. Re-enabling sensor...\n");
      bno->initIMU();
    }
    bno->armPart->setPositionTaskStatus(true);
    notificationValue = ulTaskNotifyTakeIndexed(notificationIndex, pdTRUE, pdMS_TO_TICKS(COMPASS_DATA_WAIT_TIMEOUT));
    if (notificationValue == 0) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    
    uint16_t datatype = imu->getReadings();
    if ((datatype == SENSOR_REPORTID_ROTATION_VECTOR) ||
        (datatype == SENSOR_REPORTID_GAME_ROTATION_VECTOR)) {
      bno->quaternion.fromBNO(imu->rawQuatI, imu->rawQuatJ, imu->rawQuatK, imu->rawQuatReal);
      bno->quaternion.multiplyFirst(rotateQuatenion);
      if (bno->armPart->updateQuaternion(bno) != 0) {
        printf("quat sending error\n");
      }      
    }
  
    if (datatype == SENSOR_REPORTID_GYROSCOPE) {
      bno->updateGyroscopeData(imu->rawGyroX, imu->rawGyroY, imu->rawGyroZ);      
      if (bno->armPart->updateGyroscope(bno) != 0) {
        printf("Gyro sending error\n");
      }    
    }

    if (datatype == SENSOR_REPORTID_LINEAR_ACCELERATION) {
      bno->updateAccelerometerData(imu->rawLinAccelX, imu->rawLinAccelY, imu->rawLinAccelZ);
      if (bno->armPart->updateAccelerometer(bno) != 0) {
        printf("Accelerometer sending error\n");
      }
    }
          
    bno->updateAccuracy(imu->rawQuatRadianAccuracy, imu->quatAccuracy, imu->gyroAccuracy, imu->accelLinAccuracy);
    if (bno->armPart->updateAccuracy(bno) != 0) {
      printf("Quaternion accuracy sending error\n");
    }

    if (!needCalibration && imu->quatAccuracy < 3) {
      needCalibration = true;
      imu->calibrateAll();
      printf("BNO needs calibration. Starting...\n");
    }

    if (needCalibration && imu->getQuatAccuracy() == 3) {
      imu->endCalibration();
      vTaskDelay(pdMS_TO_TICKS(200));
      imu->saveCalibration();
      printf("BNO is calibrated. Saving...\n");
      needCalibration = false;
    }
  }
}

bool LocalBNO::updateAccelerometerData(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ) {
  uint16_t xd = (rawAccX > accelerometer.x) ? rawAccX - accelerometer.x : accelerometer.x - rawAccX;
  uint16_t yd = (rawAccY > accelerometer.y) ? rawAccY - accelerometer.y : accelerometer.y - rawAccY;
  uint16_t zd = (rawAccZ > accelerometer.z) ? rawAccZ - accelerometer.z : accelerometer.z - rawAccZ;  
  accelerometer.x = rawAccX;
  accelerometer.y = rawAccY;
  accelerometer.z = rawAccZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool LocalBNO::updateGyroscopeData(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ) {
  uint16_t xd = (rawGyroX > gyroscope.x) ? rawGyroX - gyroscope.x : gyroscope.x - rawGyroX;
  uint16_t yd = (rawGyroY > gyroscope.y) ? rawGyroY - gyroscope.y : gyroscope.y - rawGyroY;
  uint16_t zd = (rawGyroZ > gyroscope.z) ? rawGyroZ - gyroscope.z : gyroscope.z - rawGyroZ;  
  gyroscope.x = rawGyroX;
  gyroscope.y = rawGyroY;
  gyroscope.z = rawGyroZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool LocalBNO::updateAccuracy(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy) {
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

void LocalBNO::initIMU() {
  imu.enableRotationVector(50);
  imu.enableLinearAccelerometer(50);
  imu.enableGyro(50);
}

LocalBNO::LocalBNO(ArmPart* armPart, const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin) :
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
  gpio_set_irq_enabled_with_callback(intPin, GPIO_IRQ_EDGE_FALL, true, &LocalBNO::compassCallback);
  
  xTaskCreate(LocalBNO::compassTask, "LocalBNO::compassTask", 1024, this, tskIDLE_PRIORITY, &LocalBNO::compassTaskHandle);  
  bi_decl(bi_2pins_with_func(sdaPin, sclPin, GPIO_FUNC_I2C));
    
  imu.begin(BNO080_DEFAULT_ADDRESS, i2c_default, intPin);  
  initIMU();
}