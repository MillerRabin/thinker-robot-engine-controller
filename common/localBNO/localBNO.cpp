#include "localBNO.h"

uint32_t LocalBNO::notificationIndex = 1;
TaskHandle_t LocalBNO::compassTaskHandle;

void LocalBNO::compassCallback(uint gpio, uint32_t events) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(LocalBNO::compassTaskHandle, LocalBNO::notificationIndex, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LocalBNO::tare(uint8_t axisMask) {
  imu.tare(axisMask, 0);
}

void LocalBNO::compassTask(void *instance) {  
  LocalBNO *bno = (LocalBNO *)instance;
  bool needCalibration = false;
  bool needSaveCalibration = false;
  TickType_t lastCalibrationCheck = 0;  
  BNO080 *imu = &(bno->imu);
  const TickType_t calibrationCheckInterval = pdMS_TO_TICKS(3000);  
  TickType_t lastSendTime = 0;
  uint noDataCount = 0;

  while (true) {
    if (imu->hasReset()) {
      printf("IMU was reset. Re-enabling sensor...\n");
      needCalibration = false;
      bno->initIMU();
    }
    
    uint32_t notificationValue = ulTaskNotifyTakeIndexed(notificationIndex, pdTRUE, pdMS_TO_TICKS(IMU_WAIT_TIMEOUT));
    if (notificationValue == 0) {            
      bno->armPart->setPositionStatus(false);
      noDataCount++;
      if (noDataCount >= 5) {
        noDataCount = 0;
        printf("Restarting imu\n");
        auto res = bno->begin();
        needCalibration = false;
        if (res) {
          printf("IMU initialized\n");
        } else {
          printf("IMU initialization failed\n");
        }
      }
      vTaskDelay(pdMS_TO_TICKS(IMU_NO_DATA_TIMEOUT));
      continue;
    }        
    
    bno->armPart->setPositionStatus(true);

    uint16_t datatype;
    
    datatype = imu->getReadings();    
    noDataCount = 0;
    TickType_t now = xTaskGetTickCount();
    lastSendTime = now;

    switch (datatype) {
      case SENSOR_REPORTID_ROTATION_VECTOR:
      case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {        
        bno->quaternion.fromBNO(imu->rawQuatI, imu->rawQuatJ, imu->rawQuatK, imu->rawQuatReal);        
        if (bno->armPart->updateQuaternion(bno) != 0) {
          printf("quat sending error\n");
        }
        break;
      }
      case SENSOR_REPORTID_GYROSCOPE: {
        bno->gyroscope.fromBNO(imu->rawGyroX, imu->rawGyroY, imu->rawGyroZ);
        if (bno->armPart->updateGyroscope(bno) != 0) {
          printf("Gyro sending error\n");
        }
        break;
      }
      case SENSOR_REPORTID_LINEAR_ACCELERATION: {
        bno->accelerometer.fromBNO(imu->rawLinAccelX, imu->rawLinAccelY, imu->rawLinAccelZ);
        if (bno->armPart->updateAccelerometer(bno) != 0) {
          printf("Accelerometer sending error\n");
        }
        break;
      }
      case SENSOR_REPORTID_ACCELEROMETER: {
        bno->accelerometer.fromBNO(imu->rawAccelX, imu->rawAccelY, imu->rawAccelZ);        
        if (bno->armPart->updateAccelerometer(bno) != 0) {
          printf("Accelerometer sending error\n");
        }
        break;
      }
      default:
        break;
    }

    bno->updateAccuracy(imu->rawQuatRadianAccuracy, imu->getQuatAccuracy(), imu->gyroAccuracy, imu->accelLinAccuracy);
    if (bno->armPart->updateAccuracy(bno) != 0) {
      printf("Quaternion accuracy sending error\n");
    }
    
    if ((now - lastCalibrationCheck) >= calibrationCheckInterval) {
      lastCalibrationCheck = now;
      uint8_t quatAcc = bno->accuracy.quaternionAccuracy;      
      if (!needCalibration && quatAcc < 3) {
        needCalibration = true;
        imu->calibrateAll();
        printf("BNO needs calibration. Starting...\n");
      }

      if (needSaveCalibration && quatAcc == 3) {        
        imu->saveCalibration();
        printf("BNO saving calibration\n");
        needSaveCalibration = false;
      }

      if (needCalibration && quatAcc == 3) {
        imu->endCalibration();                
        printf("BNO is calibrated\n");
        needCalibration = false;
        needSaveCalibration = true;
      }
    }
  }
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
  imu.enableRotationVector(10);  
  imu.enableLinearAccelerometer(10);
  imu.enableGyro(5);
}

uint LocalBNO::writeSystemOrientationQuaternion(float w, float x, float y, float z) {
  hardReset();  
  auto res = imu.writeSystemOrientationQuaternion(w, x, y, z);
  printf("Write System Orientation Quaternion result: %d\n", res);
  initIMU();
  return res;
}

void LocalBNO::hardReset() {
  gpio_put(rstPin, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_put(rstPin, 1);
  vTaskDelay(pdMS_TO_TICKS(100));
}

LocalBNO::LocalBNO(ArmPart *armPart, const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin) : 
  sdaPin(sdaPin),
  sclPin(sclPin),
  intPin(intPin),
  rstPin(rstPin),
  csPin(0xFF),
  sckPin(0xFF),
  misoPin(0xFF),
  mosiPin(0xFF),
  armPart(armPart),
  useSPI(false) {}

LocalBNO::LocalBNO(ArmPart *armPart, const uint sckPin, const uint misoPin, const uint mosiPin, const uint csPin, const uint rstPin, const uint intPin) : 
  sckPin(sckPin),
  misoPin(misoPin),
  mosiPin(mosiPin),
  rstPin(rstPin),
  intPin(intPin),
  csPin(csPin),
  sdaPin(0xFF),
  sclPin(0xFF),
  armPart(armPart),
  useSPI(true) {}


bool LocalBNO::begin() {
  if (useSPI) {
    auto spires = beginSPI();
    if (!spires) {
      printf("Failed to initialize LocalBNO IMU in SPI mode\n");
      //return false;
    }
  } else {
    auto i2cres = beginI2C();
    if (!i2cres) {
      printf("Failed to initialize LocalBNO IMU in I2C mode\n");
      //return false;
    }
  }    
  initIMU();  
  if (LocalBNO::compassTaskHandle == NULL) {
    if (xTaskCreate(LocalBNO::compassTask, "LocalBNO::compassTask", 4096, this, 6, &LocalBNO::compassTaskHandle) != pdPASS) {
      printf("LocalBNO::compassTask creation failed\n");
      return false;
    }
  }
  return true;
} 

bool LocalBNO::beginI2C() {
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
  bi_decl(bi_2pins_with_func(sdaPin, sclPin, GPIO_FUNC_I2C));
  bi_decl(bi_2pins_with_func(intPin, rstPin, GPIO_FUNC_SIO));
  return imu.begin(BNO080_DEFAULT_ADDRESS, i2c_default, intPin); 
}

bool LocalBNO::beginSPI() {
  spi_init(spi0, 3 * 1000 * 1000);
  spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

  gpio_set_function(sckPin, GPIO_FUNC_SPI);
  gpio_set_function(mosiPin, GPIO_FUNC_SPI);
  gpio_set_function(misoPin, GPIO_FUNC_SPI);
  
  gpio_init(csPin);
  gpio_set_dir(csPin, GPIO_OUT);
  gpio_put(csPin, 1);

  gpio_init(rstPin);
  gpio_set_dir(rstPin, GPIO_OUT);
    
  gpio_init(intPin);
  gpio_set_dir(intPin, GPIO_IN);
  gpio_pull_up(intPin);

  bi_decl(bi_3pins_with_func(misoPin, mosiPin, sckPin, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_func(csPin, GPIO_FUNC_SIO));
  bi_decl(bi_2pins_with_func(intPin, rstPin, GPIO_FUNC_SIO));
  gpio_set_irq_enabled_with_callback(intPin, GPIO_IRQ_EDGE_FALL, true, &LocalBNO::compassCallback);
  return imu.beginSPI(spi0, csPin, intPin, rstPin);
}