#include "localBNO.h"

uint32_t LocalBNO::notificationIndex = 1;
TaskHandle_t LocalBNO::compassTaskHandle;

// The game rotation vector can integrate the gyro about the wrong axis for a whole boot while the
// same chip's accelerometer stays right (shoulder, 2026-11-05, after a reflash): a 10deg turn about
// the joint (body y) read as 10deg about body x, and it snapped to y only ~0.5 s after the joint
// stopped. Twist about y barely moved, the correction kept driving, and the shoulder swept 0<->180
// on a still platform - the 2026-11-03 sweep too, most likely. The quaternion's gravity vs the
// accelerometer's: median 0.2-1.4deg in normal runs (platform tilts up to 140deg included); in
// that bad boot 14deg for 1.2 s on a 10deg move, 30-35deg for 2.7 s as the sweep began. 10deg for
// 0.5 s (first version) came close to tripping on a good wrist during platform tilts with the faster
// hold response: 20deg peak, 0.6 s above 10 - the end of the arm feels the shoulder's and elbow's
// accelerations most. 15deg for 1 s still stops the sweep in its first second or two.
constexpr float FUSION_MAX_ERROR_DEG = 15.0f;
constexpr TickType_t FUSION_MAX_ERROR_TIME = pdMS_TO_TICKS(1000);
constexpr float FUSION_FILTER_TAU_SEC = 0.1f;
// Readings off 1 g by more than this are a shock, not gravity. A sustained sideways acceleration
// can't fake 10deg for 0.5 s on this arm (that would take ~0.9 m/s of speed gained).
constexpr float FUSION_ACCEL_BAND = 1.5f; // m/s^2 around 9.81
constexpr TickType_t FUSION_SETTLE_TIME = pdMS_TO_TICKS(3000);
// Give up after this many resets each within the window of the previous one - a BNO a reset doesn't
// cure would otherwise recalibrate the arm forever.
constexpr int FUSION_MAX_RESETS = 3;
constexpr TickType_t FUSION_RESET_WINDOW = pdMS_TO_TICKS(60000);

// True when the quaternion has disagreed with the accelerometer for too long and the BNO should be
// reset - its new world frame invalidates the arm part's base, so the part recalibrates.
bool LocalBNO::fusionDiverged(const Accelerometer &acc, TickType_t now) {
  float dtSec = (float)(now - fusionLastCheck) / configTICK_RATE_HZ;
  fusionLastCheck = now;
  if ((int32_t)(now - fusionCheckFrom) < 0) {
    return false;
  }
  Quaternion q = quaternion.load();
  if (!q.isValid()) {
    return false;
  }
  Vector3 a = rotate.isValid() ? rotate.invert().rotate({acc.x, acc.y, acc.z}) : Vector3{acc.x, acc.y, acc.z};
  float aLen = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
  if (fabsf(aLen - 9.81f) > FUSION_ACCEL_BAND) {
    return false;
  }
  Vector3 g = q.getGravityVector();
  float gLen = sqrtf(g.x * g.x + g.y * g.y + g.z * g.z);
  float cosErr = std::clamp((g.x * a.x + g.y * a.y + g.z * a.z) / (gLen * aLen), -1.0f, 1.0f);
  float errorDeg = acosf(cosErr) * RAD_TO_DEG;
  fusionErrorDeg += (errorDeg - fusionErrorDeg) * std::min(dtSec / FUSION_FILTER_TAU_SEC, 1.0f);
  fusionErrorMaxDeg = std::max(fusionErrorMaxDeg, fusionErrorDeg);

  if (fusionErrorDeg <= FUSION_MAX_ERROR_DEG) {
    fusionBadSince = 0;
    return false;
  }
  if (fusionBadSince == 0) {
    fusionBadSince = now;
    return false;
  }
  if (now - fusionBadSince < FUSION_MAX_ERROR_TIME) {
    return false;
  }
  if (fusionResets > 0 && now - fusionLastReset > FUSION_RESET_WINDOW) {
    fusionResets = 0;
  }
  if (fusionResets >= FUSION_MAX_RESETS) {
    static Periodic giveUpPrinter(pdMS_TO_TICKS(10000));
    giveUpPrinter.interval([&]() {
      LogQueue::Log("BNO fusion still %.1fdeg off its accelerometer after %d resets - not resetting again\n",
                    fusionErrorDeg, fusionResets);
    });
    return false;
  }
  fusionResets++;
  fusionLastReset = now;
  LogQueue::Log("BNO fusion %.1fdeg off its accelerometer for %lu ms - resetting the BNO (%d of %d)\n",
                fusionErrorDeg, (unsigned long)((now - fusionBadSince) * portTICK_PERIOD_MS), fusionResets,
                FUSION_MAX_RESETS);
  return true;
}

void LocalBNO::tare(uint8_t axisMask) {
  imu.tare(axisMask, 0);
}

void LocalBNO::compassTask(void *instance) {
  LocalBNO *bno = (LocalBNO *)instance;
  bool needCalibration = false;
  bool needSaveCalibration = false;
  TickType_t lastCalibrationCheck = 0;
  TickType_t lTime = xTaskGetTickCount();
  BNO080 *imu = &(bno->imu);
  const TickType_t calibrationCheckInterval = pdMS_TO_TICKS(3000);
  // Read-rate diagnosis (2026-11-03): each pass reads one report, while game rotation vector and
  // accelerometer each arrive every getRefreshInterval() ms - if the reads fall behind, the hub's
  // queue grows and every angle comes late. Logged every 5 s: reports actually read, empty polls.
  uint32_t grvReports = 0, accReports = 0, otherReports = 0, emptyPolls = 0;
  TickType_t lastRateLog = xTaskGetTickCount();
  bno->begin();  
  while (true) {            
    if (imu->hasReset()) {
      printf("IMU was reset. Re-enabling sensor...\n");
      needCalibration = false;
      bno->initIMU();
      bno->armPart->onIMUReset();
    }

    bool isAlive = bno->isPositionOK();
    bno->armPart->setPositionStatus(isAlive);
    if (!isAlive) {
      printf("Restarting imu\n");
      auto res = bno->begin();
      needCalibration = false;
      vTaskDelay(pdMS_TO_TICKS(IMU_UPDATE_INTERVAL));
      continue;
    }
        
    if (xTaskGetTickCount() - lastRateLog >= pdMS_TO_TICKS(5000)) {
      LogQueue::Log("BNO reads 5s: grv %lu acc %lu other %lu empty %lu, fusion error max %.1fdeg\n",
                    (unsigned long)grvReports, (unsigned long)accReports, (unsigned long)otherReports,
                    (unsigned long)emptyPolls, bno->fusionErrorMaxDeg);
      grvReports = accReports = otherReports = emptyPolls = 0;
      bno->fusionErrorMaxDeg = 0.0f;
      lastRateLog = xTaskGetTickCount();
    }
    uint16_t datatype = imu->getReadings();
    if (datatype == 0) {
      emptyPolls++;
      vTaskDelay(1);
      continue;
    }
    TickType_t now = xTaskGetTickCount();
    bool resetFusion = false;
    if (datatype == SENSOR_REPORTID_GAME_ROTATION_VECTOR || datatype == SENSOR_REPORTID_ROTATION_VECTOR) {
      grvReports++;
    } else if (datatype == SENSOR_REPORTID_ACCELEROMETER) {
      accReports++;
    } else {
      otherReports++;
    }

    switch (datatype) {
      case SENSOR_REPORTID_ROTATION_VECTOR:
      case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {                
        Quaternion quat;
        quat.fromBNO(imu->rawQuatI, imu->rawQuatJ, imu->rawQuatK, imu->rawQuatReal);        
        Quaternion qr = (bno->rotate.isValid()) ? quat * bno->rotate : quat;
        bno->quaternion.store(qr);
        bno->armPart->updateQuaternion(bno);
        break;
      }
      case SENSOR_REPORTID_GYROSCOPE: {
        bno->gyroscope.fromBNO(imu->rawGyroX, imu->rawGyroY, imu->rawGyroZ);
        bno->armPart->updateGyroscope(bno);        
        break;
      }
      case SENSOR_REPORTID_LINEAR_ACCELERATION: {
        Accelerometer acc;
        acc.fromBNO(imu->rawLinAccelX, imu->rawLinAccelY, imu->rawLinAccelZ);
        bno->accelerometer.store(acc);
        bno->armPart->updateAccelerometer(bno);            
        break;
      }
      case SENSOR_REPORTID_ACCELEROMETER: {
        Accelerometer acc;
        acc.fromBNO(imu->rawAccelX, imu->rawAccelY, imu->rawAccelZ);
        bno->accelerometer.store(acc);
        bno->armPart->updateAccelerometer(bno);
        resetFusion = bno->fusionDiverged(acc, now);
        break;
      }
      default:
        break;
    }

    if (resetFusion) {
      // begin() pulses the RST pin (SPI) or soft-resets (I2C) and re-enables the reports.
      bno->begin();
      needCalibration = false;
      bno->armPart->onIMUReset();
      continue;
    }

    bno->updateAccuracy(imu->rawQuatRadianAccuracy, imu->getQuatAccuracy(), imu->gyroAccuracy, imu->accelLinAccuracy);
    bno->armPart->updateAccuracy(bno);
    
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
    // No pacing delay after a report: the next one may be queued already. Pacing at one report per
    // IMU_UPDATE_INTERVAL read 1000 of the 2000 reports the two 5 ms streams produce in 5 s (shoulder
    // and elbow alike, 0-2 empty polls), so the hub's queue sat full and every angle came late
    // (2026-11-03). An empty poll still yields with vTaskDelay(1) above.
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

uint LocalBNO::getRefreshInterval() {
  return (useSPI) ? 5 : 10;
}

void LocalBNO::initIMU() {    
  uint rr = getRefreshInterval();
  imu.enableGameRotationVector(rr);
  // imu.enableRotationVector(IMU_UPDATE_INTERVAL);
  //imu.enableLinearAccelerometer(10);
  imu.enableAccelerometer(rr);
  // imu.enableGyro(5);
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
  useSPI(false) {
    if (LocalBNO::compassTaskHandle == NULL) {
      if (xTaskCreateAffinitySet(LocalBNO::compassTask, "LocalBNO::compassTask",
                                4096, this, 5, IMU_CORE,
                                &LocalBNO::compassTaskHandle) != pdPASS) {
        printf("LocalBNO::compassTask creation failed\n");
      }
    }
}

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
  useSPI(true) {
    if (LocalBNO::compassTaskHandle == NULL) {
      if (xTaskCreateAffinitySet(LocalBNO::compassTask, "LocalBNO::compassTask",
                                 4096, this, 5, IMU_CORE,
                                 &LocalBNO::compassTaskHandle) != pdPASS) {
        printf("LocalBNO::compassTask creation failed\n");        
      }
    }
  }


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
  quaternion.makeFresh();
  fusionCheckFrom = xTaskGetTickCount() + FUSION_SETTLE_TIME;
  fusionErrorDeg = 0.0f;
  fusionBadSince = 0;
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
    
  return imu.beginSPI(spi0, csPin, intPin, rstPin);
}

bool LocalBNO::isPositionOK() {
  return quaternion.isFresh();
}