#include "rangeDetector.h"
#include "pico/stdio_usb.h"

TaskHandle_t RangeDetector::detectorTaskHandle = NULL;

RangeDetector::RangeDetector(ArmPart *armPart, i2c_inst_t *i2c,
                             const uint8_t longDetectorShutPin,
                             const uint8_t shortDetectorShutPin) : 
  armPart(armPart),
  i2c(i2c),
  longDistanceDetector(),
  shortDistanceDetector(i2c, VL6180X_ADDRESS),
  longDetectorShutPin(longDetectorShutPin),
  shortDetectorShutPin(shortDetectorShutPin)  
{
  gpio_init(shortDetectorShutPin);
  gpio_set_dir(shortDetectorShutPin, GPIO_OUT);
  gpio_init(longDetectorShutPin);
  gpio_set_dir(longDetectorShutPin, GPIO_OUT);

  if (RangeDetector::detectorTaskHandle == NULL) {
    if (xTaskCreateAffinitySet(RangeDetector::detectorTask,
                               "RangeDetector::detectorTask", 4096, this, 5,
                               IMU_CORE,
                               &RangeDetector::detectorTaskHandle) != pdPASS) {
      LogQueue::Log("RangeDetector::readTaskHandle creation failed\n");
    }
  }
}

void RangeDetector::activateSensors(bool shortSensor, bool longSensor) {
  shortSensorAvailable = false;
  longSensorAvailable = false;
  gpio_put(longDetectorShutPin, 0);
  gpio_put(shortDetectorShutPin, 0);
  // Hold both in shutdown long enough to actually reset them - a VL53L0X that skips the reset
  // keeps its moved address (0x2A) and the re-init below can't find it at 0x29.
  vTaskDelay(pdMS_TO_TICKS(RANGE_SHUTDOWN_DELAY));

  if (longSensor) {
    gpio_put(longDetectorShutPin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    // A failed VL53L0X still answers at the default 0x29, which is the VL6180X's address too -
    // shut it down again so it can't collide with the short sensor's init.
    if (!initLongDistanceSensor())
      gpio_put(longDetectorShutPin, 0);
  }

  if (shortSensor){
    gpio_put(shortDetectorShutPin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    initShortDistanceSensor();
  }
  LogQueue::Log("Range sensors: long %s, short %s\n", longSensorAvailable ? "OK" : "FAILED",
                shortSensorAvailable ? "OK" : "FAILED");
}

// Retries only the missing sensor where that's safe: the VL6180X alone can be reset while the
// VL53L0X sits at 0x2A, but a VL53L0X re-init needs 0x29 free, so it resets both.
void RangeDetector::retryMissingSensors() {
  if (!longSensorAvailable) {
    activateSensors(true, true);
    return;
  }
  gpio_put(shortDetectorShutPin, 0);
  vTaskDelay(pdMS_TO_TICKS(RANGE_SHUTDOWN_DELAY));
  gpio_put(shortDetectorShutPin, 1);
  vTaskDelay(pdMS_TO_TICKS(50));
  initShortDistanceSensor();
  LogQueue::Log("Range sensors: long OK, short %s\n", shortSensorAvailable ? "OK" : "FAILED");
}

// One-line list of the addresses that ACK on the detectors bus, with the given sensors enabled.
void RangeDetector::logBusScan(const char *stage, bool shortSensor, bool longSensor) {
  gpio_put(longDetectorShutPin, longSensor);
  gpio_put(shortDetectorShutPin, shortSensor);
  vTaskDelay(pdMS_TO_TICKS(50));
  char found[128] = "";
  size_t len = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    uint8_t rxdata;
    if (i2c_read_timeout_us(i2c, addr, &rxdata, 1, false, 2000) >= 0 && len < sizeof(found) - 6)
      len += snprintf(found + len, sizeof(found) - len, " 0x%02x", addr);
  }
  LogQueue::Log("Range I2C scan (%s):%s\n", stage, len ? found : " nothing");
}

void RangeDetector::printIdentification(struct VL6180xIdentification *temp) {
  printf("Model ID = %d\n", temp->idModel);
  printf("Model Rev = %d.%d\n", temp->idModelRevMajor, temp->idModelRevMinor);
  printf("Module Rev = %d.%d\n", temp->idModuleRevMajor, temp->idModuleRevMinor);
  printf("Manufacture Date = %d/%d/1%d Phase: %d\n",
         ((temp->idDate >> 3) & 0x001F),
         ((temp->idDate >> 8) & 0x000F),
         ((temp->idDate >> 12) & 0x000F),
         (temp->idDate & 0x0007));
  printf("Manufacture Time (s)= %d\n\n", (temp->idTime * 2));
}

bool RangeDetector::initShortDistanceSensor() {
  //VL6180xIdentification identification;
  //shortDistanceDetector.getIdentification(&identification);
  //printIdentification(&identification);

  const int res = shortDistanceDetector.VL6180xInit();
  if (res != 0) {
    LogQueue::Log("VL6180X init failed: %d (not fresh out of reset)\n", res);
    return false;
  }
  shortDistanceDetector.VL6180xDefautSettings();  
  shortSensorAvailable = true;
  return true;
}

bool RangeDetector::initLongDistanceSensor() {
  if (!longDistanceDetector.begin(i2c)) {
    LogQueue::Log("VL53L0X init failed: status %d\n", longDistanceDetector.Status);
    return false;
  }
  if (!longDistanceDetector.setAddress(VL53_NEW_ADDR)) {
    LogQueue::Log("VL53L0X address change failed: status %d\n", longDistanceDetector.Status);
    return false;
  }
  longDistanceDetector.startRangeContinuous();
  longSensorAvailable = true;
  return true;
}

void RangeDetector::detectorTask(void *instance) {
  
  RangeDetector *detector = (RangeDetector *)instance;
  detector->activateSensors(true, true);
  // The first attempt runs before USB is up, so its log is lost - and a single failure used to
  // leave the sensor NaN until reboot (e.g. the host enabling detector power after we booted).
  TickType_t lastInitTick = xTaskGetTickCount();
  bool scanned = false;
  while (true) {
    if ((!detector->longSensorAvailable || !detector->shortSensorAvailable) &&
        xTaskGetTickCount() - lastInitTick >= pdMS_TO_TICKS(RANGE_REINIT_INTERVAL)) {
      // Scanned once someone is reading the USB log - a scan right after boot is never seen.
      if (!scanned && stdio_usb_connected()) {
        detector->logBusScan("both off", false, false);
        detector->logBusScan("long only", false, true);
        detector->logBusScan("short only", true, false);
        scanned = true;
        detector->activateSensors(true, true);
      } else {
        detector->retryMissingSensors();
      }
      lastInitTick = xTaskGetTickCount();
    }

    //detector->scanI2cTask();
    if (detector->shortSensorAvailable) {
      detector->shortRange = detector->shortDistanceDetector.getDistance();
    } else {
      detector->shortRange = NAN;
    }

    if (detector->longSensorAvailable) {
      if (detector->longDistanceDetector.isRangeComplete()) {
        detector->longRange = detector->longDistanceDetector.readRange();
      }
    } else {
      detector->longRange = NAN;
    }
    
    //printf("Long range: %f mm, Short range %f mm\n", detector->longRange, detector->shortRange);

    const int res = detector->armPart->updateRange(detector->longRange, detector->shortRange);
    if (res == -1) {
      printf("Error sending range\n");
    }

    vTaskDelay(pdMS_TO_TICKS(RANGE_LOOP_TIMEOUT));
    //vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void RangeDetector::scanI2cTask() {
  printf("\nScan started\n");

  switch (dCounter) {
  case 0:
    printf("%d VL6810X is enabled\n", dCounter);
    printf("VL53L0X is disabled\n");    
    activateSensors(true, false);
    break;
  case 1:
    printf("%d VL6810X is disabled\n", dCounter);
    printf("VL53L0X is disabled\n");
    activateSensors(false, false);
    break;
  case 2:
    printf("%d VL6810X is disabled\n", dCounter);
    printf("VL53L0X is enabled\n");    
    this->activateSensors(false, true);
    break;
  case 3:
    printf("%d VL6810X is disabled\n", dCounter);
    printf("VL53L0X is disabled\n");
    this->activateSensors(false, false);
    break;
  }

  dCounter = dCounter + 1;
  if (dCounter == 4)
    dCounter = 0;

  printf("\nI2C Bus Scan\n");
  printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

  for (int addr = 0; addr < (1 << 7); ++addr)
  {
    if (addr % 16 == 0)
    {
      printf("%02x ", addr);
    }

    int ret;
    uint8_t rxdata;
    if (reserved_addr(addr))
      ret = PICO_ERROR_GENERIC;
    else
      ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);
    printf(ret < 0 ? "." : "@");
    printf(addr % 16 == 15 ? "\n" : "  ");
  }
}

bool RangeDetector::reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}