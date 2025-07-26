#include "rangeDetector.h"

bool RangeDetector::isEnabled = true;

RangeDetector::RangeDetector(ArmPart *armPart, i2c_inst_t *i2c,
                             const uint8_t longDetectorShutPin,
                             const uint8_t shortDetectorShutPin)
    : armPart(armPart),
      i2c(i2c),
      longDistanceDetector(),
      shortDistanceDetector(i2c, VL6180X_ADDRESS),
      longDetectorShutPin(longDetectorShutPin),
      shortDetectorShutPin(shortDetectorShutPin),
      useShortDistance(true),
      range(0)
{
  printf("Init detectors\n");
  gpio_init(shortDetectorShutPin);
  gpio_set_dir(shortDetectorShutPin, GPIO_OUT);
  gpio_init(longDetectorShutPin);
  gpio_set_dir(longDetectorShutPin, GPIO_OUT);

  activateSensor(true);

  xTaskCreate(RangeDetector::detectorTask, "RangeDetector::detectorTask", 1024, this, 5, NULL);
}

void RangeDetector::activateSensor(bool shortSensor)
{
  gpio_put(shortDetectorShutPin, 0);
  gpio_put(longDetectorShutPin, 0);
  vTaskDelay(pdMS_TO_TICKS(10));

  bool initSuccess = false;

  if (shortSensor){
    gpio_put(shortDetectorShutPin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (shortDistanceDetector.VL6180xInit() == 0) {
      shortDistanceDetector.changeAddress(VL618_DEFAULT_ADDR, VL618_NEW_ADDR);
      shortDistanceDetector.VL6180xDefautSettings();
      printf("Short distance sensor activated (0x%X)\n", VL618_NEW_ADDR);
      initSuccess = true;
    }
    else {
      printf("FAILED TO INITIALIZE SHORT DISTANCE SENSOR\n");
    }
  }
  else {
    gpio_put(longDetectorShutPin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (longDistanceDetector.begin(i2c))
    {
      longDistanceDetector.setAddress(VL53_NEW_ADDR);
      longDistanceDetector.startRangeContinuous();
      printf("Long distance sensor activated (0x%X)\n", VL53_NEW_ADDR);
      initSuccess = true;
    }
    else {
      printf("FAILED TO INITIALIZE LONG DISTANCE SENSOR\n");
    }
  }

  if (initSuccess) {
    useShortDistance = shortSensor;
  }
}

void RangeDetector::setupAddresses() {
  gpio_put(shortDetectorShutPin, 0);
  gpio_put(longDetectorShutPin, 0);
  vTaskDelay(pdMS_TO_TICKS(RANGE_SENSOR_STABILIZE_DELAY));

  gpio_put(shortDetectorShutPin, 1);
  vTaskDelay(pdMS_TO_TICKS(RANGE_SENSOR_STABILIZE_DELAY));
  shortDistanceDetector.VL6180xInit();
  shortDistanceDetector.changeAddress(VL618_DEFAULT_ADDR, VL618_NEW_ADDR);
  vTaskDelay(pdMS_TO_TICKS(RANGE_SENSOR_STABILIZE_DELAY));

  gpio_put(longDetectorShutPin, 1);
  vTaskDelay(pdMS_TO_TICKS(RANGE_SENSOR_STABILIZE_DELAY));
  longDistanceDetector.begin(i2c);
  longDistanceDetector.setAddress(VL53_NEW_ADDR);
  vTaskDelay(pdMS_TO_TICKS(RANGE_SENSOR_STABILIZE_DELAY));

  printf("I2C addresses set: VL6180X -> 0x%X, VL53L0X -> 0x%X\n", VL618_NEW_ADDR, VL53_NEW_ADDR);
}

void RangeDetector::printIdentification(struct VL6180xIdentification *temp)
{
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

void RangeDetector::initShortDistanceSensor() {
  VL6180xIdentification identification;
  shortDistanceDetector.getIdentification(&identification);
  printIdentification(&identification);

  if (shortDistanceDetector.VL6180xInit() != 0) {
    printf("FAILED TO INITALIZE VL6180X\n");
  }
  shortDistanceDetector.VL6180xDefautSettings();
}

void RangeDetector::initLongDistanceSensor() {
  if (longDistanceDetector.begin(i2c)) {
    longDistanceDetector.startRangeContinuous();
  }
  else {
    printf("FAILED TO INITIALIZE VL53L0X\n");
  }
}

void RangeDetector::detectorTask(void *instance) {
  RangeDetector *detector = (RangeDetector *)instance;
  while (true) {
    if (detector->useShortDistance) {
      detector->range = detector->shortDistanceDetector.getDistance();
    }
    else {
      if (detector->longDistanceDetector.isRangeComplete()) {
        detector->range = detector->longDistanceDetector.readRange();
      }
    }

    if (detector->useShortDistance && detector->range > RANGE_SHORT_TO_LONG_THRESHOLD) {
      detector->activateSensor(false);
    }
    else if (!detector->useShortDistance && detector->range < RANGE_LONG_TO_SHORT_THRESHOLD) {
      detector->activateSensor(true);
    }

    const int res = detector->armPart->updateRange(detector->range,
                                                   detector->useShortDistance ? 0 : 1);
    if (res == -1) {
      printf("Error sending range\n");
    }

    vTaskDelay(pdMS_TO_TICKS(RANGE_LOOP_TIMEOUT));
  }
}

bool RangeDetector::enabled(bool value) {
  isEnabled = value;
  if (!isEnabled) {
    gpio_put(shortDetectorShutPin, 0);
    gpio_put(longDetectorShutPin, 0);
    return false;
  } else {
    activateSensor(true);
    return true;
  }
}

void RangeDetector::scanI2cTask() {
  printf("\nScan started\n");

  switch (dCounter) {
  case 0:
    printf("%d VL6810X is enabled\n", dCounter);
    printf("VL53L0X is disabled\n");
    this->enabled(true);
    activateSensor(true);
    break;
  case 1:
    printf("%d VL6810X is disabled\n", dCounter);
    printf("VL53L0X is disabled\n");
    this->enabled(false);
    break;
  case 2:
    printf("%d VL6810X is disabled\n", dCounter);
    printf("VL53L0X is enabled\n");
    this->enabled(true);
    this->activateSensor(false);
    break;
  case 3:
    printf("%d VL6810X is disabled\n", dCounter);
    printf("VL53L0X is disabled\n");
    this->enabled(false);
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