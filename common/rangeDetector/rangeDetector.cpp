#include "rangeDetector.h"

bool RangeDetector::isEnabled = true;

RangeDetector::RangeDetector(i2c_inst_t *i2c, const uint8_t longDetectorShutPin, const uint8_t shortDetectorShutPin) : 
  i2c(i2c),
  longDistanceDetector(),
  shortDistanceDetector(i2c, VL6180X_ADDRESS),
  longDetectorShutPin(longDetectorShutPin),
  shortDetectorShutPin(shortDetectorShutPin),
  current(longDistanceDetector) {
  printf("Init detectors\n");
  gpio_init(shortDetectorShutPin);
  gpio_set_dir(shortDetectorShutPin, GPIO_OUT);
  gpio_init(longDetectorShutPin);
  gpio_set_dir(longDetectorShutPin, GPIO_OUT);
  enabled(true);
  //switchToShortDistance(true);
  //current.begin(i2c);
  switchToShortDistance(false);  
  xTaskCreate(RangeDetector::detectorTask, "RangeDetector::detectorTask", 1024, this, 5, NULL);
}

void RangeDetector::printIdentification(struct VL6180xIdentification *temp){
  printf("Model ID = ");
  printf("%d\n",temp->idModel);

  printf("Model Rev = ");
  printf("%d",temp->idModelRevMajor);
  printf(".");
  printf("%d\n",temp->idModelRevMinor);

  printf("Module Rev = ");
  printf("%d",temp->idModuleRevMajor);
  printf(".");
  printf("%d\n",temp->idModuleRevMinor);  

  printf("Manufacture Date = ");
  printf("%d",((temp->idDate >> 3) & 0x001F));
  printf("/");
  printf("%d",((temp->idDate >> 8) & 0x000F));
  printf("/1");
  printf("%d\n",((temp->idDate >> 12) & 0x000F));
  printf(" Phase: ");
  printf("%d\n",(temp->idDate & 0x0007));

  printf("Manufacture Time (s)= ");
  printf("%d\n",(temp->idTime * 2));
  printf("\n\n");
}

void RangeDetector::initShortDistanceSensor() {
  VL6180xIdentification identification;
  shortDistanceDetector.getIdentification(&identification); // Retrieve manufacture info from device memory
  printIdentification(&identification); // Helper function to print all the Module information

  if(shortDistanceDetector.VL6180xInit() != 0){
    printf("FAILED TO INITALIZE\n"); //Initialize device and check for errors
  }; 

  shortDistanceDetector.VL6180xDefautSettings(); //Load default settings to get started.
}

void RangeDetector::detectorTask(void *instance) {  
  RangeDetector* detector = (RangeDetector*)instance;
  detector->initShortDistanceSensor();
  //detector->longDistanceDetector.begin(detector->i2c);
  //detector->longDistanceDetector.startRangeContinuous();
  while(true) {
      printf("Ambient Light Level (Lux) = ");
      //Input GAIN for light levels, 
      // GAIN_20     // Actual ALS Gain of 20
      // GAIN_10     // Actual ALS Gain of 10.32
      // GAIN_5      // Actual ALS Gain of 5.21
      // GAIN_2_5    // Actual ALS Gain of 2.60
      // GAIN_1_67   // Actual ALS Gain of 1.72
      // GAIN_1_25   // Actual ALS Gain of 1.28
      // GAIN_1      // Actual ALS Gain of 1.01
      // GAIN_40     // Actual ALS Gain of 40
  
      printf("%f\n", detector->shortDistanceDetector.getAmbientLight(GAIN_1) );

      //Get Distance and report in mm
      printf("Distance measured (mm) = ");
      printf("%d\n", detector->shortDistanceDetector.getDistance() ); 
    
    /*printf("Reading range\n");
    if (detector->longDistanceDetector.isRangeComplete()) {
      printf("Distance in mm: %d\n", detector->longDistanceDetector.readRange());
    }
    //detector->scanI2cTask();*/
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

bool RangeDetector::enabled(bool value) {
  isEnabled = value;
  if (!isEnabled) {
    gpio_put(shortDetectorShutPin, 0);
    gpio_put(longDetectorShutPin, 0);
    return false;
  } else {
    gpio_put(shortDetectorShutPin, 0);
    gpio_put(longDetectorShutPin, 1);
    return true;
  }
}

void RangeDetector::switchToShortDistance(bool useShortDistance) {  
  if (useShortDistance) {
    gpio_put(shortDetectorShutPin, isEnabled);
    gpio_put(longDetectorShutPin, 0);
    //this->current = this->shortDistanceDetector;
  } else {
    gpio_put(shortDetectorShutPin, 0);
    gpio_put(longDetectorShutPin, isEnabled);
    //this->current = this->longDistanceDetector;
  }
}

void RangeDetector::scanI2cTask() {
  printf("\nScan started\n");
  
  switch (dCounter)  {
    case 0:
      printf("%d VL6810X is enabled\n", dCounter);
      printf("VL53L0X is disabled\n");
      this->enabled(true);
      this->switchToShortDistance(true);
      break;
    case 1:
      printf("%d VL6810X is disabled\n", dCounter);
      printf("VL53L0X is disabled\n");
      this->enabled(false);
      this->switchToShortDistance(true);
      break;
    case 2:
      printf("%d VL6810X is disabled\n", dCounter);
      printf("VL53L0X is enabled\n");
      this->enabled(true);
      this->switchToShortDistance(false);
      break;
    case 3:
      printf("%d VL6810X is disabled\n", dCounter);
      printf("VL53L0X is disabled\n");
      this->enabled(false);
      this->switchToShortDistance(false);
      break;
  }

  dCounter = dCounter + 1;
  if (dCounter == 4) dCounter = 0;

  printf("\nI2C Bus Scan\n");
  printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

  for (int addr = 0; addr < (1 << 7); ++addr) {
    if (addr % 16 == 0) {
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