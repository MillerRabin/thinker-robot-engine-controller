#pragma once

#include <cstdint>
#include <iostream>
#include <cstring>
#include <math.h>
#include "hardware/i2c.h"
#include "../config/config.h"
#include "../VL53L0X/Pico_VL53L0X.h"
#include "../VL6180x/VL6180x.h"
#include "../armPart/armPart.h"

#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#define VL6180X_ADDRESS 0x29

class RangeDetector {
  private:
    static bool isEnabled;
    const uint8_t longDetectorShutPin;
    const uint8_t shortDetectorShutPin;
    Pico_VL53L0X longDistanceDetector;
    VL6180x shortDistanceDetector;
    static void detectorTask(void *instance); 
    static bool reserved_addr(uint8_t addr);        
    uint dCounter = 0;
    void printIdentification(struct VL6180xIdentification *temp);
    void initShortDistanceSensor();
    void initLongDistanceSensor();
    void setupAddresses();
    void activateSensor(bool shortSensor);
  public:
    ArmPart *armPart;
    bool enabled(bool value);
    bool useShortDistance;
    i2c_inst_t *i2c;
    void scanI2cTask();
    uint16_t range = 0;
    RangeDetector(ArmPart * armPart, i2c_inst_t * i2c, const uint8_t longDetectorShutPin, const uint8_t shortDetectorShutPin);
};