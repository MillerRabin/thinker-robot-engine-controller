#pragma once

#include "../BNO080/BNO080.h"
#include <iostream>
#include <hardware/i2c.h>
#include "../armPart/armPart.h"
#include <RP2040.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/binary_info.h"

class Position : public BasePosition
{
  private:    
    const uint sdaPin;
    const uint sclPin;
    const uint intPin;
    const uint rstPin;
    static void compassTask(void* instance);
    static void compassCallback(uint gpio, uint32_t events);    
    static uint notificationIndex;
    static TaskHandle_t compassTaskHandle;
  public:
    ArmPart* armPart;
    BNO080 imu;    
    int16_t accelerometer_Q1;
    bool updateQuaternionData(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
    bool updateAccelerometerData(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ);
    bool updateGyroscopeData(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ);
    bool updateAccuracy(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy);
    Position(ArmPart* armPart, const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin);
};