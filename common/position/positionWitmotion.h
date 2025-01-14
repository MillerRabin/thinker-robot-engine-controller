#pragma once

#include "../witmotion/WitMotion.h"
#include <iostream>
#include <hardware/i2c.h>
#include "../armPart/armPart.h"
#include <RP2040.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/binary_info.h"

class PositionWitMotion : public BasePosition
{
  private:    
    const uint memsRxPin;
    const uint memsTxPin;
    const uint memsRstPin;
    const uint memsIntPin;
    static void compassTask(void* instance);
    static void compassCallback(uint gpio, uint32_t events);    
    static uint32_t notificationIndex;
    static TaskHandle_t compassTaskHandle;
  public:
    ArmPart* armPart;
    WitMotion imu;
    bool updateQuaternionData(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
    bool updateAccelerometerData(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ);
    bool updateGyroscopeData(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ);
    bool updateHeightData(uint32_t height);
    PositionWitMotion(ArmPart* armPart, const uint memsRxPin, const uint memsTxPin, const uint memsRstPin, const uint memsIntPin);
};