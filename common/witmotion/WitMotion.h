#pragma once

#include <cstdint>
#include <iostream>
#include <cstring>
#include <math.h>
#include "hardware/uart.h"
#include "../config/config.h"
#include "wit_c_sdk.h"

#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

class WitMotion {
  private:
    const uint memsRxPin;
    const uint memsTxPin;
    static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t usMs);
    static void gpio_callback(uint gpio, uint32_t events);
    static void init(void *pvParameters);    
    static void readDetectorTask(void *pvParameters);
    static void AutoScanSensor(void);
    static volatile bool dataAvailable;
    static uint32_t c_uiBaud[10];
  public:
    static volatile uint16_t rawLinAccelX;
    static volatile uint16_t rawLinAccelY;
    static volatile uint16_t rawLinAccelZ; 
    static volatile uint16_t rawGyroX;
    static volatile uint16_t rawGyroY;
    static volatile uint16_t rawGyroZ;
    static volatile uint16_t rawRoll;
    static volatile uint16_t rawPitch;
    static volatile uint16_t rawYaw;
    static volatile uint32_t rawHeight;
    static volatile uint16_t rawQuatI;
    static volatile uint16_t rawQuatJ;
    static volatile uint16_t rawQuatK;
    static volatile uint16_t rawQuatReal;
    static float getAccValue(uint16_t rawValue) { return rawValue / 32768.0f * 16.0f; }
    static float getGyroValue(uint16_t rawValue) { return rawValue / 32768.0f * 2000.0f; }
    static float getEulerValue(uint16_t rawValue) { return rawValue / 32768.0f * 180.0f; }
    static float getQuaternionValue(uint16_t rawValue) { return rawValue / 32768.0f; }
    WitMotion(const uint memsRXPin, const uint memsTxPin, const uint memsRstPin, const uint memsIntPin);
};