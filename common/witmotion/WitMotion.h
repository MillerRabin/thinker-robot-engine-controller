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

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

class WitMotion {
  private:
    const uint memsRxPin;
    const uint memsTxPin;
    static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t usMs);
    volatile QueueHandle_t queue = NULL;
    static void gpio_callback(uint gpio, uint32_t events);
    static volatile char s_cDataUpdate;
    static void writeDetectorData(void *pvParameters);
    static void readDetectorTask(void *pvParameters);
    static void AutoScanSensor(void);
    static uint32_t c_uiBaud[10];
  public:
    WitMotion(const uint memsRXPin, const uint memsTxPin, const uint memsRstPin, const uint memsIntPin);
};