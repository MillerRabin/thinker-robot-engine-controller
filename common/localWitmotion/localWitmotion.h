#pragma once

#include <cstdint>
#include <iostream>
#include <cstring>
#include <math.h>
#include "hardware/uart.h"
#include "../config/config.h"
#include "../imuBase/imuBase.h"
#include "../armPart/armPart.h"
#include "../witmotion/wit_c_sdk.h"

#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>


class LocalWitmotion : public IMUBase {
private:
  static LocalWitmotion* instance;
  const uint memsRxPin;
  const uint memsTxPin;
  static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
  static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
  static void Delayms(uint16_t usMs);
  static void gpio_callback(uint gpio, uint32_t events);
  static void init(void *pvParameters);
  static void readDetectorTask(void *pvParameters);
  static uint32_t AutoScanSensor(void);
  static volatile bool dataAvailable;
  static uint32_t c_uiBaud[10];
  static volatile uint8_t uartRxBuffer[512];
  static volatile uint16_t uartHead;
  static volatile uint16_t uartTail;
  static void Usart1Init(uint32_t baud_rate);
  static void on_uart_rx();
  static SemaphoreHandle_t uartRxSemaphore;
public:
  ArmPart *armPart;
  static SemaphoreHandle_t dataReadySemaphore;
  LocalWitmotion(ArmPart *armPart, const uint memsRXPin, const uint memsTxPin, const uint memsRstPin);
};