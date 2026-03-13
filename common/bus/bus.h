#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <event_groups.h>

// CXX
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
// Pico SDK
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "../config/config.h" 
#include "../canRingBuffer/canRingBuffer.h"

typedef void (*CanCallback)(void *armPart, CanFrame frame);

typedef enum : int32_t {
  CAN_SUCCESS = 0,
  CAN_EVENT_GROUP_CREATION_FAILED = -1,
  CAN_TASK_RECEIVE_CREATION_FAILED = -2,
  CAN_TASK_SEND_CREATION_FAILED = -3,
  CAN_TASK_WATCHDOG_CREATION_FAILED = -4,
  CAN_TASK_INIT_CREATION_FAILED = -5,
  CAN_SEND_MUTEX_CREATION_FAILED = -6
} can_status_t;

class ArmPart;

class Bus {
  private:  
    const uint32_t pio_num = 0;
    const uint32_t sys_clock = 125000000;
    const uint32_t bitrate = 1000000;
    const uint rxPin;
    const uint txPin;
    uint32_t lastRxTime = 0;
    ArmPart* armPart;  
    CanCallback busCallback;
    static void busReceiveTask(void* instance);
    static void busSendTask(void *instance);
    static void busWatchdogTask(void *instance);
    static void initTask(void *instance);
    static void PIOx_IRQHandler(void);
    static void* instance;
    static can2040 cbus;
    static void can2040_cb(can2040 *cd, uint32_t notify, CanFrame *msg);
    CanRingBuffer receiveBuffer;
    CanFrame sendBuffer[CAN_TX_BUFFER_SIZE];
    EventGroupHandle_t events = NULL;
    TaskHandle_t sendTaskHandle = NULL;
    TaskHandle_t receiveTaskHandle = NULL;
    TaskHandle_t watchdogTaskHandle = NULL;
    TaskHandle_t initTaskHandle = NULL;
    SemaphoreHandle_t sendMutex = NULL;
    bool reinitInProgress = false;
    void requestReinit();
  public:
    bool isBusAlive();
    Bus(const uint rxPin, const uint txPin, ArmPart* armPart, CanCallback callback);
    can_status_t begin();
    can_status_t init();
    bool send(uint32_t id, uint64_t data);
};