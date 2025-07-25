#pragma once

extern "C" {
  #include "../common/canBus/src/can2040.h" 
}

#include <RP2040.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
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

typedef void (* CanCallback)( void* armPart, can2040_msg frame );


typedef std::map<uint32_t, can2040_msg> CanMap;

class Bus {
  private:  
    const uint32_t pio_num = 0;
    const uint32_t sys_clock = 125000000;
    const uint32_t bitrate = 1000000;
    void* armPart;  
    CanCallback busCallback;
    static void busReceiveTask(void* instance);
    static void busSendTask(void *instance);
    static void PIOx_IRQHandler(void);
    static struct can2040 cbus;
    static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);  
    static CanMap canSendMap;
    static SemaphoreHandle_t sendMapSemaphore;
    static QueueHandle_t receiveQueue;
    static SemaphoreHandle_t receiveMapSemaphore;
    static can2040_msg receiveBuffer[CAN_RECEIVE_BUFFER_SIZE];
    static volatile size_t receiveHead;
    static volatile size_t receiveTail;
    static SemaphoreHandle_t receiveBufferMutex;
    static volatile uint32_t droppedFrames;
    static bool popReceivedFrame(can2040_msg &msg);
    static void pushReceivedFrameFromISR(const can2040_msg &msg);
    public:
      Bus(const uint rxPin, const uint txPin, void *instance, CanCallback callback);
      static uint32_t getDroppedFrames() { return droppedFrames; };
      void send(uint32_t id, uint64_t data);
    };