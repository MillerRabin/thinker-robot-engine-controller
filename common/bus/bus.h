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

class Bus
{
private:
  const uint32_t pio_num = 0;
  const uint32_t sys_clock = 125000000;
  const uint32_t bitrate = 1000000;
  void* armPart;
  CanCallback busCallback;
  static void busTask(void* instance); 
  static void PIOx_IRQHandler(void);
  static volatile QueueHandle_t queue;
  static struct can2040 cbus;
  static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);
public:    
  Bus(const uint rxPin, const uint txPin, void* instance, CanCallback callback);
  int send(uint32_t id, uint64_t data);
};