#pragma once
#include "../config/config.h"
#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "queue.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

class LogQueue {
  private:
    static QueueHandle_t queue;
    static TaskHandle_t taskHandle;
    static void printTask(void *);
  public:  
    static void Init();  
    static void Log(const char *fmt, ...);
};