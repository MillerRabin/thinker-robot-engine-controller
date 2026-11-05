#include "logQueue.h"

QueueHandle_t LogQueue::queue = NULL;
TaskHandle_t LogQueue::taskHandle = NULL;

void LogQueue::Init() {
  if (queue == NULL) {    
    queue = xQueueCreate(LOG_QUEUE_MESSAGES, LOG_QUEUE_MSG_SIZE);    
  }  
  if (taskHandle == NULL) {
    // Below the BNO and CAN tasks on this core (5-6): printing waits for USB, and at 6 a flooded or
    // stalled output kept the BNO unread for 795 ms (2026-11-06). A full queue drops messages instead.
    if (xTaskCreateAffinitySet(LogQueue::printTask, "LogQueue::printTask", 4096, NULL, 2, IMU_CORE, &taskHandle) == pdFAIL) {
      printf("Failed to create logQueue task\n");      
    } 
  }
}

void LogQueue::Log(const char *fmt, ...) {
  if (queue == NULL) {  
    return;
  }
  
  char buf[LOG_QUEUE_MSG_SIZE];  
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  BaseType_t result = xQueueSend(queue, buf, 0);
}

void LogQueue::printTask(void *instance) {  
  char buf[LOG_QUEUE_MSG_SIZE];
  while (true) {        
    if (xQueueReceive(queue, buf, portMAX_DELAY)) {      
      printf("%s", buf);
    }    
  }
}