#include "logQueue.h"

QueueHandle_t LogQueue::queue = NULL;
TaskHandle_t LogQueue::taskHandle = NULL;

void LogQueue::Init() {
  if (queue == NULL) {    
    queue = xQueueCreate(LOG_QUEUE_MESSAGES, LOG_QUEUE_MSG_SIZE);    
  }  
  if (taskHandle == NULL) {
    if (xTaskCreateAffinitySet(LogQueue::printTask, "LogQueue::printTask", 4096, NULL, 6, IMU_CORE, &taskHandle) == pdFAIL) {
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