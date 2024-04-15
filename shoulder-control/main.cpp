#include "main.h"

ArmShoulder* armShoulder;
volatile QueueHandle_t watcherQueue;

static void watcher(void *unused) {
  while(true) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    //uint8_t params;
    //xQueueReceive(watcherQueue, &params, portMAX_DELAY);    
  }  
}

int main() {	
  stdio_init_all();  
  sleep_ms(10000);
  printf("Started\n");
  watcherQueue = xQueueCreate(1, 1);
  xTaskCreate(watcher, "Watcher", 128, NULL, tskIDLE_PRIORITY, NULL);
  armShoulder = new ArmShoulder(SDA_PIN, SCL_PIN, INT_PIN, RST_PIN, SHOULDER_Z_PIN, SHOULDER_Y_PIN, RX_PIN, TX_PIN);
  printf("Launch scheduler\n");
  vTaskStartScheduler();  
  printf("Must never end here\n");
}