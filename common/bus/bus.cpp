#include "bus.h"

can2040 Bus::cbus;
CanMap Bus::canSendMap;
CanMap Bus::canReceiveMap;
SemaphoreHandle_t Bus::sendMapSemaphore;
SemaphoreHandle_t Bus::receiveMapSemaphore;

void Bus::busReceiveTask(void* pInstance) {  
  printf("Bus receive task started\n");
  Bus* instance = (Bus*)pInstance;  
  while (true) {    
    if (xSemaphoreTake(Bus::receiveMapSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
      printf("Can't obtain receiveMapSemaphore in busReceiveTask\n");
      continue;
    }
    //printf("Receiving data\n");
    for(auto item: Bus::canReceiveMap) {      
      can2040_msg frame = item.second;      
      instance->busCallback(instance->armPart, frame);
    }
    //canReceiveMap.clear();
    xSemaphoreGive(Bus::receiveMapSemaphore);
    //printf("Semaphore give\n");
    vTaskDelay(pdMS_TO_TICKS(CAN_RECEIVE_LOOP_TIMEOUT));
  }  
}

void Bus::busSendTask(void* pInstance) {
  Bus* instance = (Bus*)pInstance;
  while (true) {        
    /*if (xSemaphoreTake(Bus::sendMapSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
      printf("Can't obtain sendMapSemaphore in busSendTask\n");
      continue;
    }*/
    for(auto item: canSendMap) {
      can2040_msg frame = item.second;      
      if (can2040_transmit(&cbus, &frame) != 0) {
        printf("Bus sending error 0x%x\n", frame.id);
      }      
    }
    //canSendMap.clear();
    //xSemaphoreGive(Bus::sendMapSemaphore);
    vTaskDelay(pdMS_TO_TICKS(CAN_SEND_LOOP_TIMEOUT));
  }  
}

void Bus::can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {      
  /*if (xSemaphoreTake(Bus::receiveMapSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
    printf("Can't obtain receiveMapSemaphore in Bus::can2040_cb\n");
    return;
  } */ 
  canReceiveMap[msg->id] = *msg; 
  //xSemaphoreGive(Bus::receiveMapSemaphore);  
}

void Bus::PIOx_IRQHandler(void) {
  can2040_pio_irq_handler(&Bus::cbus);
}

Bus::Bus(const uint rxPin, const uint txPin, void* instance, CanCallback callback) : 
    armPart(instance),
    busCallback(callback) {   
  can2040_setup(&Bus::cbus, pio_num);
  can2040_callback_config(&Bus::cbus, Bus::can2040_cb);  
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, Bus::PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);  
  //Bus::sendMapSemaphore = xSemaphoreCreateMutex();
  Bus::receiveMapSemaphore = xSemaphoreCreateMutex();
  xTaskCreate(Bus::busReceiveTask, "busReceiveTask", 4096, this, tskIDLE_PRIORITY, NULL);
  xTaskCreate(Bus::busSendTask, "busSendTask", 4096, this, tskIDLE_PRIORITY, NULL);  
  can2040_start(&cbus, sys_clock, bitrate, rxPin, txPin);  
}

void Bus::send(uint32_t id, uint64_t data) {  
  can2040_msg msg = { 0 };
  msg.id = id;
  msg.dlc = 8;  
  memcpy(&msg.data, &data, 8); 
  /*if (xSemaphoreTake(Bus::sendMapSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
    printf("Can't obtain sendMapSemaphore in busSendTask\n");
    return;
  }*/
  canSendMap[id] = msg;    
  //xSemaphoreGive(Bus::sendMapSemaphore);
}