#include "bus.h"

can2040 Bus::cbus;
volatile QueueHandle_t Bus::queue;

void Bus::busTask(void* pInstance) {
  Bus* instance = (Bus*)pInstance;
  while (true) { 
    can2040_msg busMsg;    
    xQueueReceive(Bus::queue, &busMsg, portMAX_DELAY);
    instance->busCallback(instance->armPart, busMsg);
  }  
}


void Bus::can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {    
  xQueueSend(Bus::queue, msg, 0);
}

void Bus::PIOx_IRQHandler(void) {
  can2040_pio_irq_handler(&Bus::cbus);
}


Bus::Bus(const uint rxPin, const uint txPin, void* instance, CanCallback callback) : 
    armPart(instance),
    busCallback(callback) { 
  Bus::queue = xQueueCreate(10, sizeof(can2040_msg));
  can2040_setup(&Bus::cbus, pio_num);
  can2040_callback_config(&Bus::cbus, Bus::can2040_cb);  
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, Bus::PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);  
  xTaskCreate(Bus::busTask, "busTask", 1024, this, 5, NULL);
  can2040_start(&cbus, sys_clock, bitrate, rxPin, txPin);  
}

int Bus::send(uint32_t id, uint64_t data) {
  can2040_msg msg;
  msg.id = id;             
  msg.dlc = 8;
  memcpy(msg.data, &data, 8);  
  return can2040_transmit(&cbus, &msg);  
}