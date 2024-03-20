#include "bus.h"

can2040 Bus::cbus;
volatile QueueHandle_t Bus::queue;

void Bus::busTask(void *pvParameters) {
  while (true) { 
    can2040_msg busMsg;
    xQueueReceive(Bus::queue, &busMsg, portMAX_DELAY);
        
    //printf("data received id: 0x%x\n", busMsg.id);
    //printf("busMsg 0x%x:%x\n", busMsg.data32[0], busMsg.data32[1]);
  }  
}

void Bus::can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{    
  xQueueSend(Bus::queue, msg, 0);
}

void Bus::PIOx_IRQHandler(void) {
  can2040_pio_irq_handler(&Bus::cbus);
}


Bus::Bus(const uint rxPin, const uint txPin) : gpio_rx(rxPin),
                                               gpio_tx(txPin)
{    
  Bus::queue = xQueueCreate(10, sizeof(can2040_msg));
  can2040_setup(&Bus::cbus, pio_num);
  can2040_callback_config(&Bus::cbus, Bus::can2040_cb);  
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, Bus::PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);  
  BaseType_t busStatus = xTaskCreate(Bus::busTask, "busTask", 1024, NULL, 5, NULL);
  can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);  
}

int Bus::send(uint32_t id, uint8_t* data, uint8_t length) {
  can2040_msg msg;
  msg.id = id;  
  for (int i = 0; i < length; i = i + 8) {
    const uint diff = length - i;
    const uint eLength = (diff > 8) ? 8 : diff;
    msg.data32[0] = 0;
    msg.data32[1] = 0;
    msg.dlc = i + 1;
    memcpy(&msg.data, &data[i], eLength);
    uint res = can2040_transmit(&cbus, &msg);
    if (res == -1) return res;
  };
  return 0;  
}