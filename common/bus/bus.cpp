#include "bus.h"
#include "../armPart/armPart.h"

can2040 Bus::cbus;
CanMap Bus::canSendMap;
can2040_msg Bus::canReceiveMap[CAN_RECEIVE_HASH_CAPACITY];
can2040_msg Bus::localReceiveMap[CAN_RECEIVE_HASH_CAPACITY];
SemaphoreHandle_t Bus::sendMapSemaphore;

void Bus::busReceiveTask(void *pInstance) {
  auto *instance = static_cast<Bus *>(pInstance);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    instance->armPart->setBusReceivingTaskStatus(true);
    taskENTER_CRITICAL();
    for (int i = 0; i < CAN_RECEIVE_HASH_CAPACITY; ++i) {
      Bus::localReceiveMap[i] = Bus::canReceiveMap[i];
      Bus::canReceiveMap[i].id = INVALID_CAN_ID;
    }
    taskEXIT_CRITICAL();
      
    for (int i = 0; i < CAN_RECEIVE_HASH_CAPACITY; ++i) {
      if (Bus::localReceiveMap[i].id == INVALID_CAN_ID)
        continue;
      instance->busCallback(instance->armPart, Bus::localReceiveMap[i]);
      Bus::localReceiveMap[i].id = INVALID_CAN_ID;
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_RECEIVE_LOOP_TIMEOUT));
  }
}

void Bus::can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t mask = taskENTER_CRITICAL_FROM_ISR();
  Bus::canReceiveMap[msg->id % CAN_RECEIVE_HASH_CAPACITY] = *msg;
  taskEXIT_CRITICAL_FROM_ISR(mask);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

Bus::Bus(const uint rxPin, const uint txPin, ArmPart *armPart, CanCallback callback) : 
    armPart(armPart), 
    busCallback(callback) {
  can2040_setup(&Bus::cbus, pio_num);
  can2040_callback_config(&Bus::cbus, Bus::can2040_cb);
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, Bus::PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

  Bus::sendMapSemaphore = xSemaphoreCreateMutex();
  
  xTaskCreate(Bus::busReceiveTask, "busReceiveTask", 4096, this, 5, NULL);
  xTaskCreate(Bus::busSendTask, "busSendTask", 4096, this, 5, NULL);

  can2040_start(&cbus, sys_clock, bitrate, rxPin, txPin);
}

void Bus::busSendTask(void *pInstance) {
  auto *instance = static_cast<Bus *>(pInstance);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    if (xSemaphoreTake(Bus::sendMapSemaphore, pdMS_TO_TICKS(CAN_SEND_WAIT_TIMEOUT)) == pdTRUE) {
      for (auto &item : Bus::canSendMap) {
        can2040_msg frame = item.second;
        int attempts = CAN_SEND_ATTEMPTS;
        while (attempts-- > 0 && can2040_transmit(&cbus, &frame) != 0) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (attempts <= 0) {
          printf("Bus sending error 0x%x\n", frame.id);
        }
      }
      Bus::canSendMap.clear();
      xSemaphoreGive(Bus::sendMapSemaphore);
    }
    else {
      printf("Can't obtain sendMapSemaphore in busSendTask\n");
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_SEND_LOOP_TIMEOUT));
  }
}

void Bus::PIOx_IRQHandler() {
  can2040_pio_irq_handler(&Bus::cbus);
}

void Bus::send(uint32_t id, uint64_t data) {
  can2040_msg msg = {};
  msg.id = id;
  msg.dlc = 8;
  memcpy(&msg.data, &data, 8);

  if (xSemaphoreTake(Bus::sendMapSemaphore, pdMS_TO_TICKS(CAN_SEND_WAIT_TIMEOUT)) == pdTRUE) {
    canSendMap[id] = msg;
    xSemaphoreGive(Bus::sendMapSemaphore);
  }
  else {
    printf("Can't obtain sendMapSemaphore in send()\n");
  }
}
