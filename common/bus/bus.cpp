#include "bus.h"

can2040 Bus::cbus;
CanMap Bus::canSendMap;
CanMap Bus::canReceiveMap;
SemaphoreHandle_t Bus::sendMapSemaphore;
SemaphoreHandle_t Bus::receiveMapSemaphore;

void Bus::busReceiveTask(void *pInstance) {
  auto *instance = static_cast<Bus *>(pInstance);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    CanMap localCopy;
    if (xSemaphoreTake(Bus::receiveMapSemaphore, pdMS_TO_TICKS(1)) == pdTRUE) {
      localCopy = std::move(Bus::canReceiveMap);
      Bus::canReceiveMap.clear();
      xSemaphoreGive(Bus::receiveMapSemaphore);
    }

    for (auto &item : localCopy) {
      instance->busCallback(instance->armPart, item.second);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_RECEIVE_LOOP_TIMEOUT));
  }
}

void Bus::can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (xSemaphoreTakeFromISR(Bus::receiveMapSemaphore, &xHigherPriorityTaskWoken) == pdTRUE) {
    Bus::canReceiveMap[msg->id] = *msg;
    xSemaphoreGiveFromISR(Bus::receiveMapSemaphore, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

Bus::Bus(const uint rxPin, const uint txPin, void *instance, CanCallback callback)
    : armPart(instance), busCallback(callback) {
  can2040_setup(&Bus::cbus, pio_num);
  can2040_callback_config(&Bus::cbus, Bus::can2040_cb);
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, Bus::PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

  Bus::sendMapSemaphore = xSemaphoreCreateMutex();
  Bus::receiveMapSemaphore = xSemaphoreCreateMutex();
  
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
          vTaskDelay(pdMS_TO_TICKS(1));
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
