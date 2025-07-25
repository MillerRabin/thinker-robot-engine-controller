#include "bus.h"

can2040 Bus::cbus;
CanMap Bus::canSendMap;
SemaphoreHandle_t Bus::sendMapSemaphore;

can2040_msg Bus::receiveBuffer[CAN_RECEIVE_BUFFER_SIZE];
volatile size_t Bus::receiveHead = 0;
volatile size_t Bus::receiveTail = 0;
SemaphoreHandle_t Bus::receiveBufferMutex;
volatile uint32_t Bus::droppedFrames = 0;

bool Bus::popReceivedFrame(can2040_msg &msg) {
  bool hasData = false;
  if (xSemaphoreTake(Bus::receiveBufferMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    if (receiveHead != receiveTail) {
      msg = receiveBuffer[receiveTail];
      receiveTail = (receiveTail + 1) % CAN_RECEIVE_BUFFER_SIZE;
      hasData = true;
    }
    xSemaphoreGive(Bus::receiveBufferMutex);
  }
  return hasData;
}

void Bus::pushReceivedFrameFromISR(const can2040_msg &msg) {
  size_t nextHead = (receiveHead + 1) % CAN_RECEIVE_BUFFER_SIZE;
  if (nextHead == receiveTail)
  {
    // Буфер полон — перезаписываем старый кадр
    receiveTail = (receiveTail + 1) % CAN_RECEIVE_BUFFER_SIZE;
    droppedFrames++;
  }
  receiveBuffer[receiveHead] = msg;
  receiveHead = nextHead;
}

// === Задача приёма ===
void Bus::busReceiveTask(void *pInstance)
{
  auto *instance = static_cast<Bus *>(pInstance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  can2040_msg frame;

  while (true)
  {
    while (instance->popReceivedFrame(frame))
    {
      instance->busCallback(instance->armPart, frame);
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_RECEIVE_LOOP_TIMEOUT));
  }
}

void Bus::can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  Bus::pushReceivedFrameFromISR(*msg);
}

Bus::Bus(const uint rxPin, const uint txPin, void *instance, CanCallback callback)
    : armPart(instance), busCallback(callback) {
  can2040_setup(&Bus::cbus, pio_num);
  can2040_callback_config(&Bus::cbus, Bus::can2040_cb);
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, Bus::PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

  Bus::sendMapSemaphore = xSemaphoreCreateMutex();
  Bus::receiveBufferMutex = xSemaphoreCreateMutex();

  xTaskCreate(Bus::busReceiveTask, "busReceiveTask", 4096, this, 5, NULL);
  xTaskCreate(Bus::busSendTask, "busSendTask", 4096, this, 5, NULL);

  can2040_start(&cbus, sys_clock, bitrate, rxPin, txPin);
}

void Bus::busSendTask(void *pInstance) {
  auto *instance = static_cast<Bus *>(pInstance);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    if (xSemaphoreTake(Bus::sendMapSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
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
    } else {
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
