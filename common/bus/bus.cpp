#include "bus.h"
#include "../armPart/armPart.h"

can2040 Bus::cbus;
void* Bus::instance = NULL;

void Bus::busReceiveTask(void *pInstance) {
  auto *instance = static_cast<Bus *>(pInstance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  CanFrame frame;
  while (true) {
    if (instance->reinitInProgress) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    xEventGroupWaitBits(instance->events, CAN_EVENT_RX, pdTRUE, pdFALSE, portMAX_DELAY);
    instance->armPart->setBusReceivingTaskStatus(true);
    while (instance->receiveBuffer.pop(&frame)) {      
      instance->busCallback(instance->armPart, frame);
    }    
  }
}

void Bus::can2040_cb(struct can2040 *cd, uint32_t notify, CanFrame *msg) {  
  auto *bus = static_cast<Bus *>(Bus::instance);
  
  if (notify == CAN2040_NOTIFY_ERROR) {
    bus->requestReinit();
    return;
  }

  if (notify != CAN2040_NOTIFY_RX) {
    return;
  }

  bus->lastRxTime = to_ms_since_boot(get_absolute_time());
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (bus->receiveBuffer.push(*msg)) {
    xEventGroupSetBitsFromISR(bus->events, CAN_EVENT_RX,
                              &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Bus::initTask(void *instance) {
  Bus *bus = static_cast<Bus *>(instance);
  while (true) {
    xEventGroupWaitBits(bus->events, CAN_EVENT_REINIT, pdTRUE, pdFALSE, portMAX_DELAY);
    printf("Restarting CAN\n");
    bus->init();
  }
}

Bus::Bus(const uint rxPin, const uint txPin, ArmPart *armPart, CanCallback callback) : 
    rxPin(rxPin),
    txPin(txPin),
    armPart(armPart), 
    busCallback(callback) {

  Bus::instance = this;
  can2040_setup(&cbus, pio_num);
  can2040_callback_config(&cbus, Bus::can2040_cb);
  irq_set_exclusive_handler(PIO0_IRQ_0, Bus::PIOx_IRQHandler);
  irq_set_priority(PIO0_IRQ_0, 1);
  irq_set_enabled(PIO0_IRQ_0, 1);
}

can_status_t Bus::begin() {
  if (events == NULL) {
    events = xEventGroupCreate();
    if (events == NULL) {
      printf("Failed to create event group\n");
      return CAN_EVENT_GROUP_CREATION_FAILED;
    }
  }
  if (sendMutex == NULL) {
    sendMutex = xSemaphoreCreateMutex();
    if (sendMutex == NULL) {
      printf("Failed to create sendMutex\n");
      return CAN_SEND_MUTEX_CREATION_FAILED;
    }
  }

  if (receiveTaskHandle == NULL) {
    if (xTaskCreate(Bus::busReceiveTask, "busReceiveTask", 2048, this, 5,
                    &receiveTaskHandle) != pdPASS) {
      printf("Failed to create receiveTask\n");
      return CAN_TASK_RECEIVE_CREATION_FAILED;
    }
  }

  if (sendTaskHandle == NULL) {
    if (xTaskCreate(Bus::busSendTask, "busSendTask", 2048, this, 5, &sendTaskHandle) != pdPASS) {
      printf("Failed to create sendTask\n");
      return CAN_TASK_SEND_CREATION_FAILED;
    }
  }

  if (watchdogTaskHandle == NULL) {
    if (xTaskCreate(Bus::busWatchdogTask, "busWatchdogTask", 2048, this, 2, &watchdogTaskHandle) != pdPASS) {
      printf("Failed to create watchDog\n");
      return CAN_TASK_WATCHDOG_CREATION_FAILED;
    }
  }

  if (initTaskHandle == NULL) {
    if (xTaskCreate(Bus::initTask, "busInitTask", 2048, this, 5,
                    &initTaskHandle) != pdPASS) {
      printf("Failed to create initTask\n");
      return CAN_TASK_INIT_CREATION_FAILED;
    }
  }

  init();
  return CAN_SUCCESS;
}

can_status_t Bus::init() {    
  can2040_stop(&cbus);  
  vTaskDelay(pdMS_TO_TICKS(100));
  can2040_start(&cbus, sys_clock, bitrate, rxPin, txPin);  
  lastRxTime = to_ms_since_boot(get_absolute_time());
  reinitInProgress = false;  
  return CAN_SUCCESS;
}

void Bus::requestReinit() {
  if (!reinitInProgress) {
    reinitInProgress = true;
    xEventGroupSetBits(events, CAN_EVENT_REINIT);
  }
}

void Bus::busSendTask(void *pInstance) {
  auto *instance = static_cast<Bus *>(pInstance);
  CanFrame localFrames[CAN_TX_BUFFER_SIZE];
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(instance->sendMutex, pdMS_TO_TICKS(CAN_SEND_WAIT_TIMEOUT)) == pdTRUE) {
      memcpy(localFrames, instance->sendBuffer, sizeof(localFrames));
      memset(instance->sendBuffer, 0, sizeof(instance->sendBuffer));                  
      xSemaphoreGive(instance->sendMutex);
      if (!instance->isBusAlive()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      if (instance->reinitInProgress) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      bool transferSuccess = false;

      for (uint i = 0; i < CAN_TX_BUFFER_SIZE; i++) {
        CanFrame frame = localFrames[i];
        if (frame.dlc != 0) {
          transferSuccess = (can2040_transmit(&(Bus::cbus), &frame) == 0);
          if (!transferSuccess) {          
            break;
          }
        }
      }
    }    
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_SEND_LOOP_TIMEOUT));
  }
}

bool Bus::send(uint32_t id, uint64_t data) {
  can2040_msg msg = {};
  msg.id = id;
  msg.dlc = 8;
  memcpy(&msg.data, &data, 8);
  if (xSemaphoreTake(sendMutex, pdMS_TO_TICKS(CAN_SEND_WAIT_TIMEOUT)) == pdTRUE) {
    sendBuffer[id % CAN_TX_BUFFER_SIZE] = msg;
    xSemaphoreGive(sendMutex); 
    return true;
  } else {
    return false;
  }
}

bool Bus::isBusAlive() {
  uint32_t now = to_ms_since_boot(get_absolute_time());
  if (now - lastRxTime > 3000) {
    return false;
  }
  return true;
}

void Bus::busWatchdogTask(void *instance) {
  Bus *bus = static_cast<Bus *>(instance);
  while (true) {
    if (!bus->isBusAlive()) {      
      bus->requestReinit();
    } else {      
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Bus::PIOx_IRQHandler() {
  can2040_pio_irq_handler(&Bus::cbus);
}
