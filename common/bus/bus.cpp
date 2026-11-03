#include "bus.h"
#include "../armPart/armPart.h"

can2040 Bus::cbus;
void* Bus::instance = NULL;

void Bus::busReceiveTask(void *pInstance) {  
  auto *instance = static_cast<Bus *>(pInstance);
  TickType_t lastWakeTime = xTaskGetTickCount();
  CanFrame frame;
  while (true) {
    if (instance->reinitInProgress.load(std::memory_order_relaxed)) {      
      vTaskDelay(pdMS_TO_TICKS(1000));
      lastWakeTime = xTaskGetTickCount();
      continue;
    }
    instance->armPart->setBusReceivingTaskStatus(true);
    bool received = false;
    while (instance->receiveBuffer.pop(&frame)) {
      received = true;
      instance->busCallback(instance->armPart, frame);
    }
    // Stamped here rather than in can2040_cb: the clock read and 64-bit divide live in flash
    if (received)
      instance->lastRxTime.store(to_ms_since_boot(get_absolute_time()), std::memory_order_relaxed);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5));
  }  
}

// Runs inside the can2040 irq, so it lives in RAM with can2040 itself (see RAM_CODE_LINKER_SCRIPT)
void __not_in_flash("can2040") Bus::can2040_cb(struct can2040 *cd, uint32_t notify, CanFrame *msg) {
  auto *bus = static_cast<Bus *>(Bus::instance);
  if (notify == CAN2040_NOTIFY_ERROR) {
    bus->rxStalls = bus->rxStalls + 1;
    return;
  }

  if (notify == CAN2040_NOTIFY_TX) {
    bus->txInFlight = false;
    return;
  }

  if (notify != CAN2040_NOTIFY_RX) {
    return;
  }

  bus->receiveBuffer.push(*msg);
}

void Bus::initTask(void *instance) {  
  Bus *bus = static_cast<Bus *>(instance);
  uint32_t seconds = 0;
  // Start can2040 right away: isBusAlive() only reports a dead bus 3 s after boot (lastRxTime starts at 0)
  bus->init();
  while (true) {    
    if (!bus->isBusAlive()) {
      LogQueue::Log("Bus is not alive\n");
      bus->init();  
    } else {}
    if (++seconds % CAN_STATS_LOG_INTERVAL == 0)
      bus->logStatistics();
    vTaskDelay(pdMS_TO_TICKS(1000));      
  }
}

// can2040 counters per interval. Parse errors are the corruption signal: they rise on every node that
// hears a damaged frame. tx attempts above tx sent is normal on a busy bus - can2040 re-arms (and counts)
// a pending transmit after each frame it acks while waiting, as well as after a lost arbitration.
void Bus::logStatistics() {
  can2040_stats stats;
  can2040_get_statistics(&cbus, &stats);
  uint32_t stalls = rxStalls;
  if (stats.tx_attempt < lastStats.tx_attempt)
    lastStats = {};
  LogQueue::Log("CAN stats %us: rx %lu, tx %lu, tx attempts %lu, parse errors %lu, rx stalls %lu\n",
                (unsigned)CAN_STATS_LOG_INTERVAL,
                (unsigned long)(stats.rx_total - lastStats.rx_total),
                (unsigned long)(stats.tx_total - lastStats.tx_total),
                (unsigned long)(stats.tx_attempt - lastStats.tx_attempt),
                (unsigned long)(stats.parse_error - lastStats.parse_error),
                (unsigned long)(stalls - lastRxStalls));
  lastStats = stats;
  lastRxStalls = stalls;
  // Longest pause between transmits of each frame id this interval (ms) - a receiver drops a sender
  // whose frames pause past its freshness window (500 ms for RemoteWrist).
  char gaps[LOG_QUEUE_MSG_SIZE - 2];
  int len = snprintf(gaps, sizeof(gaps), "CAN tx max gap ms:");
  for (uint i = 0; i < CAN_TX_BUFFER_SIZE && len < (int)sizeof(gaps) - 10; i++) {
    if (slotLastTx[i] == 0)
      continue;
    len += snprintf(gaps + len, sizeof(gaps) - len, " %02lx:%lu", (unsigned long)slotId[i],
                    (unsigned long)(slotMaxGap[i] * portTICK_PERIOD_MS));
    slotMaxGap[i] = 0;
  }
  LogQueue::Log("%s\n", gaps);
}

Bus::Bus(const uint rxPin, const uint txPin, ArmPart *armPart, CanCallback callback) : 
    rxPin(rxPin),
    txPin(txPin),
    armPart(armPart), 
    busCallback(callback) {

  Bus::instance = this;
  // Hold TX recessive (high) until can2040_start() hands the pin to the PIO. The pad's reset default is
  // a pull-down, which drags the transceiver's D input toward dominant while nothing drives it.
  gpio_init(txPin);
  gpio_pull_up(txPin);
  gpio_put(txPin, 1);
  gpio_set_dir(txPin, GPIO_OUT);
  can2040_setup(&cbus, pio_num);
  can2040_callback_config(&cbus, Bus::can2040_cb);
  irq_set_exclusive_handler(PIO0_IRQ_0, Bus::PIOx_IRQHandler);
  irq_set_priority(PIO0_IRQ_0, 1);
  irq_set_enabled(PIO0_IRQ_0, 1);
}

can_status_t Bus::begin() {  
  if (sendMutex == NULL) {
    sendMutex = xSemaphoreCreateMutex();
    if (sendMutex == NULL) {
      LogQueue::Log("Failed to create sendMutex\n");
      return CAN_SEND_MUTEX_CREATION_FAILED;
    }
  }

  if (receiveTaskHandle == NULL) {
    if (xTaskCreateAffinitySet(Bus::busReceiveTask, "busReceiveTask", 2048, this, 6, CAN_CORE, &receiveTaskHandle) != pdPASS) {
      LogQueue::Log("Failed to create receiveTask\n");
      return CAN_TASK_RECEIVE_CREATION_FAILED;
    }
  }
  
  if (sendTaskHandle == NULL) {
    if (xTaskCreateAffinitySet(Bus::busSendTask, "busSendTask", 2048, this, 5, CAN_CORE, &sendTaskHandle) != pdPASS) {
      LogQueue::Log("Failed to create sendTask\n");
      return CAN_TASK_SEND_CREATION_FAILED;
    }
  }
    
  if (initTaskHandle == NULL) {
    if (xTaskCreateAffinitySet(Bus::initTask, "busInitTask", 2048, this, 5, CAN_CORE, &initTaskHandle) != pdPASS) {
      LogQueue::Log("Failed to create initTask\n");
      return CAN_TASK_INIT_CREATION_FAILED;
    }
  }
  
  return CAN_SUCCESS;
}

can_status_t Bus::init() {
  reinitInProgress.store(true, std::memory_order_relaxed);
  can2040_stop(&cbus);
  vTaskDelay(pdMS_TO_TICKS(100));
  taskENTER_CRITICAL();
  can2040_start(&cbus, sys_clock, bitrate, rxPin, txPin);
  lastRxTime.store(to_ms_since_boot(get_absolute_time()),
                   std::memory_order_relaxed);
  taskEXIT_CRITICAL();

  reinitInProgress.store(false, std::memory_order_relaxed);
  return CAN_SUCCESS;
}

// Hands can2040 one frame at a time, at most one per tick. Queueing several let can2040 send them back
// to back, each SOF right after this node's own previous frame, and at a 5 ms period those follow-up
// frames were misread by every can2040 node, this one included.
void Bus::busSendTask(void *pInstance) {  
  auto *instance = static_cast<Bus *>(pInstance);
  CanFrame pending[CAN_TX_BUFFER_SIZE] = {};
  uint32_t next = 0;
  TickType_t lastSnapshot = xTaskGetTickCount();
  TickType_t txSince = 0;
  while (true) {
    vTaskDelay(1);
    TickType_t now = xTaskGetTickCount();
    if (now - lastSnapshot >= pdMS_TO_TICKS(CAN_SEND_LOOP_TIMEOUT)) {
      lastSnapshot = now;
      if (xSemaphoreTake(instance->sendMutex, pdMS_TO_TICKS(CAN_SEND_WAIT_TIMEOUT)) == pdTRUE) {
        for (uint i = 0; i < CAN_TX_BUFFER_SIZE; i++)
          if (instance->sendBuffer[i].dlc != 0)
            pending[i] = instance->sendBuffer[i];
        memset(instance->sendBuffer, 0, sizeof(instance->sendBuffer));
        xSemaphoreGive(instance->sendMutex);
      }
    }
    if (!instance->isBusAlive() || instance->reinitInProgress.load(std::memory_order_relaxed)) {
      instance->txInFlight = false;
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (instance->txInFlight) {
      // can2040_stop() on a reinit drops the queue without a NOTIFY_TX - don't wait on it forever
      if (now - txSince < pdMS_TO_TICKS(CAN_TX_IN_FLIGHT_TIMEOUT))
        continue;
      instance->txInFlight = false;
    }
    for (uint n = 0; n < CAN_TX_BUFFER_SIZE; n++) {
      uint i = (next + n) % CAN_TX_BUFFER_SIZE;
      if (pending[i].dlc == 0)
        continue;
      instance->txInFlight = true;
      txSince = now;
      if (can2040_transmit(&(Bus::cbus), &pending[i]) != 0) {
        instance->txInFlight = false;
        break;
      }
      if (instance->slotLastTx[i] != 0 && now - instance->slotLastTx[i] > instance->slotMaxGap[i])
        instance->slotMaxGap[i] = now - instance->slotLastTx[i];
      instance->slotLastTx[i] = now;
      instance->slotId[i] = pending[i].id;
      pending[i].dlc = 0;
      next = i + 1;
      break;
    }
  }
}

bool Bus::send(uint32_t id, uint64_t data) {
  can2040_msg msg = {};
  msg.id = id;
  msg.dlc = 8;
  memcpy(&msg.data, &data, 8);    
  if (xSemaphoreTake(sendMutex, pdMS_TO_TICKS(CAN_SEND_WAIT_TIMEOUT)) == pdTRUE) {
    int32_t index = id % CAN_TX_BUFFER_SIZE;    
    sendBuffer[index] = msg;
    xSemaphoreGive(sendMutex); 
    return true;
  } else {
    return false;
  }
}

// Signed difference: busReceiveTask() may stamp lastRxTime between our clock read and its load, and
// unsigned that wraps to ~4e9 - "dead" - so busSendTask() slept 1000 ms with frames still arriving.
// The wrist hit it often enough (all its frames paused 1007 ms, its status up to 4 s) that the claw,
// which drops the wrist after 500 ms of silence, kept recalibrating (2026-11-03).
bool Bus::isBusAlive() {
  uint32_t now = to_ms_since_boot(get_absolute_time());
  return (int32_t)(now - lastRxTime.load(std::memory_order_relaxed)) <= 3000;
}

void __not_in_flash("can2040") Bus::PIOx_IRQHandler() {
  can2040_pio_irq_handler(&Bus::cbus);
}
