#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include <atomic>

template <typename T> class AtomicValue {
  T buffers[2];
  std::atomic<uint32_t> index{0};
  std::atomic<TickType_t> lastUpdated{0};
  TickType_t maxInterval;

public:
  AtomicValue() : maxInterval(pdMS_TO_TICKS(500)) {}

  explicit AtomicValue(TickType_t maxInterval) : maxInterval(maxInterval) {}

  explicit AtomicValue(const T &initial,
                       TickType_t maxInterval = pdMS_TO_TICKS(500))
      : maxInterval(maxInterval) {
    buffers[0] = initial;
    buffers[1] = initial;
  }

  void store(const T &value) {
    uint32_t next = (index.load(std::memory_order_relaxed) + 1) & 1;
    buffers[next] = value;
    index.store(next, std::memory_order_release);
    lastUpdated.store(xTaskGetTickCount(), std::memory_order_relaxed);
  }

  bool isFresh() const {
    TickType_t now = xTaskGetTickCount();
    return (now - lastUpdated.load(std::memory_order_relaxed)) < maxInterval;
  }

  void makeFresh() {
    lastUpdated.store(xTaskGetTickCount(), std::memory_order_relaxed);
  }

  T load() const { return buffers[index.load(std::memory_order_acquire)]; }
};