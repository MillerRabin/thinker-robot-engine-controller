#pragma once
#include <atomic>
#include <cstdint>

template <typename T, uint16_t SIZE> class RingBuffer {
private:
  T buffer[SIZE];
  std::atomic<uint16_t> head{0};
  std::atomic<uint16_t> tail{0};

public:
  RingBuffer() = default;

  
  bool push(const T &item) {
    uint16_t h = head.load(std::memory_order_relaxed);
    uint16_t next = (h + 1) % SIZE;

    if (next == tail.load(std::memory_order_acquire)) {
      return false;
    }

    buffer[h] = item;
    head.store(next, std::memory_order_release);
    return true;
  }
  
  bool pop(T *item) {
    uint16_t t = tail.load(std::memory_order_relaxed);

    if (t == head.load(std::memory_order_acquire)) {
      return false;
    }

    *item = buffer[t];
    tail.store((t + 1) % SIZE, std::memory_order_release);
    return true;
  }

  bool isEmpty() const {
    return tail.load(std::memory_order_acquire) ==
           head.load(std::memory_order_acquire);
  }

  uint16_t count() const {
    uint16_t h = head.load(std::memory_order_acquire);
    uint16_t t = tail.load(std::memory_order_acquire);
    return (h - t + SIZE) % SIZE;
  }
};