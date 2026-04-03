#pragma once
#include <cstdint>

template <typename T, uint16_t SIZE> class RingBuffer {
private:
  T buffer[SIZE];
  volatile uint16_t head = 0;
  volatile uint16_t tail = 0;

public:
  inline bool push(const T &item) {
    uint16_t next = (head + 1) % SIZE;
    if (next == tail)
      return false;
    buffer[head] = item;
    head = next;
    return true;
  }

  inline bool pop(T *item) {
    if (tail == head)
      return false;
    *item = buffer[tail];
    tail = (tail + 1) % SIZE;
    return true;
  }

  inline bool peek(uint16_t i, T *item) const {
    if (i >= count())
      return false;
    *item = buffer[(tail + i) % SIZE];
    return true;
  }

  inline bool isEmpty() const { return tail == head; }
  inline uint16_t count() const { return (head - tail + SIZE) % SIZE; }  
};