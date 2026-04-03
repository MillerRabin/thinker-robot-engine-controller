#pragma once
#include <cstdint>

template <typename T, uint16_t SIZE> class OverwriteRingBuffer {
private:
  T buffer[SIZE];
  volatile uint16_t head = 0;
  volatile uint16_t tail = 0;
  volatile bool full = false;

public:
  inline void push(const T &item) {
    buffer[head] = item;
    head = (head + 1) % SIZE;
    if (full)
      tail = (tail + 1) % SIZE; // затираем старый элемент
    full = (head == tail);
  }

  inline bool pop(T *item) {
    if (isEmpty())
      return false;
    *item = buffer[tail];
    tail = (tail + 1) % SIZE;
    full = false;
    return true;
  }

  inline bool peek(uint16_t i, T *item) const {
    if (i >= count())
      return false;
    *item = buffer[(tail + i) % SIZE];
    return true;
  }

  inline bool isEmpty() const { return !full && (head == tail); }
  inline bool isFull() const { return full; }
  inline uint16_t count() const {
    if (full)
      return SIZE;
    return (head - tail + SIZE) % SIZE;
  }
};