#pragma once

#include "FreeRTOS.h"
#include "task.h"

class Periodic {
public:
  explicit Periodic(TickType_t intervalTicks) : interval_(intervalTicks), lastWake_(xTaskGetTickCount()) {}

  template <typename Func> void run(Func &&func) {
    TickType_t now = xTaskGetTickCount();

    if ((now - lastWake_) >= interval_) {
      lastWake_ = now;
      func();
    }
  }

  void reset() { lastWake_ = xTaskGetTickCount(); }

private:
  TickType_t interval_;
  TickType_t lastWake_;
};