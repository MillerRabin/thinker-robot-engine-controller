#pragma once

#include "FreeRTOS.h"
#include "task.h"

class Periodic {
public:
  explicit Periodic(TickType_t intervalTicks) : interval_(intervalTicks), lastWake_(xTaskGetTickCount()) {}

  template <typename Func> void interval(Func &&func) {
    TickType_t now = xTaskGetTickCount();

    if ((now - lastWake_) >= interval_) {
      lastWake_ = now;
      func();
    }
  }

  template <typename Func> void timeout(Func &&func) {
    TickType_t now = xTaskGetTickCount();
    if (!called && (now - lastWake_) >= interval_) {      
      called = true;
      func();
    }
  }

  void reset() { 
    lastWake_ = xTaskGetTickCount(); 
    called = false;
  }

  bool isCalled() { return called; }

private:
  TickType_t interval_;
  TickType_t lastWake_;
  bool called = false;
};