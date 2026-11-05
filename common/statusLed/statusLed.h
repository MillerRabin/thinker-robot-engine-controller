#pragma once

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

enum class LedState : uint8_t {
  Off,
  Calibrating,
  EnginesDisabled,
  Ready,
};

class StatusLed {
private:
  PIO pio;
  uint sm;
  LedState state = LedState::Off;
  void putPixel(uint8_t r, uint8_t g, uint8_t b);

public:
  StatusLed(uint pin, PIO pio = pio1, uint sm = 0);
  void setState(LedState newState);
};
