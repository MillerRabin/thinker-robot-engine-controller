#include "statusLed.h"
#include "ws2812.pio.h"

StatusLed::StatusLed(uint pin, PIO pio, uint sm) : pio(pio), sm(sm) {
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, pin, 800000.0f);
  lastBlinkToggle = get_absolute_time();
  putPixel(0, 0, 0);
}

// WS2812 wire order is GRB, not RGB - reordered here so callers can stay in r,g,b.
void StatusLed::putPixel(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t grb = (static_cast<uint32_t>(g) << 16) |
                 (static_cast<uint32_t>(r) << 8) |
                 static_cast<uint32_t>(b);
  pio_sm_put_blocking(pio, sm, grb << 8u);
}

void StatusLed::setState(LedState newState) {
  if (newState != state) {
    state = newState;
    blinkOn = true;
    lastBlinkToggle = get_absolute_time();
  }

  switch (state) {
    case LedState::Off:
      putPixel(0, 0, 0);
      break;
    case LedState::Calibrating:
      putPixel(60, 60, 0);
      break;
    case LedState::GuardTripped:
      if (absolute_time_diff_us(lastBlinkToggle, get_absolute_time()) >= blinkIntervalUs) {
        blinkOn = !blinkOn;
        lastBlinkToggle = get_absolute_time();
      }
      putPixel(blinkOn ? 80 : 0, 0, 0);
      break;
    case LedState::EnginesDisabled:
      putPixel(0, 0, 80);
      break;
  }
}
