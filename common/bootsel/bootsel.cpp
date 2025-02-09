
#include "bootsel.h"

void rebootInBootMode() {
  reset_usb_boot(1<<PICO_DEFAULT_LED_PIN, 0);
}
