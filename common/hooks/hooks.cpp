#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "task.h"

extern "C" {

  static void panic_blink(int pattern) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1) {
      for (int i = 0; i < pattern; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_ms(200);
      }
      busy_wait_ms(1000);
    }
  }

  void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
      panic_blink(2);
  }

  void vApplicationMallocFailedHook(void) {
    panic_blink(4);
  }

}