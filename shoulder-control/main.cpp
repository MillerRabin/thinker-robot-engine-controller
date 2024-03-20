#include "main.h"

#include "armShoulder/armShoulder.h"

#include <math.h>
#include <hardware/pwm.h>

#define SDA_PIN 0
#define SCL_PIN 1
#define INT_PIN 10
#define RST_PIN 2
#define SHOULDER_Z_PIN 8
#define SHOULDER_Y_PIN 3
#define RX_PIN 4
#define TX_PIN 5



int main() {	
  stdio_init_all();  
  sleep_ms(10000);
  printf("Started\n");
  ArmShoulder armShoulder(SDA_PIN, SCL_PIN, INT_PIN, RST_PIN, SHOULDER_Z_PIN, SHOULDER_Y_PIN, RX_PIN, TX_PIN);  
  vTaskStartScheduler();  
}