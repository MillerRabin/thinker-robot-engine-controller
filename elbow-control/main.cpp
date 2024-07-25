#include "main.h"

ArmElbow* armElbow;

int main() {	
  stdio_init_all();  
  //sleep_ms(10000);
  printf("Started\n");  
  armElbow = new ArmElbow(SDA_PIN, SCL_PIN, INT_PIN, RST_PIN, ELBOW_Y_PIN, RX_PIN, TX_PIN);
  vTaskStartScheduler();    
}