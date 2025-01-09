#include "main.h"

ArmElbow* armElbow;

int main() {	
  stdio_init_all();  
  //sleep_ms(10000);
  printf("Started\n");  
  armElbow = new ArmElbow(MEMS_SDA_PIN, MEMS_SCL_PIN, MEMS_INT_PIN, MEMS_RST_PIN, 
                          ELBOW_Y_PIN, CAN_RX_PIN, CAN_TX_PIN);
  vTaskStartScheduler();    
}