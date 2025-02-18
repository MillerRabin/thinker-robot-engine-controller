#include "main.h"

ArmWrist* armWrist;

int main() {	
  //sleep_ms(500); 
  stdio_init_all();
  //printf("Started\n");  
  armWrist = new ArmWrist(MEMS_SDA_PIN, MEMS_SCL_PIN, MEMS_INT_PIN, MEMS_RST_PIN, 
                          WRIST_Z_PIN, WRIST_Y_PIN, 
                          CAN_RX_PIN, CAN_TX_PIN);  
  vTaskStartScheduler();    
}