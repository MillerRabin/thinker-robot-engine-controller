#include "main.h"

ArmWrist* armWrist;

int main() {	
  stdio_init_all();     
  armWrist = new ArmWrist(MEMS_SDA_PIN, MEMS_SCL_PIN, MEMS_INT_PIN, MEMS_RST_PIN, 
                          WRIST_Z_PIN, WRIST_Y_PIN, 
                          CAN_RX_PIN, CAN_TX_PIN);  
  vTaskStartScheduler();    
}