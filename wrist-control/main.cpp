#include "main.h"

ArmWrist* armWrist;

int main() {	
  stdio_init_all();     
  armWrist = new ArmWrist(SDA_PIN, SCL_PIN, INT_PIN, RST_PIN, SHOULDER_Z_PIN, SHOULDER_Y_PIN, RX_PIN, TX_PIN);  
  vTaskStartScheduler();    
}