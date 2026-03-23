#include "main.h"

ArmShoulder* armShoulder;

int main() {	
  stdio_init_all();    
  armShoulder = new ArmShoulder(
      SHOULDER_MEMS_SCK_PIN, SHOULDER_MEMS_MISO_PIN, SHOULDER_MEMS_MOSI_PIN, SHOULDER_MEMS_CS_PIN,
      MEMS_INT_PIN, SHOULDER_MEMS_RST_PIN, SHOULDER_Z_PIN, SHOULDER_Y_PIN,
      CAN_RX_PIN, CAN_TX_PIN);    
  auto res = armShoulder->begin();
  fflush(stdout);
  if (res != 0) {
    printf("Failed to initialize ArmShoulder, error code: %d\n", res);
    return res;
  }  
  vTaskStartScheduler();    
  return 0;
}