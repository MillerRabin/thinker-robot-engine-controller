#include "main.h"

ArmShoulder* armShoulder;

int main() {	
  stdio_init_all();     
  LogQueue::Init();
  armShoulder = new ArmShoulder(
      MEMS_SCK_PIN, MEMS_MISO_PIN, MEMS_MOSI_PIN, MEMS_CS_PIN,
      MEMS_INT_PIN, MEMS_RST_PIN, SHOULDER_Z_PIN, SHOULDER_Y_PIN,
      CAN_RX_PIN, CAN_TX_PIN);     
  auto res = armShoulder->begin();  
  if (res != 0) {
    printf("Failed to initialize ArmShoulder, error code: %d\n", res);
    return res;
  }
  vTaskStartScheduler();
  return 0;
}