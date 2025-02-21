#include "main.h"

ArmClaw* armClaw;

int main() {  
  stdio_init_all();
  sleep_ms(1000);
  armClaw = new ArmClaw(DETECTORS_SDA_PIN, DETECTORS_SCL_PIN, CLAW_X_PIN, CLAW_Y_PIN, CLAW_Z_PIN, CLAW_GRIPPER_PIN, 
                        CAN_RX_PIN, CAN_TX_PIN, MEMS_RX_PIN, MEMS_TX_PIN, WITMOTION_XSHUT_PIN, WITMOTION_INT_PIN, 
                        VL_6180X_SHUT_PIN, VL_53LOX_SHUT_PIN);  
  vTaskStartScheduler();    
}