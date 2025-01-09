#include "armClaw.h"

volatile QueueHandle_t ArmClaw::queue;

void ArmClawQueueParams::set(uint8_t data[]) {
  uint16_t clawX;
  uint16_t clawY;
  uint16_t clawZ;
  uint16_t clawGripper;
  
  memcpy(&clawX, &data[0], 2);
  memcpy(&clawY, &data[2], 2);
  memcpy(&clawZ, &data[4], 2);
  memcpy(&clawGripper, &data[6], 2);


}

void ArmClaw::engineTask(void *instance) {  
  ArmClaw* claw = (ArmClaw*)instance;
  while(true) {        
    claw->clawX.tick();   
    claw->clawY.tick();
    claw->clawZ.tick(); 
    claw->clawGripper.tick();
    Euler rEuler = claw->platform.bno.quaternion.getEuler();    
    printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    Euler sEuler = claw->position.quaternion.getEuler();      
    printf("Wrist roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle()); 
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ArmClaw::busReceiverTask(void *instance) {  
  ArmClaw* claw = (ArmClaw*)instance;
  while(true) {        
    ArmClawQueueParams params;    
    xQueueReceive(ArmClaw::queue, &params, portMAX_DELAY);        
    claw->clawX.setTargetAngle(params.clawX); 
    claw->clawY.setTargetAngle(params.clawY);
    claw->clawZ.setTargetAngle(params.clawZ);
  }
}

ArmClaw::ArmClaw(
  const uint detectorsSdaPin, 
  const uint detectorsSclPin, 
  const uint engineXPin,
  const uint engineYPin,
  const uint engineZPin, 
  const uint engineGripperPin,
  const uint canRxPin,
  const uint canTxPin,
  const uint memsRxPin,
  const uint memsTxPin,
  const uint memsRstPin,
  const uint memsIntPin
) :
    ArmPart(canRxPin, canTxPin),    
    clawX(engineXPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawZ(engineZPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawGripper(engineGripperPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    position(this, memsRxPin, memsTxPin, memsRstPin, memsIntPin)
  {            
    ArmClaw::queue = xQueueCreate(10, sizeof(ArmClawQueueParams));
    xTaskCreate(ArmClaw::busReceiverTask, "ArmClaw::busReceiverTask", 1024, this, 1, NULL);
    xTaskCreate(ArmClaw::engineTask, "ArmClaw::engineTask", 1024, this, tskIDLE_PRIORITY, NULL);
}

int ArmClaw::updateQuaternion(BasePosition* position) {  
  Euler euler = position->quaternion.getEuler();  
  //printf("Euler get pitch angle: %f\n", euler.getPitchAngle());
  clawX.euler = euler; 
  clawY.euler = euler;
  clawZ.euler = euler;
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmClaw::updateGyroscope(BasePosition* position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmClaw::updateAccelerometer(BasePosition* position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmClaw::updateAccuracy(BasePosition* position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmClaw::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_CLAW_SET_XYZG_DEGREE) {    
    ArmClawQueueParams params;
    params.set(frame.data);    
    xQueueSend(ArmClaw::queue, &params, 0);
  }
}