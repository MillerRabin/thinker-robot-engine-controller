#include "armShoulder.h"

volatile QueueHandle_t ArmShoulder::queue;

void ArmShoulder::engineTask(void *instance) {  
  ArmShoulder* shoulder = (ArmShoulder*)instance;
  while(true) {        
    shoulder->shoulderY.tick();   
    
    Euler rEuler = shoulder->platform.bno.quaternion.getEuler();    
    //printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    
    Euler sEuler = shoulder->position.quaternion.getEuler();      
    //printf("Shoulder roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle()); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ArmShoulder::busReceiverTask(void *instance) {  
  ArmShoulder* shoulder = (ArmShoulder*)instance;
  while(true) {        
    ArmShoulderQueueParams params;    
    xQueueReceive(ArmShoulder::queue, &params, portMAX_DELAY);    
    shoulder->shoulderZ.setTargetAngle(params.shoulderZ);    
    shoulder->shoulderY.setTargetAngle(params.shoulderY);
  }
}

ArmShoulder::ArmShoulder(
  const uint memsSdaPin, 
  const uint memsSclPin, 
  const uint memsIntPin, 
  const uint memsRstPin, 
  const uint engineZPin, 
  const uint engineYPin, 
  const uint canRxPin,
  const uint canTxPin) :
    ArmPart(canRxPin, canTxPin),
    shoulderZ(engineZPin, Range(0, 270), Range(-180, 180), IMU_USE_YAW, 100),
    shoulderY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    position(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
  {            
    ArmShoulder::queue = xQueueCreate(10, sizeof(ArmShoulderQueueParams));
    xTaskCreate(ArmShoulder::busReceiverTask, "ArmShoulder::busReceiverTask", 1024, this, 1, NULL);
    xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 1024, this, tskIDLE_PRIORITY, NULL);
}

int ArmShoulder::updateQuaternion(BasePosition* position) {  
  Euler euler = position->quaternion.getEuler();  
  //printf("Euler get pitch angle: %f\n", euler.getPitchAngle());
  shoulderY.euler = euler;
  shoulderZ.euler = euler;
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmShoulder::updateGyroscope(BasePosition* position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmShoulder::updateAccelerometer(BasePosition* position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmShoulder::updateAccuracy(BasePosition* position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmShoulder::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {    
    ArmShoulderQueueParams params;    
    memcpy(&params.shoulderY, &frame.data32[0], 4);
    memcpy(&params.shoulderZ, &frame.data32[1], 4);              
    xQueueSend(ArmShoulder::queue, &params, 0);
  }  
  if (frame.id == CAN_SHOULDER_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}