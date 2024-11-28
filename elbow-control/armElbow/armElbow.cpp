#include "armElbow.h"

volatile QueueHandle_t ArmElbow::queue;

void ArmElbow::engineTask(void *instance) {  
  ArmElbow* elbow = (ArmElbow*)instance;
  while(true) {        
    elbow->elbowY.tick();   
    
    Euler rEuler = elbow->platform.bno.quaternion.getEuler();    
    printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    
    Euler sEuler = elbow->position.quaternion.getEuler();      
    printf("Shoulder roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle()); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ArmElbow::busReceiverTask(void *instance) {  
  ArmElbow* elbow = (ArmElbow*)instance;
  while(true) {        
    ArmElbowQueueParams params;    
    xQueueReceive(ArmElbow::queue, &params, portMAX_DELAY);        
    elbow->elbowY.setTargetAngle(params.elbowY);
  }
}

ArmElbow::ArmElbow(
  const uint memsSdaPin, 
  const uint memsSclPin, 
  const uint memsIntPin, 
  const uint memsRstPin, 
  const uint engineZPin, 
  const uint engineYPin, 
  const uint canRxPin,
  const uint canTxPin) :
    ArmPart(canRxPin, canTxPin),    
    elbowY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    position(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
  {            
    ArmElbow::queue = xQueueCreate(10, sizeof(ArmElbowQueueParams));
    xTaskCreate(ArmElbow::busReceiverTask, "ArmElbow::busReceiverTask", 1024, this, 1, NULL);
    xTaskCreate(ArmElbow::engineTask, "ArmElbow::engineTask", 1024, this, tskIDLE_PRIORITY, NULL);
}

int ArmElbow::updateQuaternion(BasePosition* position) {  
  Euler euler = position->quaternion.getEuler();  
  //printf("Euler get pitch angle: %f\n", euler.getPitchAngle());
  elbowY.euler = euler;  
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmElbow::updateGyroscope(BasePosition* position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmElbow::updateAccelerometer(BasePosition* position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmElbow::updateAccuracy(BasePosition* position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmElbow::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {    
    ArmElbowQueueParams params;    
    memcpy(&params.elbowY, &frame.data32[0], 4);    
    xQueueSend(ArmElbow::queue, &params, 0);
  }
}