#include "armWrist.h"

volatile QueueHandle_t ArmWrist::queue;

void ArmWrist::engineTask(void *instance) {  
  ArmWrist* wrist = (ArmWrist*)instance;
  while(true) {        
    wrist->wristY.tick();
    wrist->wristZ.tick();   
    
    Euler rEuler = wrist->platform.bno.quaternion.getEuler();    
    printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    
    Euler sEuler = wrist->position.quaternion.getEuler();      
    printf("Wrist roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle()); 
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ArmWrist::busReceiverTask(void *instance) {  
  ArmWrist* wrist = (ArmWrist*)instance;
  while(true) {        
    ArmWristQueueParams params;    
    xQueueReceive(ArmWrist::queue, &params, portMAX_DELAY);        
    wrist->wristY.setTargetAngle(params.wristY);
    wrist->wristZ.setTargetAngle(params.wristZ);
  }
}

ArmWrist::ArmWrist(
  const uint memsSdaPin, 
  const uint memsSclPin, 
  const uint memsIntPin, 
  const uint memsRstPin, 
  const uint engineZPin, 
  const uint engineYPin, 
  const uint canRxPin,
  const uint canTxPin) :
    ArmPart(canRxPin, canTxPin),    
    wristY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    wristZ(engineZPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    position(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
  {            
    ArmWrist::queue = xQueueCreate(10, sizeof(ArmWristQueueParams));
    xTaskCreate(ArmWrist::busReceiverTask, "ArmWrist::busReceiverTask", 1024, this, 1, NULL);
    xTaskCreate(ArmWrist::engineTask, "ArmWrist::engineTask", 1024, this, tskIDLE_PRIORITY, NULL);
}

int ArmWrist::updateQuaternion(BasePosition* position) {  
  Euler euler = position->quaternion.getEuler();  
  //printf("Euler get pitch angle: %f\n", euler.getPitchAngle());
  wristY.euler = euler;
  wristZ.euler = euler;
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmWrist::updateGyroscope(BasePosition* position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmWrist::updateAccelerometer(BasePosition* position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmWrist::updateAccuracy(BasePosition* position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmWrist::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {    
    ArmWristQueueParams params;    
    memcpy(&params.wristY, &frame.data32[0], 4);
    memcpy(&params.wristZ, &frame.data32[1], 4);
    xQueueSend(ArmWrist::queue, &params, 0);
  }
}