#include "armShoulder.h"

volatile QueueHandle_t ArmShoulder::queue;

void ArmShoulder::engineTask(void *instance) {  
  ArmShoulder* shoulder = (ArmShoulder*)instance;
  while(true) {        
    ArmShoulderQueueParams params;    
    xQueueReceive(ArmShoulder::queue, &params, portMAX_DELAY);    
    shoulder->shoulderZ.setDegree(params.shoulderZ);    
    shoulder->shoulderY.setDegree(params.shoulderY);
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
    shoulderZ(engineZPin, Range(-90, 90), Range(0, 270), IMU_USE_YAW, 100),
    shoulderY(engineYPin, Range(-180, 180), Range(0, 180), IMU_USE_PITCH, 100),
    position(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
  {            
    ArmShoulder::queue = xQueueCreate(10, sizeof(ArmShoulderQueueParams));
    xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 1024, this, tskIDLE_PRIORITY, NULL);    
}

int ArmShoulder::updateQuaternion(BasePosition* position) {
  shoulderY.euler = position->quaternion.getEuler();
  shoulderZ.euler = position->quaternion.getEuler();  
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
}