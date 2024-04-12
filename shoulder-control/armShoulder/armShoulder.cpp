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
    shoulderZ(engineZPin, 270, 100),
    shoulderY(engineYPin, 180, 100),
    position(this, memsSdaPin, memsSclPin, memsIntPin, memsRstPin)
  {        
    ArmShoulder::queue = xQueueCreate(10, sizeof(ArmShoulderQueueParams));
    xTaskCreate(ArmShoulder::engineTask, "ArmShoulder::engineTask", 1024, this, 5, NULL);
}

int ArmShoulder::sendQuaternion(Quaternion quat) {
  return sendQuaternionInternal(CAN_SHOULDER_QUATERNION, quat);
}

int ArmShoulder::sendGyroscope(Gyroscope gyro) {
  return sendGyroscopeInternal(CAN_SHOULDER_GYROSCOPE, gyro);
}

int ArmShoulder::sendAccelerometer(Accelerometer acc) {
  return sendAccelerometerInternal(CAN_SHOULDER_ACCELEROMETER, acc);
}

int ArmShoulder::sendAccuracy(Accuracy acc) {
  return sendAccuracyInternal(CAN_SHOULDER_ACCURACY, acc);
}

void ArmShoulder::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_SHOULDER_SET_YZ_DEGREE) {    
    ArmShoulderQueueParams params;    
    memcpy(&params.shoulderY, &frame.data32[0], 4);
    memcpy(&params.shoulderZ, &frame.data32[1], 4);              
    xQueueSend(ArmShoulder::queue, &params, 0);
  }
}