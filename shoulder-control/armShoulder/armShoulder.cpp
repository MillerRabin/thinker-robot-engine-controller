#include "armShoulder.h"

void ArmShoulder::engineTask(void *instance) {	      
  ArmShoulder* shoulder = (ArmShoulder*)instance;
  while(true) {
    printf("Set move\n");
    shoulder->shoulderZ.setDegree(135);    
    shoulder->shoulderY.setDegree(90);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
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
    BaseType_t engineTaskStatus = xTaskCreate(ArmShoulder::engineTask, "engineTask", 1024, this, 5, NULL);
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