#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/position/positionBNO.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"

class ArmElbowQueueParams {
  public:    
    float elbowY = NAN;    
};

class ArmElbow : public ArmPart {
  private:
    Position position;
    static void busReceiverTask(void *instance);
    static void engineTask(void *instance);
    void busReceiveCallback(can2040_msg frame);
    static volatile QueueHandle_t queue;
  public:    
    Servo elbowY;
    ArmElbow(
      uint memsSdaPin, 
      uint memsSclPin, 
      uint memsIntPin, 
      uint memsRstPin, 
      uint engineYPin, 
      uint canRxPin,
      uint canTxPin
    );
    uint32_t getQuaternionMessageId() { return CAN_ELBOW_QUATERNION; };
    uint32_t getAccelerometerMessageId() { return CAN_ELBOW_ACCELEROMETER; };
    uint32_t getGyroscopeMessageId() { return CAN_ELBOW_GYROSCOPE; };
    uint32_t getAccuracyMessageId() { return CAN_ELBOW_ACCURACY; };
    int updateQuaternion(BasePosition* position);
    int updateAccelerometer(BasePosition* position);
    int updateGyroscope(BasePosition* position);
    int updateAccuracy(BasePosition* position);
};