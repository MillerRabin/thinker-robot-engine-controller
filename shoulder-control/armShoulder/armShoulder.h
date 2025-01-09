#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/position/positionBNO.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"

class ArmShoulderQueueParams {
  public:    
    float shoulderY = NAN;
    float shoulderZ = NAN;
};

class ArmShoulder : public ArmPart {
  private:
    Position position;
    static void busReceiverTask(void *instance);
    static void engineTask(void *instance);
    void busReceiveCallback(can2040_msg frame);
    static volatile QueueHandle_t queue;
  public:
    Servo shoulderZ;
    Servo shoulderY;
    ArmShoulder(
      uint memsSdaPin, 
      uint memsSclPin, 
      uint memsIntPin, 
      uint memsRstPin, 
      uint engineZPin, 
      uint engineYPin, 
      uint canRxPin,
      uint canTxPin
    );
    uint32_t getQuaternionMessageId() { return CAN_SHOULDER_QUATERNION; };
    uint32_t getAccelerometerMessageId() { return CAN_SHOULDER_ACCELEROMETER; };
    uint32_t getGyroscopeMessageId() { return CAN_SHOULDER_GYROSCOPE; };
    uint32_t getAccuracyMessageId() { return CAN_SHOULDER_ACCURACY; };
    int updateQuaternion(BasePosition* position);
    int updateAccelerometer(BasePosition* position);
    int updateGyroscope(BasePosition* position);
    int updateAccuracy(BasePosition* position);
};