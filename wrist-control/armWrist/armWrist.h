#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/localBNO/localBNO.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"
#include "../../common/bootsel/bootsel.h"

class ArmWristQueueParams {
  public:    
    float wristY = NAN;
    float wristZ = NAN;
};

class ArmWrist : public ArmPart {
  private:
    LocalBNO bno;
    static void busReceiverTask(void *instance);
    static void engineTask(void *instance);
    void busReceiveCallback(can2040_msg frame);
    static volatile QueueHandle_t queue;
  public:    
    Servo wristY;
    Servo wristZ;
    ArmWrist(
      uint memsSdaPin, 
      uint memsSclPin, 
      uint memsIntPin, 
      uint memsRstPin, 
      uint engineZPin, 
      uint engineYPin, 
      uint canRxPin,
      uint canTxPin
    );
    uint32_t getQuaternionMessageId() { return CAN_WRIST_QUATERNION; };
    uint32_t getAccelerometerMessageId() { return CAN_WRIST_ACCELEROMETER; };
    uint32_t getGyroscopeMessageId() { return CAN_WRIST_GYROSCOPE; };
    uint32_t getAccuracyMessageId() { return CAN_WRIST_ACCURACY; };
    uint32_t getStatusesMessageId() { return CAN_WRIST_STATUSES; };
    int updateQuaternion(BasePosition* position);
    int updateAccelerometer(BasePosition* position);
    int updateGyroscope(BasePosition* position);
    int updateAccuracy(BasePosition* position);

};