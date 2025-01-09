#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/position/positionWitmotion.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"

class ArmClawQueueParams {
  public:
    void set(uint8_t data[]);
    float clawY = NAN;
    float clawZ = NAN;
    float clawX = NAN;
    float clawGripper = NAN;
};

class ArmClaw : public ArmPart {
  private:
    PositionWitMotion position;
    static void busReceiverTask(void *instance);
    static void engineTask(void *instance);
    void busReceiveCallback(can2040_msg frame);
    static volatile QueueHandle_t queue;
  public:    
    Servo clawX;
    Servo clawY;
    Servo clawZ;
    Servo clawGripper;
    ArmClaw(
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
    );

    uint32_t getQuaternionMessageId() { return CAN_CLAW_QUATERNION; };
    uint32_t getAccelerometerMessageId() { return CAN_CLAW_ACCELEROMETER; };
    uint32_t getGyroscopeMessageId() { return CAN_CLAW_GYROSCOPE; };
    uint32_t getAccuracyMessageId() { return CAN_CLAW_ACCURACY; };
    int updateQuaternion(BasePosition* position);
    int updateAccelerometer(BasePosition* position);
    int updateGyroscope(BasePosition* position);
    int updateAccuracy(BasePosition* position);
};