#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/position/positionWitmotion.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"
#include "../../common/rangeDetector/rangeDetector.h"

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
    RangeDetector rangeDetector;
  public:    
    Servo clawX;
    Servo clawY;
    Servo clawZ;
    Servo clawGripper;
    ArmClaw(
      const uint8_t detectorsSdaPin, 
      const uint8_t detectorsSclPin, 
      const uint8_t engineXPin, 
      const uint8_t engineYPin, 
      const uint8_t engineZPin,
      const uint8_t engineGripperPin,
      const uint8_t canRxPin,
      const uint8_t canTxPin,
      const uint8_t memsRxPin,
      const uint8_t memsTxPin,
      const uint8_t memsRstPin,
      const uint8_t memsIntPin,
      const uint8_t shortDetectorShutPin,
      const uint8_t longDetectorShutPin
    );

    uint32_t getQuaternionMessageId() { return CAN_CLAW_QUATERNION; };
    uint32_t getAccelerometerMessageId() { return CAN_CLAW_ACCELEROMETER; };
    uint32_t getGyroscopeMessageId() { return CAN_CLAW_GYROSCOPE; };    
    uint32_t getHeightMessageId() { return CAN_CLAW_HEIGHT; };
    int updateQuaternion(BasePosition* position);
    int updateAccelerometer(BasePosition* position);
    int updateGyroscope(BasePosition* position);
    int updateAccuracy(BasePosition* position);
};