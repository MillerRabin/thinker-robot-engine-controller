#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/position/position.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"
#include "../../common/i2cScan/i2cScan.h"
#include "../../common/bmp280/bmp280.h"
#include "../../common/mpu6500/mpu6500.h"

class ArmElbowQueueParams {
  public:    
    float elbowY = NAN;    
};

class ArmElbow : public ArmPart {
  private:        
    static void busReceiverTask(void *instance);
    static void engineTask(void *instance);
    void busReceiveCallback(can2040_msg frame);
    static volatile QueueHandle_t queue;    
    static BMP280 bmp;
    static MPU6500 mpu;
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