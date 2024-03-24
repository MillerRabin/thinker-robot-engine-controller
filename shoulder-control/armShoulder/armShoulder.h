#pragma once

#include "pico/stdlib.h"
#include "../../common/servo/servo.h"
#include "../../common/position/position.h"
#include "../../common/bus/bus.h"
#include "../../common/config/config.h"
#include "../../common/armPart/armPart.h"

class ArmShoulder : private ArmPart {
  private:
    Position position;
    static void engineTask(void *instance);
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
    int sendQuaternion(Quaternion quat);
    int sendAccelerometer(Accelerometer acc);
    int sendGyroscope(Gyroscope gyro);
    int sendAccuracy(Accuracy acc);
};