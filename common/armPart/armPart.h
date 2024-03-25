#pragma once

#include "pico/stdlib.h"
#include "../servo/servo.h"
#include "../bus/bus.h"
#include "../config/config.h"
#include "../detectors/detectors.h"

class ArmPart {
  private:
    Bus bus;
  protected:
    int sendQuaternionInternal(uint32_t id, Quaternion quat);
    int sendGyroscopeInternal(uint32_t id, Gyroscope gyro);
    int sendAccelerometerInternal(uint32_t id, Accelerometer acc);
    int sendAccuracyInternal(uint32_t id, Accuracy acc);
  public:    
    virtual int sendQuaternion(Quaternion quat) { return 1; };
    virtual int sendAccelerometer(Accelerometer acc) { return 1; };
    virtual int sendGyroscope(Gyroscope gyro) { return 1; };
    virtual int sendAccuracy(Accuracy acc) { return 1; };
    ArmPart(const uint canRxPin, const uint canTxPin);    
};