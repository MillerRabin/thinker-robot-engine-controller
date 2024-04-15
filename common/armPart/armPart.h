#pragma once

#include "pico/stdlib.h"
#include "../servo/servo.h"
#include "../bus/bus.h"
#include "../config/config.h"
#include "../detectors/detectors.h"

class BasePosition {
  public:
    Quaternion quaternion;
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
};

class ArmPart {
  private:
    Bus bus;
    static void canCallback(void* pArmPart, can2040_msg frame);
  protected:    
    virtual void busReceiveCallback(can2040_msg frame) {};
    int updateQuaternion(Quaternion quat);
    int updateAccelerometer(Accelerometer acc);
    int updateGyroscope(Gyroscope gyro);
    int updateAccuracy(Accuracy acc);
  public:    
    virtual uint32_t getQuaternionMessageId() { return 0; };
    virtual uint32_t getAccelerometerMessageId() { return 0; };
    virtual uint32_t getGyroscopeMessageId() { return 0; };
    virtual uint32_t getAccuracyMessageId() { return 0; };
    virtual int updateQuaternion(BasePosition* position) { return 0; };
    virtual int updateAccelerometer(BasePosition* position) { return 0; };
    virtual int updateGyroscope(BasePosition* position) { return 0; };
    virtual int updateAccuracy(BasePosition* position) { return 0; };
    ArmPart(const uint canRxPin, const uint canTxPin);
};

