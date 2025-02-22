#pragma once

#include "pico/stdlib.h"
#include "../servo/servo.h"
#include "../bus/bus.h"
#include "../config/config.h"
#include "../detectors/detectors.h"
#include "../measureRange/measureRange.h"
#include "../armPlatform/armPlatform.h"

class BasePosition {
  public:
    Quaternion quaternion;
    Accuracy accuracy;
    Gyroscope gyroscope;
    Accelerometer accelerometer;
    uint32_t height;
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
    int updateHeight(uint32_t height);    
  public:
    ArmPlatform platform; 
    virtual uint32_t getQuaternionMessageId() { return 0; };
    virtual uint32_t getAccelerometerMessageId() { return 0; };
    virtual uint32_t getGyroscopeMessageId() { return 0; };
    virtual uint32_t getAccuracyMessageId() { return 0; };
    virtual uint32_t getHeightMessageId() { return 0; };
    virtual uint32_t getRangeMessageId() { return 0; };
    virtual int updateQuaternion(BasePosition* position) { return 0; };
    virtual int updateAccelerometer(BasePosition* position) { return 0; };
    virtual int updateGyroscope(BasePosition* position) { return 0; };
    virtual int updateAccuracy(BasePosition* position) { return 0; };
    virtual int updateHeight(BasePosition* position) { return 0; };  
    virtual int updateRange(uint16_t range, uint16_t measureType);
    ArmPart(const uint canRxPin, const uint canTxPin);
};

