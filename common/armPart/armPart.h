#pragma once

#include "pico/stdlib.h"
#include "../servo/servo.h"
#include "../bus/bus.h"
#include "../config/config.h"

class Quaternion {
  public:
    uint16_t i;
    uint16_t j;
    uint16_t k;
    uint16_t real; 
    uint8_t Q1;
    uint8_t Q2;
    uint8_t Q3;
    uint64_t serializeData();
};

class Accuracy {
  public:    
    uint16_t quaternionRadAccuracy;
    uint8_t quaternionAccuracy;
    uint8_t accelerometerAccuracy;
    uint8_t gyroscopeAccuracy;
    uint64_t serializeData();
};

class Accelerometer {
  public:
    uint16_t x;
    uint16_t y;
    uint16_t z;    
    uint8_t Q1;
    uint8_t Q2;
    uint8_t Q3;
    uint64_t serializeData();
};

class Gyroscope {
  public:
    uint16_t x;
    uint16_t y;
    uint16_t z;    
    uint8_t Q1;
    uint8_t Q2;
    uint8_t Q3;
    uint64_t serializeData();
};

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