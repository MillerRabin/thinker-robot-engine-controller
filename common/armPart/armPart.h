#pragma once

#include "pico/stdlib.h"
#include "../servo/servo.h"
#include "../bus/bus.h"
#include "../config/config.h"

class Quaternon {
  public:
    uint16_t i;
    uint16_t j;
    uint16_t k;
    uint16_t real; 
    uint16_t quatRadAcc;
    uint16_t quatAcc;
    uint8_t Q1;
    uint8_t Q2;
    uint8_t Q3;
  uint serialize(uint8_t* data, uint maxLength);
};

class ArmPart {
  private:
    Bus bus;
  protected:
    int sendQuaternonInternal(uint32_t id, Quaternon quat);
  public:    
    virtual int sendQuaternon(Quaternon quat) { return 0; };
    ArmPart(const uint canRxPin, const uint canTxPin);    
};