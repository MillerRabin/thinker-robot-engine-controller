#pragma once

#include "../BNO080/BNO080.h"
#include <iostream>
#include <hardware/i2c.h>
#include "../armPart/armPart.h"
#include <RP2040.h>
#ifndef PI
  #define PI					3.14159265358979f
#endif
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/binary_info.h"

class Position
{
  private:    
    const uint sdaPin;
    const uint sclPin;
    const uint intPin;
    const uint rstPin;    
  public:
    ArmPart* armPart;
    BNO080 imu;
    Quaternon rawQuat;        
    int16_t accelerometer_Q1;
    bool updateQuat(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal, uint16_t rawQuatRadianAccuracy, uint8_t quatAccuracy);
    Position(ArmPart* armPart, const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin);
};