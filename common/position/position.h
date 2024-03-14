#pragma once

#include "../BNO080/BNO080.h"
#include <iostream>
#include <hardware/i2c.h>
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
    BNO080 imu;
    const uint sdaPin;
    const uint sclPin;
    const uint intPin;
    const uint rstPin;    
  public:
    Position(const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin);
};