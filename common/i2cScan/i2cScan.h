#pragma once

#include <cstdint>
#include <hardware/i2c.h>
#include <iostream>

class I2CScan {
  private: 
    static bool reserved_addr(uint8_t addr);
  public:
    static void scan(i2c_inst_t* bus);
};