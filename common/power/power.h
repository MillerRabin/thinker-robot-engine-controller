#pragma once

#include "pico/stdlib.h"

class Power {
private:
  uint16_t voltage;
  uint16_t current;  
public:
  Power() : voltage(0), current(0) {}
  Power(uint16_t voltage, uint16_t current) : voltage(voltage), current(current) {}
  Power(float voltage, float current) : voltage(voltage * 1000.0f), current(current * 1000.0f) {}
  float getVoltage() const { return voltage / 1000.0f; }
  float getCurrent() const { return current / 1000.0f;    
  }
  
  uint32_t serialize() const;
  void deserialize(uint8_t data[4]);
  void set(uint16_t voltage, uint16_t current) {
    this->voltage = voltage;
    this->current = current;    
  }
  void set(float voltage, float current) {
    this->voltage = voltage * 1000.0f;
    this->current = current * 1000.0f;    
  }   
  float getPower() const { return getVoltage() * getCurrent(); }
};