#include "pico/stdlib.h"

class Quaternion
{
public:
  uint16_t i;
  uint16_t j;
  uint16_t k;
  uint16_t real;
  uint8_t Q1;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
};

class Accuracy
{
public:
  uint16_t quaternionRadAccuracy;
  uint8_t quaternionAccuracy;
  uint8_t accelerometerAccuracy;
  uint8_t gyroscopeAccuracy;  
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
};

class Accelerometer
{
public:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t Q1;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
};

class Gyroscope
{
public:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t Q1;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
};
