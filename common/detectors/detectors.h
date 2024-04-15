#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"

enum ImuUseAngle {
  IMU_USE_ROLL,
  IMU_USE_PITCH,
  IMU_USE_YAW
};

class Range {
  public:
    const float from;
    const float to;
    Range(const float from, const float to) : 
      from(from),
      to(to) {};
};

class RangeMap {
  private:
    const float destDelta;
    const float step;
    const float sourceDelta;
    Range source;
    Range dest;
  public: 
    RangeMap(Range source, Range dest) : 
      source(source),
      dest(dest),
      destDelta(dest.to - dest.from),
      sourceDelta(source.to - source.from),
      step(destDelta / sourceDelta) {};
    const float getDestValue(const float sourceValue);
};

class Euler {
  public:
    Euler(const float roll, const float pitch, const float yaw);
    float yaw;
    float pitch;
    float roll;
    const float getRollAngle();
    const float getPitchAngle();
    const float getYawAngle();
    const float getAngle(ImuUseAngle useAngle);
};

class StructureBasic {
  public:
    float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
};

class Quaternion : public StructureBasic {
  private:
    void convertRawData();    
  public:
    uint16_t rawI;
    uint16_t rawJ;
    uint16_t rawK;
    uint16_t rawReal;
    float i;
    float j;
    float k;
    float real;
    uint8_t Q1 = 14;
    uint8_t Q2;
    uint8_t Q3;
    uint64_t serialize();
    void deserialize(uint8_t data[8]);
    Euler getEuler();
};

class Accuracy : public StructureBasic {
public:
  uint16_t quaternionRadAccuracy;
  uint8_t quaternionAccuracy;
  uint8_t accelerometerAccuracy;
  uint8_t gyroscopeAccuracy;  
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
};

class Accelerometer : public StructureBasic {
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

class Gyroscope : public StructureBasic {
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
