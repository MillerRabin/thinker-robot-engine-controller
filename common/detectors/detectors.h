#pragma once

#include "pico/stdlib.h"
#include <math.h>
#include "../config/config.h"
#include <iostream>

#include "../structureBase/structureBase.h"

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
    const float sourceDelta;
    Range source;
    Range dest;
  public: 
    RangeMap(Range source, Range dest) : 
      source(source),
      dest(dest),
      destDelta(dest.to - dest.from),
      sourceDelta(source.to - source.from)
      {};
    const float getDestValue(const float sourceValue);
};


class Accuracy : public StructureBase {
public:
  uint16_t quaternionRadAccuracy;
  uint8_t quaternionAccuracy;
  uint8_t accelerometerAccuracy;
  uint8_t gyroscopeAccuracy;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
};

class Accelerometer : public StructureBase {
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

class Gyroscope : public StructureBase {
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
