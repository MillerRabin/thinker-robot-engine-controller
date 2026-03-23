#pragma once

#include "../qBase/qBase.h"

class Accuracy : public QBase {
public:
  uint16_t quaternionRadAccuracy;
  uint8_t quaternionAccuracy;
  uint8_t accelerometerAccuracy;
  uint8_t gyroscopeAccuracy;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
};
