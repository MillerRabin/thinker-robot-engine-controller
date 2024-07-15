#include "structureBase.h"

float StructureBase::qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return qFloat;
}

uint16_t StructureBase::floatToQ(float q, uint8_t qPoint){
  return (int16_t) (q * pow(2, qPoint));
}
