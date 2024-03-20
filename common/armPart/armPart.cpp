#include "armPart.h"

uint writeWord(uint8_t* data, uint16_t value) {
  data[0] = value & 0xff;
  data[1] = value >> 8;
  return 2;
}

uint Quaternon::serialize(uint8_t* data, uint maxLength) {
  uint i = 0;
  if (maxLength < 12) return 0;
  i+= writeWord(&data[0], i);
  i+= writeWord(&data[2], j);
  i+= writeWord(&data[4], k);
  i+= writeWord(&data[6], real);
  i+= writeWord(&data[6], quatAcc);
  i+= writeWord(&data[10], quatRadAcc);  
  return i;
}

ArmPart::ArmPart(const uint canRxPin, const uint canTxPin) {
  Bus bus = Bus(canRxPin, canTxPin);  
}

int ArmPart::sendQuaternonInternal(uint32_t id, Quaternon quat) {
  uint8_t data[12];
  uint size = quat.serialize(data, 128);
  if (size == 0) return -2;
  return bus.send(id, data, size);
}

