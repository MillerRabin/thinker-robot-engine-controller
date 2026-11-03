
#include "remoteClaw.h"

void RemoteClaw::dispatchMessage(can2040_msg frame) {
  if (frame.id == CAN_CLAW_QUATERNION) {
    Quaternion quat;
    quat.deserialize(frame.data);
    imu.quaternion.store(quat);
  }
  if (frame.id == CAN_CLAW_GYROSCOPE)
    imu.gyroscope.deserialize(frame.data);
  if (frame.id == CAN_CLAW_ACCELEROMETER)
    imu.accelerometer.deserialize(frame.data);
  if (frame.id == CAN_CLAW_ACCURACY)
    imu.accuracy.deserialize(frame.data);
  if (frame.id == CAN_CLAW_STATUSES) {
    uint64_t value;
    memcpy(&value, frame.data, sizeof(value));
    status.store(value);    
    updateTime();
  }
}

bool RemoteClaw::isOnline() {
  auto now = xTaskGetTickCount();
  // Signed: the CAN task may stamp lastUpdated after our tick read - see Bus::isBusAlive().
  return (int32_t)(now - lastUpdated) < (int32_t)maxInterval;
}

bool RemoteClaw::isPositionOK() {
  if (!isCalibrated())
    return false;
  return imu.quaternion.isFresh();
}

bool RemoteClaw::isCalibrated() {
  if (!isOnline())
    return false;  
  return (this->status.load() & ARM_CALIBRATED) != 0;
}