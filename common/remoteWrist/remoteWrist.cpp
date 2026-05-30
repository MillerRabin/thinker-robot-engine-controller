
#include "remoteWrist.h"

void RemoteWrist::dispatchMessage(can2040_msg frame) {
  if (frame.id == CAN_WRIST_QUATERNION) {
    Quaternion quat;
    quat.deserialize(frame.data);
    imu.quaternion.store(quat);
  }
  if (frame.id == CAN_WRIST_GYROSCOPE)
    imu.gyroscope.deserialize(frame.data);
  if (frame.id == CAN_WRIST_ACCELEROMETER)
    imu.accelerometer.deserialize(frame.data);
  if (frame.id == CAN_WRIST_ACCURACY)
    imu.accuracy.deserialize(frame.data);
  if (frame.id == CAN_WRIST_STATUSES) {
    uint64_t value;
    memcpy(&value, frame.data, sizeof(value));
    status.store(value);    
    updateTime();
  }
}

bool RemoteWrist::isOnline() {
  auto now = xTaskGetTickCount();
  return (now - lastUpdated < maxInterval);
}

bool RemoteWrist::isPositionOK() {
  if (!isCalibrated())
    return false;
  return imu.quaternion.isFresh();
}

bool RemoteWrist::isCalibrated() {
  if (!isOnline())
    return false;  
  return (this->status.load() & ARM_CALIBRATED) != 0;
}