
#include "remoteElbow.h"

void RemoteElbow::dispatchMessage(can2040_msg frame) {
  if (frame.id == CAN_ELBOW_QUATERNION) {
    Quaternion quat;
    quat.deserialize(frame.data);
    imu.quaternion.store(quat);
  }
  if (frame.id == CAN_ELBOW_GYROSCOPE)
    imu.gyroscope.deserialize(frame.data);
  if (frame.id == CAN_ELBOW_ACCELEROMETER)
    imu.accelerometer.deserialize(frame.data);
  if (frame.id == CAN_ELBOW_ACCURACY)
    imu.accuracy.deserialize(frame.data);
  if (frame.id == CAN_ELBOW_STATUSES) {
    uint64_t value;
    memcpy(&value, frame.data, sizeof(value));
    status.store(value);    
    updateTime();
  }
}

bool RemoteElbow::isOnline() {
  auto now = xTaskGetTickCount();
  return (now - lastUpdated < maxInterval);
}

bool RemoteElbow::isPositionOK() {
  if (!isCalibrated())
    return false;
  return imu.quaternion.isFresh();
}

bool RemoteElbow::isCalibrated() {
  if (!isOnline())
    return false;  
  return (this->status.load() & ARM_CALIBRATED) != 0;
}