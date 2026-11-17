
#include "remoteElbow.h"

float RemoteElbow::getSettleSpeed() const {
  if (time_us_32() - settleSpeedAtUs > SETTLE_SPEED_STALE_MS * 1000u) {
    return 0.0f;
  }
  return settleSpeed;
}

void RemoteElbow::dispatchMessage(can2040_msg frame) {
  if (frame.id == CAN_ELBOW_SETTLE_SPEED) {
    float v;
    memcpy(&v, frame.data, sizeof(float));
    settleSpeed = isnan(v) ? 0.0f : v;
    settleSpeedAtUs = time_us_32();
    return;
  }
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
  // Signed: the CAN task may stamp lastUpdated after our tick read - see Bus::isBusAlive().
  return (int32_t)(now - lastUpdated) < (int32_t)maxInterval;
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