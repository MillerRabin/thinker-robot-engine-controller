
#include "remoteShoulder.h"

void RemoteShoulder::dispatchMessage(can2040_msg frame)
{
  if (frame.id == CAN_SHOULDER_QUATERNION)
    imu.quaternion.deserialize(frame.data);
  if (frame.id == CAN_SHOULDER_QUATERNION)
    imu.gyroscope.deserialize(frame.data);  
  if (frame.id == CAN_SHOULDER_QUATERNION)
    imu.accelerometer.deserialize(frame.data);
  if (frame.id == CAN_SHOULDER_QUATERNION)
    imu.accuracy.deserialize(frame.data);
}