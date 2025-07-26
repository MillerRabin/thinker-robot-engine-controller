
#include "armPlatform.h"

void ArmPlatform::dispatchMessage(can2040_msg frame)
{
  if (frame.id == CAN_PLATFORM_QUATERNION)
    imu.quaternion.deserialize(frame.data);
  if (frame.id == CAN_PLATFORM_GYROSCOPE)
    imu.gyroscope.deserialize(frame.data);  
  if (frame.id == CAN_PLATFORM_ACCELEROMETER)
    imu.accelerometer.deserialize(frame.data);  
  if (frame.id == CAN_PLATFORM_ACCURACY)
    imu.accuracy.deserialize(frame.data);
}