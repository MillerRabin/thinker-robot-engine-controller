
#include "armPlatform.h"

void ArmPlatform::dispatchMessage(can2040_msg frame)
{
  if (frame.id == CAN_PLATFORM_QUATERNION)
    bno.quaternion.deserialize(frame.data);
  if (frame.id == CAN_PLATFORM_GYROSCOPE)
    bno.gyroscope.deserialize(frame.data);  
  if (frame.id == CAN_PLATFORM_ACCELEROMETER)
    bno.accelerometer.deserialize(frame.data);  
  if (frame.id == CAN_PLATFORM_ACCURACY)
    bno.accuracy.deserialize(frame.data);
}