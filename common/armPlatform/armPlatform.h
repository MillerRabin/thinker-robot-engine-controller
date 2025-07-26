#pragma once

#include "pico/stdlib.h"
#include "../../common/bus/bus.h"
#include "../../common/remoteBNO/remoteBNO.h"
#include "../../common/config/config.h"

class ArmPlatform {
  public:
    RemoteBNO imu;
    void dispatchMessage(can2040_msg frame);
};