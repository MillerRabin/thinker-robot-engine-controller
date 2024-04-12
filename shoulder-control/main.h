
#pragma once

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
// CXX
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
// Pico SDK
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "armShoulder/armShoulder.h"
#include <math.h>
#include <hardware/pwm.h>
#include "../common/config/config.h" 

using std::string;

#ifdef __cplusplus
extern "C"
{
#endif

  

#ifdef __cplusplus
} // extern "C"
#endif
