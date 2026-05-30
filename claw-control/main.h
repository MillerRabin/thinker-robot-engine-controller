
#pragma once

// FreeRTOS
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>
// CXX
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// Pico SDK
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "../common/config/config.h"
#include "../common/logQueue/logQueue.h"
#include "armClaw/armClaw.h"
#include <hardware/pwm.h>
#include <math.h>

using std::string;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} // extern "C"
#endif
