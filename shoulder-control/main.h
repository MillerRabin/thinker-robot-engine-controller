#ifndef MAIN_H
#define MAIN_H

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

using std::string;

#ifdef __cplusplus
extern "C" {
#endif

    void log_debug(const char* msg);

#ifdef __cplusplus
}           // extern "C"
#endif


#endif      // MAIN_H
