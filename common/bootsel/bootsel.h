#pragma once
#include <stdio.h>

extern "C" {
    #include "pico/bootrom.h"
}

void rebootInBootMode();