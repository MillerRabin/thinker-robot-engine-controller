cmake_minimum_required(VERSION 3.14)

set(FREERTOS_KERNEL_PATH "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel")


set(PROJECT_NAME "THINKER_ROBOT_ENGINE_CONTROL")
set(BUILD_NUMBER "1")

set(APP_SHOULDER_NAME "ENGINE_CONTROL_SHOULDER")
set(APP_SHOULDER_VERSION_NUMBER "0.0.1")
set(APP_ELBOW_NAME "ENGINE_CONTROL_ELBOW")
set(APP_ELBOW_VERSION_NUMBER "0.0.1")
set(APP_WRIST_NAME "ENGINE_CONTROL_WRIST")
set(APP_WRIST_VERSION_NUMBER "0.0.1")
set(APP_CLAW_NAME "ENGINE_CONTROL_CLAW")
set(APP_CLAW_VERSION_NUMBER "0.0.1")

set(APP_SHOULDER_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/shoulder-control")
set(APP_ELBOW_SRC_DIRECTORY    "${CMAKE_SOURCE_DIR}/elbow-control")
set(APP_WRIST_SRC_DIRECTORY    "${CMAKE_SOURCE_DIR}/wrist-control")
set(APP_CLAW_SRC_DIRECTORY     "${CMAKE_SOURCE_DIR}/claw-control")
set(APP_COMMON_DIRECTORY       "${CMAKE_SOURCE_DIR}/common")
set(FREERTOS_CFG_DIRECTORY     "${CMAKE_SOURCE_DIR}/Config")

add_compile_definitions($<$<CONFIG:Debug>:DEBUG=1>)

set(FREERTOS_ENABLE_TIMERS 1)

# Pico SDK path
set(ENV{PICO_SDK_PATH} "${CMAKE_SOURCE_DIR}/pico-sdk")

include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(${PROJECT_NAME}
    LANGUAGES C CXX ASM
    DESCRIPTION "Engine control for robotic arm"
)

pico_sdk_init()

# can2040 must react to its PIO irq within ~7 bit times (7 us at 1 Mbit/s) or it misses the ack slot,
# and a few XIP flash cache misses already cost that (canBus/docs/API.md, "Low interrupt latency").
# The whole binary doesn't fit in RAM next to the 192 KB FreeRTOS heap, so only can2040.c moves: this
# is the SDK's default script with can2040.c's object excluded from the flash .text/.rodata, which
# drops it into the RAM .data section's *(.text*) / *(.rodata*) catch-alls.
set(SDK_MEMMAP_LD "${PICO_SDK_PATH}/src/rp2_common/pico_crt0/rp2040/memmap_default.ld")
set(CAN2040_RAM_LINKER_SCRIPT "${CMAKE_BINARY_DIR}/memmap_can2040_ram.ld")
set(FLASH_EXCLUDES "*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:)")
file(READ ${SDK_MEMMAP_LD} MEMMAP_LD)
string(FIND "${MEMMAP_LD}" "${FLASH_EXCLUDES} .text*)" TEXT_POS)
string(FIND "${MEMMAP_LD}" "${FLASH_EXCLUDES} .rodata*)" RODATA_POS)
if(TEXT_POS EQUAL -1 OR RODATA_POS EQUAL -1)
    message(FATAL_ERROR "${SDK_MEMMAP_LD} changed: can2040 can no longer be moved to RAM this way")
endif()
string(REPLACE "*libm.a:)" "*libm.a: *can2040.c.o*)" MEMMAP_LD "${MEMMAP_LD}")
file(CONFIGURE OUTPUT ${CAN2040_RAM_LINKER_SCRIPT} CONTENT "${MEMMAP_LD}" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${SDK_MEMMAP_LD})
# The irq path still zeroes its parse state through the SDK's memset wrapper - keep that in RAM too
add_compile_definitions(PICO_MEM_IN_RAM=1)

add_subdirectory(${APP_SHOULDER_SRC_DIRECTORY})
add_subdirectory(${APP_ELBOW_SRC_DIRECTORY})
add_subdirectory(${APP_WRIST_SRC_DIRECTORY})
add_subdirectory(${APP_CLAW_SRC_DIRECTORY})