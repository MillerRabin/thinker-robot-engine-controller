cmake_minimum_required(VERSION 3.14)

# Set project data
set(PROJECT_NAME "THINKER_ROBOT_ENGINE_CONTROL")
set(BUILD_NUMBER "1")

# Set app name(s) and version(s)
set(APP_SHOULDER_NAME "ENGINE_CONTROL_SHOULDER")
set(APP_SHOULDER_VERSION_NUMBER "0.0.1")

set(APP_ELBOW_NAME "ENGINE_CONTROL_ELBOW")
set(APP_ELBOW_VERSION_NUMBER "0.0.1")

set(APP_WRIST_NAME "ENGINE_CONTROL_WRIST")
set(APP_WRIST_VERSION_NUMBER "0.0.1")

# Specify the app(s) source code
set(APP_SHOULDER_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/shoulder-control")
set(APP_ELBOW_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/elbow-control")
set(APP_WRIST_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/wrist-control")

set(APP_COMMON_DIRECTORY "${CMAKE_SOURCE_DIR}/common")

# FROM 1.4.2 -- Set app-side debugging on debug builds
# Now uses CMake generator expression to extract config type
add_compile_definitions($<$<CONFIG:Debug>:DEBUG=1>)

# Set env variable 'PICO_SDK_PATH' to the local Pico SDK
# Comment out the set() if you have a global copy of the
# SDK set and $PICO_SDK_PATH defined in your $PATH
set(ENV{PICO_SDK_PATH} "${CMAKE_SOURCE_DIR}/pico-sdk")

# Set flags and directory variables
set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}/Config")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel")

# Include the Pico SDK
include(pico_sdk_import.cmake)

# Name the project
project(${PROJECT_NAME}
        LANGUAGES C CXX ASM
        DESCRIPTION "Engine control for robotic arm"
)

# Initialise the Pico SDK
pico_sdk_init()

# Add FreeRTOS as a library
add_library(FreeRTOS STATIC
    ${FREERTOS_SRC_DIRECTORY}/event_groups.c
    ${FREERTOS_SRC_DIRECTORY}/list.c
    ${FREERTOS_SRC_DIRECTORY}/queue.c
    ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
    ${FREERTOS_SRC_DIRECTORY}/tasks.c
    ${FREERTOS_SRC_DIRECTORY}/timers.c    
    ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
)

# Build FreeRTOS
target_include_directories(FreeRTOS PUBLIC
    ${FREERTOS_CFG_DIRECTORY}/
    ${FREERTOS_SRC_DIRECTORY}/include
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0
)

include_directories(RP2040
    ${CMAKE_SOURCE_DIR}/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include
    ${CMAKE_SOURCE_DIR}/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Core/Include    
)



# Include the apps' source code
add_subdirectory(${APP_SHOULDER_SRC_DIRECTORY})
add_subdirectory(${APP_ELBOW_SRC_DIRECTORY})
add_subdirectory(${APP_WRIST_SRC_DIRECTORY})
