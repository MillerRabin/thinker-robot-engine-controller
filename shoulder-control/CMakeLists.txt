cmake_minimum_required(VERSION 3.14)

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_SHOULDER_NAME}")
add_compile_definitions(APP_VERSION="${APP_SHOULDER_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# Include app source code file(s)
add_executable(${APP_SHOULDER_NAME}
    ${APP_SHOULDER_SRC_DIRECTORY}/main.cpp    
    ${APP_SHOULDER_SRC_DIRECTORY}/wit_c_sdk.c
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/Pico_VL53L0X.cpp
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_calibration.cpp
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_core.cpp
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_ranging.cpp
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_strings.cpp
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/core/src/vl53l0x_api.cpp
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/platform/src/vl53l0x_i2c_comms.cpp
    ${APP_SHOULDER_SRC_DIRECTORY}/VL53L0X/platform/src/vl53l0x_platform.cpp    
)

# Link to built libraries
target_link_libraries(${APP_SHOULDER_NAME} LINK_PUBLIC
    pico_stdlib
    hardware_i2c
    FreeRTOS
)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_SHOULDER_NAME} 1)
pico_enable_stdio_uart(${APP_SHOULDER_NAME} 0)

# Enable extra build products
pico_add_extra_outputs(${APP_SHOULDER_NAME})
