cmake_minimum_required(VERSION 3.14)

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_CLAW_NAME}")
add_compile_definitions(APP_VERSION="${APP_CLAW_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# Include app source code file(s)
add_executable(${APP_CLAW_NAME}
  ${APP_CLAW_SRC_DIRECTORY}/main.cpp
  ${APP_CLAW_SRC_DIRECTORY}/armClaw/armClaw.cpp
  ${APP_COMMON_DIRECTORY}/canBus/src/can2040.c
  ${APP_COMMON_DIRECTORY}/witmotion/wit_c_sdk.c
  ${APP_COMMON_DIRECTORY}/localWitmotion/localWitmotion.cpp
  ${APP_COMMON_DIRECTORY}/remoteBNO/remoteBNO.cpp    
  ${APP_COMMON_DIRECTORY}/servo/servo.cpp
  ${APP_COMMON_DIRECTORY}/flashSettings/flashSettings.cpp
  ${APP_COMMON_DIRECTORY}/bus/bus.cpp  
  ${APP_COMMON_DIRECTORY}/armPart/armPart.cpp  
  ${APP_COMMON_DIRECTORY}/structureBase/structureBase.cpp
  ${APP_COMMON_DIRECTORY}/euler/euler.cpp
  ${APP_COMMON_DIRECTORY}/quaternion/quaternion.cpp
  ${APP_COMMON_DIRECTORY}/detectors/detectors.cpp
  ${APP_COMMON_DIRECTORY}/armPlatform/armPlatform.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/Pico_VL53L0X.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_calibration.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_core.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_ranging.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/core/src/vl53l0x_api_strings.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/core/src/vl53l0x_api.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/platform/src/vl53l0x_i2c_comms.cpp
  ${APP_COMMON_DIRECTORY}/VL53L0X/platform/src/vl53l0x_platform.cpp
  ${APP_COMMON_DIRECTORY}/VL6180x/VL6180x.cpp
  ${APP_COMMON_DIRECTORY}/rangeDetector/rangeDetector.cpp
  ${APP_COMMON_DIRECTORY}/measureRange/measureRange.cpp
  ${APP_COMMON_DIRECTORY}/bootsel/bootsel.cpp
)

# Link to built libraries
target_link_libraries(${APP_CLAW_NAME} LINK_PUBLIC
  pico_stdlib
  hardware_i2c
  hardware_pwm
  hardware_flash
  hardware_sync
  FreeRTOS
)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_CLAW_NAME} 1)
pico_enable_stdio_uart(${APP_CLAW_NAME} 0)

# Enable extra build products
pico_add_extra_outputs(${APP_CLAW_NAME})
