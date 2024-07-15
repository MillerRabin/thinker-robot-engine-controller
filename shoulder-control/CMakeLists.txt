cmake_minimum_required(VERSION 3.14)

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_SHOULDER_NAME}")
add_compile_definitions(APP_VERSION="${APP_SHOULDER_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# Include app source code file(s)
add_executable(${APP_SHOULDER_NAME}
  ${APP_SHOULDER_SRC_DIRECTORY}/main.cpp
  ${APP_SHOULDER_SRC_DIRECTORY}/armShoulder/armShoulder.cpp
  ${APP_COMMON_DIRECTORY}/canBus/src/can2040.c
  ${APP_COMMON_DIRECTORY}/BNO080/BNO080.cpp
  ${APP_COMMON_DIRECTORY}/remoteBNO/remoteBNO.cpp    
  ${APP_COMMON_DIRECTORY}/servo/servo.cpp
  ${APP_COMMON_DIRECTORY}/position/position.cpp
  ${APP_COMMON_DIRECTORY}/bus/bus.cpp  
  ${APP_COMMON_DIRECTORY}/armPart/armPart.cpp  
  ${APP_COMMON_DIRECTORY}/structureBase/structureBase.cpp
  ${APP_COMMON_DIRECTORY}/euler/euler.cpp
  ${APP_COMMON_DIRECTORY}/quaternion/quaternion.cpp
  ${APP_COMMON_DIRECTORY}/detectors/detectors.cpp
  ${APP_COMMON_DIRECTORY}/armPlatform/armPlatform.cpp  
)

# Link to built libraries
target_link_libraries(${APP_SHOULDER_NAME} LINK_PUBLIC
  pico_stdlib
  hardware_i2c
  hardware_pwm
  FreeRTOS
)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_SHOULDER_NAME} 1)
pico_enable_stdio_uart(${APP_SHOULDER_NAME} 0)

# Enable extra build products
pico_add_extra_outputs(${APP_SHOULDER_NAME})
