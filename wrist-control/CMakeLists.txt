cmake_minimum_required(VERSION 3.14)

# ==============================
# Compile definitions
# ==============================

add_compile_definitions(APP_NAME="${APP_WRIST_NAME}")
add_compile_definitions(APP_VERSION="${APP_WRIST_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# ==============================
# Sources
# ==============================

file(GLOB_RECURSE APP_WRIST_SOURCES CONFIGURE_DEPENDS
    "${APP_WRIST_SRC_DIRECTORY}/*.cpp"
    "${APP_COMMON_DIRECTORY}/*.cpp"
    "${APP_COMMON_DIRECTORY}/*.c"
)

list(FILTER APP_WRIST_SOURCES EXCLUDE REGEX ".*/example/.*")
list(FILTER APP_WRIST_SOURCES EXCLUDE REGEX ".*/examples/.*")
list(FILTER APP_WRIST_SOURCES EXCLUDE REGEX ".*/test/.*")
list(FILTER APP_WRIST_SOURCES EXCLUDE REGEX ".*/tests/.*")
list(FILTER APP_WRIST_SOURCES EXCLUDE REGEX ".*/mock/.*")

# ==============================
# Executable
# ==============================

add_executable(${APP_WRIST_NAME}
    ${APP_WRIST_SOURCES}
)

# ==============================
# Include directories
# ==============================

target_include_directories(${APP_WRIST_NAME} PRIVATE
    ${FREERTOS_CFG_DIRECTORY}
    ${APP_COMMON_DIRECTORY}
)

# ==============================
# Link libraries
# ==============================

target_link_libraries(${APP_WRIST_NAME}
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_pwm
    hardware_flash
    hardware_sync
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# ==============================
# Pico settings
# ==============================

pico_enable_stdio_usb(${APP_WRIST_NAME} 1)
pico_enable_stdio_uart(${APP_WRIST_NAME} 0)

pico_add_extra_outputs(${APP_WRIST_NAME})