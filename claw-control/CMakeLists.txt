cmake_minimum_required(VERSION 3.14)

# ==============================
# Compile definitions
# ==============================

add_compile_definitions(APP_NAME="${APP_CLAW_NAME}")
add_compile_definitions(APP_VERSION="${APP_CLAW_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# ==============================
# Sources
# ==============================

file(GLOB_RECURSE APP_CLAW_SOURCES CONFIGURE_DEPENDS
    "${APP_CLAW_SRC_DIRECTORY}/*.cpp"
    "${APP_COMMON_DIRECTORY}/*.cpp"
    "${APP_COMMON_DIRECTORY}/*.c"
)

list(FILTER APP_CLAW_SOURCES EXCLUDE REGEX ".*/example/.*")
list(FILTER APP_CLAW_SOURCES EXCLUDE REGEX ".*/examples/.*")
list(FILTER APP_CLAW_SOURCES EXCLUDE REGEX ".*/test/.*")
list(FILTER APP_CLAW_SOURCES EXCLUDE REGEX ".*/tests/.*")
list(FILTER APP_CLAW_SOURCES EXCLUDE REGEX ".*/mock/.*")

# ==============================
# Executable
# ==============================

add_executable(${APP_CLAW_NAME}
    ${APP_CLAW_SOURCES}
)

# ==============================
# Include directories
# ==============================

target_include_directories(${APP_CLAW_NAME} PRIVATE
    ${FREERTOS_CFG_DIRECTORY}
    ${APP_COMMON_DIRECTORY}
)

# ==============================
# Link libraries
# ==============================

target_link_libraries(${APP_CLAW_NAME}
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

pico_enable_stdio_usb(${APP_CLAW_NAME} 1)
pico_enable_stdio_uart(${APP_CLAW_NAME} 0)
pico_add_extra_outputs(${APP_CLAW_NAME})