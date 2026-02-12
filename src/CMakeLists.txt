cmake_minimum_required(VERSION 3.25)

# Set project data
set(PROJECT_NAME "OpenTricklerController")
set(TARGET_NAME "OpenTrickler_2.9.0_RP2040")

message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

# Set env variable 'PICO_SDK_PATH' to the local Pico SDK
# Comment out the set() if you have a global copy of the
# SDK set and $PICO_SDK_PATH defined in your $PATH
set(ENV{PICO_SDK_PATH} "${CMAKE_SOURCE_DIR}/library/pico-sdk")
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

# Target board - can be overridden with -DPICO_BOARD=xxx
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico_w)
endif()

# Include the Pico SDK
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Name the project
project(${PROJECT_NAME}
        LANGUAGES C CXX ASM
        DESCRIPTION "Firmware for OpenTrickler Controller"
)

# Initialise the Pico SDK
pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Set flags and directory variables
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Os")
# Source files are at root level in v1.17
set(SRC_DIRECTORY "${CMAKE_SOURCE_DIR}")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/library/FreeRTOS-Kernel")
set(U8G2_SRC_DIRECYTORY "${CMAKE_SOURCE_DIR}/library/u8g2")
set(U8G2_MUI_DIRECTORY "${U8G2_SRC_DIRECYTORY}/csrc")
set(TMC_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/library/Trinamic-library")
set(SCRIPTS_DIRECTORY "${CMAKE_SOURCE_DIR}/scripts")
set(PICO_BOARD_HEADER_DIRS "${CMAKE_SOURCE_DIR}/targets")

# Append compiler flags
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections -Wall")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fdata-sections -ffunction-sections -Wall")
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# Note: RP2350 FPU is automatically enabled by pico-sdk for rp2350-arm-s platform
# The SDK sets proper float ABI and FPU flags. We avoid setting -mfloat-abi=hard here
# because it would conflict with libraries (u8g2) that don't use the same ABI.


# Application
add_executable("${TARGET_NAME}")

# Include source
include_directories(${SRC_DIRECTORY})

# Pull in FreeRTOS - select correct port based on platform (RP2040 vs RP2350)
# PICO_PLATFORM is set by pico_sdk_init() based on PICO_BOARD
if ("${PICO_PLATFORM}" STREQUAL "rp2040")
    message(STATUS "Using FreeRTOS port for RP2040")
    include(${FREERTOS_SRC_DIRECTORY}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
elseif("${PICO_PLATFORM}" STREQUAL "rp2350-arm-s")
    message(STATUS "Using FreeRTOS port for RP2350 ARM (Non-TrustZone)")
    include(${FREERTOS_SRC_DIRECTORY}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)
elseif("${PICO_PLATFORM}" STREQUAL "rp2350-riscv")
    message(STATUS "Using FreeRTOS port for RP2350 RISC-V")
    include(${FREERTOS_SRC_DIRECTORY}/portable/ThirdParty/GCC/RP2350_RISC-V/FreeRTOS_Kernel_import.cmake)
else()
    message(STATUS "Unknown platform '${PICO_PLATFORM}', defaulting to RP2040 FreeRTOS port")
    include(${FREERTOS_SRC_DIRECTORY}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
endif()

# Pull in u8g2
add_subdirectory(${U8G2_SRC_DIRECYTORY})

# Pull in Trinamic library (we don't use CMakeLists provided by TMC library as I don't want hal stuff)
add_library(trinamic INTERFACE)
target_sources(trinamic INTERFACE
    ${TMC_SRC_DIRECTORY}/common.c
    ${TMC_SRC_DIRECTORY}/tmc_interface.c
    ${TMC_SRC_DIRECTORY}/tmc2130.c
    ${TMC_SRC_DIRECTORY}/tmc2209.c
    ${TMC_SRC_DIRECTORY}/tmc5160.c
)
target_include_directories(trinamic INTERFACE ${TMC_SRC_DIRECTORY})



# Pull in u8g2 mui for Mini 12864
add_library(u8g2_mui
    ${U8G2_MUI_DIRECTORY}/mui_u8g2.c
    ${U8G2_MUI_DIRECTORY}/mui.c
    ${U8G2_MUI_DIRECTORY}/u8g2_button.c
    ${U8G2_MUI_DIRECTORY}/u8g2_box.c
)
target_link_libraries(u8g2_mui u8g2)

message(STATUS "Building with Mini 12864 display (u8g2)")

# Include src build commands (HTML generation, PIO headers, etc.)
# Note: Using include() instead of add_subdirectory() since source is at root level
include(${CMAKE_SOURCE_DIR}/src_CMakeLists.txt)

# Collect all source files
file(GLOB SRC ${SRC_DIRECTORY}/*.c 
              ${SRC_DIRECTORY}/*.cpp)

# Include application source file
target_sources("${TARGET_NAME}" PUBLIC
    ${SRC}
)

# Include additional headers
target_include_directories("${TARGET_NAME}" PUBLIC ${CMAKE_SOURCE_DIR}/targets)

# Include libraries
target_link_libraries("${TARGET_NAME}"
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_i2c
    hardware_pwm
    hardware_dma
    hardware_flash
    hardware_sync
    hardware_watchdog
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    trinamic
    app_version

    # Required by cyw43 wireless module
    pico_cyw43_arch_lwip_sys_freertos
    # pico_lwip_http - not used, project has custom httpd in http_rest.c

    # Required for OTA firmware updates
    pico_flash
    hardware_flash
    hardware_watchdog
)

# Display library
target_link_libraries("${TARGET_NAME}" u8g2 u8g2_mui)

# Configure AP IP address (192.168.4.1)
# Make USB serial printf non-blocking so it doesn't stall lwIP callbacks
# (default 500ms timeout blocks HTTP handlers when no serial terminal is open)
target_compile_definitions("${TARGET_NAME}" PRIVATE
    CYW43_DEFAULT_IP_AP_ADDRESS=0xC0A80401
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
)

# OTA Firmware Update Configuration
# Note: TBYB and WiFi partition disabled for now - need to flash WiFi firmware to partition first
# target_compile_definitions("${TARGET_NAME}" PRIVATE
#     PICO_CRT0_IMAGE_TYPE_TBYB=1
# )
# pico_use_wifi_firmware_partition("${TARGET_NAME}")

target_link_options("${TARGET_NAME}" PUBLIC -Wl,--gc-sections -Wl,--print-memory-usage)
# set( CMAKE_VERBOSE_MAKEFILE on )

# Generate extra outputs
pico_add_extra_outputs("${TARGET_NAME}")

# Disable USB and UART stdio (not needed - saves memory and boot time)
pico_enable_stdio_usb("${TARGET_NAME}" 0)
pico_enable_stdio_uart("${TARGET_NAME}" 0)

# OTA firmware update support (DISABLED)
# message(STATUS "Building with OTA firmware update support")
# add_subdirectory(ota)
# target_link_libraries("${TARGET_NAME}" ota)
