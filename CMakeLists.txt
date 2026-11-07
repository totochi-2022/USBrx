cmake_minimum_required(VERSION 3.13)

# Locate the Pico SDK. Prefer an in-tree ./pico-sdk (handy for local work),
# otherwise fall back to the PICO_SDK_PATH environment variable, which is the
# standard Pico workflow (see README). The SDK is NOT committed to this repo.
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/pico-sdk/pico_sdk_init.cmake)
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
endif()

# Target the Raspberry Pi Pico (RP2040) by default; -DPICO_BOARD=pico2 for RP2350.
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(usbrx C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(usbrx
    src/main.c
    src/usb_descriptors.c
    src/i2s_rx.c
)

# I2S slave receiver PIO program (from DSPi's i2s_input.pio).
pico_generate_pio_header(usbrx ${CMAKE_CURRENT_LIST_DIR}/src/i2s_rx.pio)

# Make tusb_config.h (in src/) visible to the TinyUSB sources, which are compiled
# in this executable's context by the SDK.
target_include_directories(usbrx PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(usbrx
    pico_stdlib
    hardware_pio
    hardware_dma
    tinyusb_device
)

# Debug log over UART0 (GP0/GP1); keep USB free for audio.
pico_enable_stdio_uart(usbrx 1)
pico_enable_stdio_usb(usbrx 0)

# Produce .uf2 / .elf / .bin / .map etc.
pico_add_extra_outputs(usbrx)
