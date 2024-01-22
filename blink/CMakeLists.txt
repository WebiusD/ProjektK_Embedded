# Set minimum required version:
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK:
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set the name of the project and c++ standards:
project(blink C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Create the pico-sdk subdirectory in our project for the libraries:
pico_sdk_init()

# Tell CMake where to find the executable source file:
add_executable(${PROJECT_NAME}
    main.c
)

# Create uf2 file:
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
)

# Enable/Disable USB or UART output:
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)