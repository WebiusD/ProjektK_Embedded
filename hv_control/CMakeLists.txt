# Set minimum required version:
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK:
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set the name of the project and c++ standards:
project(hv_control C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Create the pico-sdk subdirectory in our project for the libraries:
pico_sdk_init()

# Tell CMake where to find the executable source file:
add_executable(${PROJECT_NAME}
    main.cpp
)

# Create uf2 file:
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
)