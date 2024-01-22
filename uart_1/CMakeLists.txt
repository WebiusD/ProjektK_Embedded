cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(uart1_test C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    main.c
)

pico_add_extra_outputs(${PROJECT_NAME})

# pull in common dependencies and additional pwm hardware support
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
)