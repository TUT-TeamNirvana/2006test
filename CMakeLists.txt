cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 2006test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/bsp/can/bsp_can.c
        Core/bsp/can/bsp_can.h
        Core/modules/motor/m2006motor/m2006_motor.c
        Core/modules/motor/m2006motor/m2006_motor.h
        Core/bsp/can/bsp_can.h
        Core/bsp/can/bsp_can.c
        Core/bsp/Allheader/Allheader.h
        Core/modules/motor/servo_motor/ring_buffer.h
        Core/modules/motor/servo_motor/ring_buffer.c
        Core/modules/motor/servo_motor/servo_motor_uart.c
        Core/modules/motor/servo_motor/servo_motor_uart.h
        Core/modules/motor/servo_motor/motor_test.h
        Core/modules/motor/servo_motor/motor_test.c
        Core/modules/pid/2006pid.c
        Core/modules/pid/2006pid.c
        Core/modules/motor/servo_motor/servo_motor.c
        Core/modules/motor/servo_motor/servo_motor.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/bsp/can
        Core/modules/pid
        Core/modules/motor/m2006motor
        Core/modules/motor/servo_motor
        Core/bsp/Allheader
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
