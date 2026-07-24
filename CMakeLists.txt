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
set(CMAKE_PROJECT_NAME NUEDC_2026_Control)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/drivers/gray_sensor/Gray_Sensor.c
    App/drivers/icm42688p/icm42688p.c
    App/drivers/motor/motor.c
    App/drivers/servo/Servo.c
    App/lib/imu_fusion/imu_fusion.c
    App/lib/imu_service/app_imu.c
    App/lib/simulink_pid/simulink_pid_ert_rtw/simulink_pid.c
    App/lib/simulink_pid/simulink_pid_ert_rtw/simulink_pid_data.c
    Task/motor_speed_control/motor_speed_control.c
)


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/gray_sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/icm42688p
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/servo
    ${CMAKE_CURRENT_SOURCE_DIR}/App/lib/imu_fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/App/lib/imu_service
    ${CMAKE_CURRENT_SOURCE_DIR}/App/lib/simulink_pid
    ${CMAKE_CURRENT_SOURCE_DIR}/App/lib/simulink_pid/simulink_pid_ert_rtw
    ${CMAKE_CURRENT_SOURCE_DIR}/Task/motor_speed_control
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
    m

    # Add user defined libraries
)
