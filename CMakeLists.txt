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
    App/drivers/encoder/Encoder.c
    App/drivers/gray_sensor/Gray_Sensor.c
    App/drivers/hc_sr04/HC-SR04.c
    App/drivers/icm42688p/icm42688p.c
    App/drivers/oled/OLED.c
    App/drivers/oled/OLED_Data.c
    App/drivers/servo/Servo.c
    App/drivers/tb6612/TB6612.c
    App/lib/hcsr04_oled/HC-SR04_OLED.c
    App/lib/imu_fusion/imu_fusion.c
    App/lib/imu_service/app_imu.c
)

# Font tables intentionally use flattened aggregate initializers.
set_source_files_properties(App/drivers/oled/OLED_Data.c PROPERTIES
    COMPILE_OPTIONS "-Wno-missing-braces"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/encoder
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/gray_sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/hc_sr04
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/icm42688p
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/oled
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/servo
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drivers/tb6612
    ${CMAKE_CURRENT_SOURCE_DIR}/App/lib/hcsr04_oled
    ${CMAKE_CURRENT_SOURCE_DIR}/App/lib/imu_fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/App/lib/imu_service
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
