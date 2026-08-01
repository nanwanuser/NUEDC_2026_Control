cmake_minimum_required(VERSION 3.22)
project(vision_mode_retry_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(vision_mode_retry_test
    vision_mode_retry_test.c
    ../../App/lib/vision_uart/vision_mode_retry.c
)

target_include_directories(vision_mode_retry_test PRIVATE
    ../../App/lib/vision_uart
    ../../App/lib/vision_protocol
    ../../App/lib/decision
    ../../App/lib/trajectory
)

target_compile_options(vision_mode_retry_test PRIVATE -Wall -Wextra -Werror)

add_test(NAME vision_mode_retry_test COMMAND vision_mode_retry_test)
