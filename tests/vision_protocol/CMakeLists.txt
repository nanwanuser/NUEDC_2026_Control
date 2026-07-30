cmake_minimum_required(VERSION 3.22)
project(vision_protocol_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(vision_protocol_test
    vision_protocol_test.c
    ../../App/lib/vision_protocol/vision_protocol.c
)

target_include_directories(vision_protocol_test PRIVATE
    ../../App/lib/vision_protocol
    ../../App/lib/decision
    ../../App/lib/trajectory
)

target_compile_options(vision_protocol_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(vision_protocol_test PRIVATE m)

add_test(NAME vision_protocol_test COMMAND vision_protocol_test)
