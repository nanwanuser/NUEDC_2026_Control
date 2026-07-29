cmake_minimum_required(VERSION 3.22)
project(trajectory_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(trajectory_test
    trajectory_test.c
    ../../App/lib/trajectory/trajectory.c
)

target_include_directories(trajectory_test PRIVATE
    ../../App/lib/trajectory
)

target_compile_options(trajectory_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(trajectory_test PRIVATE m)

add_test(NAME trajectory_test COMMAND trajectory_test)
