cmake_minimum_required(VERSION 3.22)
project(route_planning_yaw_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(route_planning_yaw_test
    route_planning_yaw_test.c
    ../../Task/route_planning/route_yaw_bias.c
)

target_include_directories(route_planning_yaw_test PRIVATE
    ../../Task/crane_control
    ../../Task/route_planning
    ../../App/lib/trajectory
)

target_compile_options(route_planning_yaw_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(route_planning_yaw_test PRIVATE m)

add_test(NAME route_planning_yaw_test COMMAND route_planning_yaw_test)
