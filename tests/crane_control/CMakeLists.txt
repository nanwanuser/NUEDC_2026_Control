cmake_minimum_required(VERSION 3.22)
project(crane_control_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(crane_control_test
    crane_control_test.c
    ../../Task/crane_control/crane_control.c
    ../../Task/crane_control/crane_lift_trigger.c
)

target_include_directories(crane_control_test PRIVATE
    stubs
    ../../Task/crane_control
    ../../Task/route_planning
    ../../App/lib/trajectory
)

target_compile_options(crane_control_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(crane_control_test PRIVATE m)

add_test(NAME crane_control_test COMMAND crane_control_test)
add_test(NAME crane_startup_retry_test
         COMMAND crane_control_test --startup-retry)
