cmake_minimum_required(VERSION 3.22)
project(decision_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(decision_test
    decision_test.c
    ../../App/lib/decision/decision.c
    ../../App/lib/trajectory/trajectory.c
)

target_include_directories(decision_test PRIVATE
    ../../App/lib/decision
    ../../App/lib/trajectory
)

target_compile_options(decision_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(decision_test PRIVATE m)

add_test(NAME decision_test COMMAND decision_test)
