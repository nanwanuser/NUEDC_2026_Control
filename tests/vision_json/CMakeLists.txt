cmake_minimum_required(VERSION 3.22)
project(vision_json_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(vision_json_test
    vision_json_test.c
    ../../App/lib/vision_json/vision_json.c
)

target_include_directories(vision_json_test PRIVATE
    ../../App/lib/vision_json
    ../../App/lib/decision
    ../../App/lib/trajectory
    ../../App/third_party/jsmn
)

target_compile_options(vision_json_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(vision_json_test PRIVATE m)

add_test(NAME vision_json_test COMMAND vision_json_test)
