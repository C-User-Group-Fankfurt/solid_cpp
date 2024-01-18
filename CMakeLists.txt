cmake_minimum_required(VERSION 3.10)
project(driving_system)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(driving_system
        src/driving_system.cpp
)

target_include_directories(driving_system
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_options(driving_system PUBLIC
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>: -Wall -Wextra -Wpedantic -Werror>
        $<$<CXX_COMPILER_ID:MSVC>: /W4>)

set_target_properties(driving_system
        PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED On)
