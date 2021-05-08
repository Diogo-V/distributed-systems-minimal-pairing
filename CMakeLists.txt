cmake_minimum_required(VERSION 3.17)
project(distributed_systems)

set(CMAKE_CXX_STANDARD 14)
set(GXX_COVERAGE_COMPILE_FLAGS "-g -ansi -Wall -Wextra -pthread -lm -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GXX_COVERAGE_COMPILE_FLAGS}" )

add_executable(main src/main.cpp)
