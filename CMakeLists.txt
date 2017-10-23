cmake_minimum_required (VERSION 2.8.11)
project (a-star)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(source)
add_library(a-star source/AStar.cpp)

add_executable(main main.cpp)
target_link_libraries(main a-star)
