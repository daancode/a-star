cmake_minimum_required (VERSION 2.8.11)
project (a-star)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(BUILD_EXAMPLE "Build a-star example")
add_library(a-star source/AStar.cpp)

if(BUILD_EXAMPLE)
	add_executable(main main.cpp)
	target_link_libraries(main a-star)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/incl)
