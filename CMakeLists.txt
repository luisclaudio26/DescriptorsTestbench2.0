cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(DescriptorBenchmark)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g -fopenmp -Wno-deprecated -Wno-deprecated-declarations")
find_package(PCL 1.8 REQUIRED)

#Headers
include_directories(inc)
include_directories(${PCL_INCLUDE_DIRS})

#Source files
file(GLOB SOURCES "src/*.cpp")

#PCL stuff
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(benchmark ${SOURCES})

target_link_libraries(benchmark ${PCL_LIBRARIES})