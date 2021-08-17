cmake_minimum_required(VERSION 2.8)
project( BokehCamera )
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( BokehCamera BokehCamera.cpp )
target_link_libraries( BokehCamera ${OpenCV_LIBS} ${realsense2_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
