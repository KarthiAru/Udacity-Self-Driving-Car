cmake_minimum_required(VERSION 3.29 FATAL_ERROR)

project(playback)

set(CMAKE_CXX_STANDARD 11)  # Updated to C++14, if necessary
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")  # Corrected the syntax here

find_package(PCL 1.4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries(environment ${PCL_LIBRARIES} ${Boost_LIBRARIES})