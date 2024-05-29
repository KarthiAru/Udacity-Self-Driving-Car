cmake_minimum_required(VERSION 3.10)
project(Localization)

# Set the C++ standard directly via the CMake variable
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -g allows for gdb debugging
# turn on -O3 for best performance
add_compile_options(-O3)

# If using pthread, you should find the thread package and link it properly
find_package(Threads REQUIRED)

set(sources icp1-main.cpp helper.cpp)

# Find PCL and link it
find_package(PCL 1.4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(icp ${sources})

# Linking against pthread, PCL, and the standard C++ library
target_link_libraries(icp ${PCL_LIBRARIES} Threads::Threads)

# Ensure we are linking against the correct C++ standard library on macOS
if(APPLE)
    include_directories(/usr/include/libcxxabi)
    target_link_libraries(icp c++abi)
endif()

# project(Localization)
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# cmake_minimum_required (VERSION 3.10)

# # -g allows for gdb debugging
# # turn on -03 for best performance
# add_definitions(-std=c++14 -O3)

# set(CXX_FLAGS "-pthread -fPIC")
# set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

# set(sources icp1-main.cpp helper.cpp)

# find_package(PCL 1.4 REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# add_executable(icp ${sources} )

# target_link_libraries(icp ${PCL_LIBRARIES})
