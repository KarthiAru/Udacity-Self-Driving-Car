project(Localization)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required (VERSION 3.10)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14 -O3)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wextra")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -Wl,-Bstatic -lboost_filesystem -Wl,-Bdynamic")

set(sources c3-main.cpp helper.cpp)

set(CARLALIB_DIR ${CMAKE_SOURCE_DIR}/libcarla-install/lib)
set(carla_libs  ${CARLALIB_DIR}/libboost_filesystem.a ${CARLALIB_DIR}/libboost_program_options.a ${CARLALIB_DIR}/libboost_python311.a ${CARLALIB_DIR}/libboost_system.a ${CARLALIB_DIR}/libcarla_client.a ${CARLALIB_DIR}/librpc.a ${CARLALIB_DIR}/libDebugUtils.a ${CARLALIB_DIR}/libDetour.a ${CARLALIB_DIR}/libDetourCrowd.a ${CARLALIB_DIR}/libDetourTileCache.a ${CARLALIB_DIR}/libRecast.a)

link_directories(libcarla-install/lib)
#include_directories(libcarla-install/include)
#include_directories(libcarla-install/include/system)
#include_directories(rpclib/include)
include_directories(/home/ka/carla/LibCarla/source)
include_directories(/home/ka/carla/Build/rpclib-v2.2.1_c5-c10-libcxx-install/include)
include_directories(/home/ka/carla/Build/rpclib-v2.2.1_c5-c10-libcxx-install/include/rpc/msgpack)
include_directories(/home/ka/carla/Build/rpclib-v2.2.1_c5-c10-libcxx-install/include/rpc)

find_package(PCL 1.14 REQUIRED)
find_package(Boost 1.81.0 REQUIRED COMPONENTS filesystem)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${Boost_INCLUDE_DIRS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(cloud_loc ${sources} )

target_link_libraries(cloud_loc ${PCL_LIBRARIES} ${carla_libs} ${Boost_LIBRARIES})


# cmake_minimum_required(VERSION 3.10)
# project(Localization)

# # Set the C++ standard directly via the CMake variable
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # -g allows for gdb debugging
# # turn on -O3 for best performance
# add_compile_options(-O3)

# # If using pthread, you should find the thread package and link it properly
# find_package(Threads REQUIRED)

# set(sources c3-main.cpp helper.cpp)

# set(CARLALIB_DIR ${CMAKE_SOURCE_DIR}/libcarla-install/lib)
# set(carla_libs  ${CARLALIB_DIR}/libboost_filesystem.a ${CARLALIB_DIR}/libboost_program_options.a ${CARLALIB_DIR}/libboost_python38.a ${CARLALIB_DIR}/libboost_system.a ${CARLALIB_DIR}/libcarla_client.a ${CARLALIB_DIR}/librpc.a ${CARLALIB_DIR}/libDebugUtils.a ${CARLALIB_DIR}/libDetour.a ${CARLALIB_DIR}/libDetourCrowd.a ${CARLALIB_DIR}/libDetourTileCache.a ${CARLALIB_DIR}/libRecast.a)

# link_directories(libcarla-install/lib)
# include_directories(libcarla-install/include)
# include_directories(libcarla-install/include/system)
# include_directories(rpclib/include)

# # Find PCL and link it
# find_package(PCL 1.4 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(cloud_loc ${sources})

# # Linking against pthread, PCL, and the standard C++ library
# target_link_libraries(cloud_loc ${PCL_LIBRARIES} Threads::Threads)

# # Ensure we are linking against the correct C++ standard library on macOS
# if(APPLE)
#     include_directories(/usr/include/libcxxabi)
#     target_link_libraries(cloud_loc c++abi)
# endif()