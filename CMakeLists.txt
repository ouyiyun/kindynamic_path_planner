cmake_minimum_required(VERSION 3.0.2)
project(kindynamic_path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Debug
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar_ros.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Node2D.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Node3D.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pure_pursuit.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vector2d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test.cpp
)

find_package(ompl 1.6 REQUIRED)
if(OMPL_FOUND)
  message(STATUS "Found ompl!")
else()
  message(STATUS "Not found ompl!")
endif()

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  roscpp
  nav_core
  base_local_planner
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kindynamic_path_planner
  CATKIN_DEPENDS 
    costmap_2d
    roscpp
    nav_core
    base_local_planner
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(dynamicvoronoi src/bucketedqueue.cpp src/dynamicvoronoi.cpp)
target_link_libraries(dynamicvoronoi ${catkin_LIBRARIES}) 

add_library(kindynamic_path_planner ${SOURCES})
target_link_libraries(kindynamic_path_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES}) 

add_executable(test_node ${SOURCES})
target_link_libraries(test_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES}) 