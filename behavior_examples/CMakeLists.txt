cmake_minimum_required(VERSION 2.8.3)
project(behavior_examples)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)

# Directories definition
set(BEHAVIOR_EXAMPLES_SOURCE_DIR
  src/source
)

set(BEHAVIOR_EXAMPLES_INCLUDE_DIR
  src/include
)

set(BEHAVIOR_EXAMPLES_HEADER_FILES
  ${BEHAVIOR_EXAMPLES_INCLUDE_DIR}/behavior_move_forward.h
  ${BEHAVIOR_EXAMPLES_INCLUDE_DIR}/behavior_move_to_point.h
)

set(BEHAVIOR_EXAMPLES_SOURCE_FILES
  ${BEHAVIOR_EXAMPLES_SOURCE_DIR}/behavior_move_forward.cpp
  ${BEHAVIOR_EXAMPLES_SOURCE_DIR}/behavior_move_to_point.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  behaviorlib
  geometry_msgs
  turtlesim
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BEHAVIOR_EXAMPLES_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  nodelet
  std_msgs
  behaviorlib
  geometry_msgs
  turtlesim
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(behavior_examples
  ${BEHAVIOR_EXAMPLES_SOURCE_DIR}/behavior_move_forward.cpp
  ${BEHAVIOR_EXAMPLES_SOURCE_DIR}/behavior_move_to_point.cpp
)
target_link_libraries(behavior_examples ${catkin_LIBRARIES})
target_link_libraries(behavior_examples yaml-cpp)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(behavior_examples ${catkin_EXPORTED_LIBRARIES})
endif()
