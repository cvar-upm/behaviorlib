cmake_minimum_required(VERSION 2.8.3)
project(behavior_execution_manager)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
add_definitions(-std=c++11)

# Directories definition
set(BEHAVIORLIB_SOURCE_DIR
  src/source
)

set(BEHAVIORLIB_INCLUDE_DIR
  src/include
)

# Files declaration
set(BEHAVIORLIB_SOURCE_FILES
  ${BEHAVIORLIB_SOURCE_DIR}/BehaviorExecutionManager.cpp
)

set(BEHAVIORLIB_HEADER_FILES
  ${BEHAVIORLIB_INCLUDE_DIR}/BehaviorExecutionManager.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  behavior_execution_manager_msgs
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BEHAVIORLIB_INCLUDE_DIR}
  LIBRARIES behavior_execution_manager
  CATKIN_DEPENDS
  roscpp
  std_msgs
  behavior_execution_manager_msgs
)

###########
## Build ##
###########
include_directories(
  ${BEHAVIORLIB_INCLUDE_DIR}
  ${BEHAVIORLIB_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(behavior_execution_manager ${BEHAVIORLIB_SOURCE_FILES} ${BEHAVIORLIB_HEADER_FILES})
add_dependencies(behavior_execution_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_execution_manager ${catkin_LIBRARIES})

