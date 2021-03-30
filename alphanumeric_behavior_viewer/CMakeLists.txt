##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME alphanumeric_behavior_viewer)
project(${PROJECT_NAME})

add_definitions(-std=c++11)
##############################################################################
# Catkin
##############################################################################
set(ALPHANUMERIC_BEHAVIOR_VIEWER_CONTROLLER_SOURCE_DIR
        src/source)

set(ALPHANUMERIC_BEHAVIOR_VIEWER_CONTROLLER_INCLUDE_DIR
        src/include)

set(ALPHANUMERIC_BEHAVIOR_VIEWER_HEADER_FILES
        ${ALPHANUMERIC_BEHAVIOR_VIEWER_CONTROLLER_INCLUDE_DIR}/alphanumeric_behavior_viewer.h
)

set(ALPHANUMERIC_BEHAVIOR_VIEWER_SOURCE_FILES
       ${ALPHANUMERIC_BEHAVIOR_VIEWER_CONTROLLER_SOURCE_DIR}/alphanumeric_behavior_viewer_main.cpp
)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs droneMsgsROS
  drone_process robot_process sensor_msgs geometry_msgs image_transport aerostack_msgs behavior_coordinator_msgs behavior_execution_manager behavior_execution_manager_msgs)

catkin_package(
        INCLUDE_DIRS ${ALPHANUMERIC_BEHAVIOR_VIEWER_CONTROLLER_INCLUDE_DIR}
        DEPENDS yaml-cpp
        CATKIN_DEPENDS roscpp std_msgs robot_process sensor_msgs geometry_msgs image_transport aerostack_msgs behavior_coordinator_msgs behavior_execution_manager behavior_execution_manager_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_library(alphanumeric_behavior_viewer_lib ${ALPHANUMERIC_BEHAVIOR_VIEWER_SOURCE_FILES}
  ${ALPHANUMERIC_BEHAVIOR_VIEWER_HEADER_FILES}
  )
add_dependencies(alphanumeric_behavior_viewer_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(alphanumeric_behavior_viewer_lib ncursesw ${catkin_LIBRARIES})


add_executable(alphanumeric_behavior_viewer ${ALPHANUMERIC_BEHAVIOR_VIEWER_CONTROLLER_SOURCE_DIR}/alphanumeric_behavior_viewer_main.cpp)
add_dependencies(alphanumeric_behavior_viewer ${catkin_EXPORTED_TARGETS})
target_link_libraries(alphanumeric_behavior_viewer alphanumeric_behavior_viewer_lib)
target_link_libraries(alphanumeric_behavior_viewer yaml-cpp)
target_link_libraries(alphanumeric_behavior_viewer ${catkin_LIBRARIES})

