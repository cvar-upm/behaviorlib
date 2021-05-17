##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME keyboard_behavior_activation)
project(${PROJECT_NAME})

add_definitions(-std=c++11)
##############################################################################
# Catkin
##############################################################################
set(KEYBOARD_BEHAVIOR_ACTIVATION_CONTROLLER_SOURCE_DIR
        src/source)

set(KEYBOARD_BEHAVIOR_ACTIVATION_CONTROLLER_INCLUDE_DIR
        src/include)

set(KEYBOARD_BEHAVIOR_ACTIVATION_HEADER_FILES
        ${KEYBOARD_BEHAVIOR_ACTIVATION_CONTROLLER_INCLUDE_DIR}/keyboard_behavior_activation_main.h
)

set(KEYBOARD_BEHAVIOR_ACTIVATION_SOURCE_FILES
       ${KEYBOARD_BEHAVIOR_ACTIVATION_CONTROLLER_SOURCE_DIR}/keyboard_behavior_activation_main.cpp
)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs droneMsgsROS
  drone_process robot_process sensor_msgs geometry_msgs image_transport aerostack_msgs behavior_execution_manager behavior_execution_manager_msgs)

catkin_package(
        INCLUDE_DIRS ${KEYBOARD_BEHAVIOR_ACTIVATION_CONTROLLER_INCLUDE_DIR}
        LIBRARIES keyboard_behavior_activation_lib
        DEPENDS yaml-cpp
        CATKIN_DEPENDS roscpp std_msgs robot_process sensor_msgs geometry_msgs image_transport aerostack_msgs behavior_execution_manager behavior_execution_manager_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_library(keyboard_behavior_activation_lib ${KEYBOARD_BEHAVIOR_ACTIVATION_SOURCE_FILES}
  ${KEYBOARD_BEHAVIOR_ACTIVATION_HEADER_FILES}
  )
add_dependencies(keyboard_behavior_activation_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_behavior_activation_lib ncursesw ${catkin_LIBRARIES})


add_executable(keyboard_behavior_activation ${KEYBOARD_BEHAVIOR_ACTIVATION_CONTROLLER_SOURCE_DIR}/keyboard_behavior_activation_main.cpp)
add_dependencies(keyboard_behavior_activation ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_behavior_activation keyboard_behavior_activation_lib)
target_link_libraries(keyboard_behavior_activation yaml-cpp)
target_link_libraries(keyboard_behavior_activation ${catkin_LIBRARIES})

