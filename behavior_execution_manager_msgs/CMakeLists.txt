cmake_minimum_required(VERSION 2.8.3)
project(behavior_execution_manager_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  BehaviorActivationFinished.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ActivateBehavior.srv
  CheckSituation.srv
  DeactivateBehavior.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
