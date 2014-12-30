cmake_minimum_required(VERSION 2.8.3)
project(iis_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  trajectory_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
#   FILES
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
#   FILES
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   trajectory_msgs
   sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime geometry_msgs trajectory_msgs sensor_msgs
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)