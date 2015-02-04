cmake_minimum_required(VERSION 2.8.3)
project(iis_komo)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  iis_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  roscpp
  actionlib
  iis_fast_ik
)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   AddPrimitiveShape.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   PlanTrajectory.srv
#   ExecuteTrajectory.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   trajectory_msgs
#   geometry_msgs
#   sensor_msgs
#)


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
  INCLUDE_DIRS include
#  LIBRARIES iis_komo
  CATKIN_DEPENDS roscpp iis_msgs trajectory_msgs geometry_msgs iis_fast_ik
#  DEPENDS $ENV{KOMO_PATH}/src
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/iis_motion_planning/KOMO/share/src
)

link_directories(
  ${CMAKE_SOURCE_DIR}/iis_motion_planning/KOMO/share/lib
)

## Declare a cpp library
add_library(komo_wrapper src/komo_wrapper.cpp)
target_link_libraries(komo_wrapper Ors Core Motion ${catkin_LIBRARIES})
set_target_properties(komo_wrapper PROPERTIES COMPILE_FLAGS -std=c++0x)

add_library(robot_interface src/robot_interface.cpp)
target_link_libraries(robot_interface ${catkin_LIBRARIES})
set_target_properties(robot_interface PROPERTIES COMPILE_FLAGS -std=c++0x)

add_library(time_parameterization src/time_parameterization.cpp)
target_link_libraries(time_parameterization Core ${catkin_LIBRARIES})
set_target_properties(time_parameterization PROPERTIES COMPILE_FLAGS -std=c++0x)

## Declare a cpp executable
add_executable(iis_komo src/iis_komo.cpp)
add_dependencies(iis_komo komo_wrapper robot_interface iis_msgs_generate_messages_cpp)
target_link_libraries(iis_komo komo_wrapper robot_interface time_parameterization ${catkin_LIBRARIES})
set_target_properties(iis_komo PROPERTIES COMPILE_FLAGS -std=c++0x)

#add_executable(komo_test src/test.cpp)
#target_link_libraries(komo_test Ors Core ${catkin_LIBRARIES})
#set_target_properties(komo_test PROPERTIES COMPILE_FLAGS -std=c++0x)

add_executable(komo_info src/info.cpp)
add_dependencies(komo_info iis_msgs_generate_messages_cpp)
target_link_libraries(komo_info Ors Core Motion Optim Gui ${catkin_LIBRARIES})
set_target_properties(komo_info PROPERTIES COMPILE_FLAGS -std=c++0x)

## examples

add_executable(move_cart examples/move_cart.cpp)
add_dependencies(move_cart iis_msgs_generate_messages_cpp)
target_link_libraries(move_cart ${catkin_LIBRARIES})

add_executable(move_joint examples/move_joint.cpp)
add_dependencies(move_joint iis_msgs_generate_messages_cpp)
target_link_libraries(move_joint ${catkin_LIBRARIES})

add_executable(planning_scene examples/planning_scene.cpp)
add_dependencies(planning_scene iis_msgs_generate_messages_cpp)
target_link_libraries(planning_scene ${catkin_LIBRARIES})

## tests
add_executable(test_komo_pick test/test_KOMO_pick.cpp)
add_dependencies(test_komo_pick iis_msgs_generate_messages_cpp)
target_link_libraries(test_komo_pick ${catkin_LIBRARIES})

add_executable(test_arm_move test/test_arm_move.cpp)
add_dependencies(test_arm_move iis_msgs_generate_messages_cpp)
target_link_libraries(test_arm_move ${catkin_LIBRARIES})

add_executable(performance_test test/performance_test.cpp)
target_link_libraries(performance_test Ors Core Motion Optim ${catkin_LIBRARIES})
set_target_properties(performance_test PROPERTIES COMPILE_FLAGS -std=c++0x)

## deactivated because of unknown dependency problems...
#add_executable(performance_test_ik test/performance_test_ik.cpp)
#target_link_libraries(performance_test_ik Ors Core Motion Optim ${catkin_LIBRARIES})
#set_target_properties(performance_test_ik PROPERTIES COMPILE_FLAGS -std=c++0x)

add_executable(reachability_test test/reachability_test.cpp)
target_link_libraries(reachability_test Ors Core Motion Optim ${catkin_LIBRARIES})
set_target_properties(reachability_test PROPERTIES COMPILE_FLAGS -std=c++0x)
