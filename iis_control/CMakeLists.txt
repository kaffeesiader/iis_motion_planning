cmake_minimum_required(VERSION 2.8.3)
project(iis_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_manager
    actionlib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  INCLUDE_DIRS include
#  LIBRARIES iis_control
    CATKIN_DEPENDS roscpp controller_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    /usr/include/eigen3
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(hardware_adapter
    src/IISRobotHW.cpp
    src/IISControllerManager.cpp)
target_link_libraries(hardware_adapter ${catkin_LIBRARIES})

add_executable(test_gripper_action test/test_gripper_action_client.cpp)
target_link_libraries(test_gripper_action ${catkin_LIBRARIES})
