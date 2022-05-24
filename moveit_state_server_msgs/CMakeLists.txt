cmake_minimum_required(VERSION 3.06)
project(moveit_state_server_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        message_generation
        actionlib_msgs
        )

## Generate messages in the 'msg' folder
#add_message_files(
#        FILES
#)

## Generate services in the 'srv' folder
add_service_files(
   FILES
  StorePose.srv
        RetrievePose.srv
 )

## Generate actions in the 'action' folder
add_action_files(
        FILES
        GoToStoredState.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS std_msgs message_runtime
)
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
