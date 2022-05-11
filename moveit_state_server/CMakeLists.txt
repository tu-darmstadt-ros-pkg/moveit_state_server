cmake_minimum_required(VERSION 3.06)
project(moveit_state_server)

set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        geometry_msgs
        moveit_state_server_msgs
        controller_manager
        )

find_package(Eigen3 REQUIRED)
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        moveit_core
        moveit_ros_planning_interface

)

include_directories(${EIGEN_INCLUDE_DIRS} include)
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)



add_executable(moveit_state_server
        src/moveit_state_server.cpp)

target_link_libraries(moveit_state_server
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS moveit_state_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#install(DIRECTORY
#        config
#        launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )

