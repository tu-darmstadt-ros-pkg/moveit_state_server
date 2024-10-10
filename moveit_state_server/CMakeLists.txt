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
        dynamic_reconfigure
        warehouse_ros
        )

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(config/MoveitStateServer.cfg)

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        moveit_core
        moveit_ros_planning_interface
        moveit_state_server_msgs
)

include_directories(${EIGEN_INCLUDE_DIRS} include)
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)



add_executable(moveit_state_server
        src/moveit_state_server.cpp )

add_dependencies(moveit_state_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

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

install(DIRECTORY
        config
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
    find_package(roscpp REQUIRED)
    find_package(rostest REQUIRED)
    find_package(roslib REQUIRED) # Provides ros::package
    set(TEST_LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

    include_directories(
            ${catkin_INCLUDE_DIRS}
            ${EIGEN3_INCLUDE_DIRS}
            ${roscpp_INCLUDE_DIRS}
            ${roslib_INCLUDE_DIRS}
            ${rostest_INCLUDE_DIRS}
    )
    ## Add gtest based cpp test target and link libraries
    catkin_add_gtest(joint_state_storage_test test/joint_state_storage_test.cpp)
    target_link_libraries(joint_state_storage_test ${catkin_LIBRARIES})
endif ()