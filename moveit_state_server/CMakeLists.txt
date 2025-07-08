cmake_minimum_required(VERSION 3.25)
project(moveit_state_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_state_server_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_orchestrator REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(joint_state_storage_objs
        src/joint_state_file_storage.cpp          # + src/joint_state_database.cpp ...
)
target_include_directories(joint_state_storage_objs PUBLIC include)
ament_target_dependencies(joint_state_storage_objs
        rclcpp sensor_msgs
)

add_executable(moveit_state_server
        src/moveit_state_server.cpp
        )
target_link_libraries(moveit_state_server joint_state_storage_objs)
ament_target_dependencies(moveit_state_server
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        moveit_state_server_msgs
        geometry_msgs
        controller_manager
        sensor_msgs
        tf2_ros
        controller_orchestrator
)

install(TARGETS moveit_state_server
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        config
        launch
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(joint_state_storage_test test/joint_state_storage_test.cpp)
    target_link_libraries(joint_state_storage_test joint_state_storage_objs)
    ament_target_dependencies(joint_state_storage_test
            rclcpp
            moveit_core
            moveit_ros_planning_interface
            moveit_state_server_msgs
            geometry_msgs
            controller_manager
    )
endif ()


ament_package()
