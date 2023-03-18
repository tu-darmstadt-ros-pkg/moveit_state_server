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
find_package(Boost REQUIRED COMPONENTS filesystem)
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        moveit_core
        moveit_ros_planning_interface
        moveit_state_server_msgs
)

###########
## Build ##
###########


add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${catkin_INCLUDE_DIRS}
        )
target_link_libraries(${PROJECT_NAME} INTERFACE ${EIGEN3_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
    target_link_libraries(joint_state_storage_test ${TEST_LIBRARIES})
endif ()

# to run: catkin build --this --no-deps -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args hector_math_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information on how to get the coverage
if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/build/*")
    add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif ()