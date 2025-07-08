cmake_minimum_required(VERSION 3.10)
project(moveit_state_server_msgs)

# Use modern C++
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Add service files
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/StorePose.srv"
  "srv/RetrievePose.srv"
  "action/GoToStoredState.action"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  action_msgs)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
