cmake_minimum_required(VERSION 3.8)
project(zbot_franka_control)

# Set the C++ standard if it is not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Apply stricter compile options for GCC and Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Define executables from source files
add_executable(joint_state_to_control_pub src/joint_state_to_control_pub.cpp)
ament_target_dependencies(joint_state_to_control_pub
  rclcpp
  sensor_msgs
  trajectory_msgs
)

add_executable(joint_state_to_control_action_client src/joint_state_to_control_action_client.cpp)
ament_target_dependencies(joint_state_to_control_action_client
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  action_msgs
  trajectory_msgs
)

# Specify the installation path for executables in the 'lib' directory under the project name
install(TARGETS
  joint_state_to_control_pub
  joint_state_to_control_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch directories to the 'share' directory under the project name
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Configure the project as a ROS 2 package
ament_package()
