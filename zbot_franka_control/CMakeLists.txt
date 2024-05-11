cmake_minimum_required(VERSION 3.8)
project(zbot_franka_control)

# Set the C++ standard to use
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Add compile options for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 base dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Declare the executable from the specified source file
add_executable(joint_state_to_control src/joint_state_to_control.cpp)

# Declare dependencies of the executable for ament to automatically provide include directories, libraries, etc.
ament_target_dependencies(joint_state_to_control
  rclcpp
  sensor_msgs
  std_msgs
)

# Install the executable to lib/<project_name>
install(TARGETS
  joint_state_to_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories, config and launch, to share/<project_name>
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Package the project as a ROS 2 package
ament_package()
