cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sparkcan REQUIRED)  # Your SparkMax CAN library
find_package(nav_msgs REQUIRED)
include_directories(
  include
)

# Build the main nav2-integrated node
add_executable(navigation_node
  src/navigation_node.cpp         # If you renamed SparkBase (1).cpp to this
)

ament_target_dependencies(navigation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  sparkcan
  nav_msgs
)

# Install the executable
install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()

