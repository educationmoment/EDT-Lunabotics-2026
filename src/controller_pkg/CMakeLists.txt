cmake_minimum_required(VERSION 3.8)
project(controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sparkcan REQUIRED)

# Add the executable
add_executable(controller_node src/controller_node.cpp)

# Link Dependencies
ament_target_dependencies(controller_node rclcpp std_msgs sensor_msgs sparkcan)

# Install the Executable
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament Package
ament_package()
