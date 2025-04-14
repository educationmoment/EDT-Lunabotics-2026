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
find_package(std_msgs REQUIRED)
find_package(interfaces_pkg REQUIRED)

# Add executables
add_executable(controller_node src/controller_node.cpp)
add_executable(depositing_node src/depositing_node.cpp)
add_executable(excavation_node src/excavation_node.cpp)
add_executable(health_node src/health_node.cpp)
# Link Dependencies
ament_target_dependencies(controller_node rclcpp std_msgs sensor_msgs sparkcan interfaces_pkg)
ament_target_dependencies(depositing_node rclcpp std_msgs sensor_msgs sparkcan interfaces_pkg)
ament_target_dependencies(excavation_node rclcpp std_msgs sensor_msgs sparkcan interfaces_pkg)
ament_target_dependencies(health_node rclcpp std_msgs sensor_msgs sparkcan interfaces_pkg)
# Install the Executables
install(TARGETS
  controller_node
  depositing_node
  excavation_node
  health_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
