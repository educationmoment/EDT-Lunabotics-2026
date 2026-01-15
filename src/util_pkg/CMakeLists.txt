cmake_minimum_required(VERSION 3.5)
project(util_pkg)

# Compiler settings
add_definitions(-w)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compile options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_library(SERIAL_LIB serial)


# Define executables
add_executable(blade_joint_controller src/blade_joint_controller.cpp)
add_executable(hardware_monitor src/hardware_monitor.cpp)
add_executable(imu_rotator src/imu_rotator.cpp)
add_executable(topic_remapper src/topic_remapper.cpp)
add_executable(actuator_position src/actuator_position.cpp)
add_executable(arduino_serial_node src/arduino_serial_node.cpp)


target_link_libraries(arduino_serial_node ${SERIAL_LIB})
ament_target_dependencies(arduino_serial_node rclcpp std_msgs)
# Set target dependencies
ament_target_dependencies(blade_joint_controller rclcpp sensor_msgs)
ament_target_dependencies(hardware_monitor rclcpp sensor_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs tf2)
ament_target_dependencies(topic_remapper rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(actuator_position rclcpp std_msgs sensor_msgs)

# Install targets
install(TARGETS
  blade_joint_controller
  hardware_monitor
  imu_rotator
  topic_remapper
  actuator_position
  arduino_serial_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
