cmake_minimum_required(VERSION 3.8)
project(vision_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rs_camera_node src/rs_camera_node.cpp)

target_link_libraries(rs_camera_node ${realsense2_LIBRARY})
ament_target_dependencies(rs_camera_node rclcpp realsense2 sensor_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(TARGETS
  rs_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
