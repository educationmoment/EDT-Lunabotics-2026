cmake_minimum_required(VERSION 3.8)
project(interfaces_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ✅ Add these (needed for actions, and often msgs)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DepositingRequest.srv"
  "srv/ExcavationRequest.srv"
  "srv/NavigationRequest.srv"
  "msg/MotorHealth.msg"
  "action/Excavation.action"
  "action/Depositing.action"
  "action/Navigation.action"
  DEPENDENCIES action_msgs std_msgs
)

# ✅ This is the critical export so downstream C++ targets link correctly
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
