cmake_minimum_required(VERSION 3.8)
project(motor_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED) 

add_executable(motor_control_node src/motor_control_node.cpp)
ament_target_dependencies(motor_control_node rclcpp rcl_interfaces std_msgs)

install(TARGETS
  motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()