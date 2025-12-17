cmake_minimum_required(VERSION 3.8)
project(robot_circle_movement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(robot_circle_movement src/robot_circle_movement.cpp)
ament_target_dependencies(
  robot_circle_movement
  geometry_msgs
  rclcpp
)

install(TARGETS
  robot_circle_movement
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
