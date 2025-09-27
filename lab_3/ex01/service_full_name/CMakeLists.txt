cmake_minimum_required(VERSION 3.8)
project(service_full_name)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_full_name_interfaces REQUIRED)

add_executable(service_name src/service_name.cpp)
ament_target_dependencies(service_name rclcpp service_full_name_interfaces)
add_executable(client_name src/client_name.cpp)
ament_target_dependencies(client_name rclcpp service_full_name_interfaces)

install(TARGETS
  service_name
  client_name
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
