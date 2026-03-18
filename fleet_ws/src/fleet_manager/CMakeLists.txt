cmake_minimum_required(VERSION 3.8)
project(fleet_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(fleet_manager_node src/fleet_manager_node.cpp)
target_compile_features(fleet_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(fleet_manager_node rclcpp fleet_msgs)

install(TARGETS fleet_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
