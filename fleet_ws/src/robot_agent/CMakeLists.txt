cmake_minimum_required(VERSION 3.8)
project(robot_agent)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_agent_node src/robot_agent_node.cpp)
target_compile_features(robot_agent_node PUBLIC cxx_std_17)
ament_target_dependencies(robot_agent_node
  fleet_msgs
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS robot_agent_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
