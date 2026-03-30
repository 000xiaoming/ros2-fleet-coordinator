cmake_minimum_required(VERSION 3.8)
project(fleet_runtime)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(goal_follower_node src/goal_follower_node.cpp)
target_compile_features(goal_follower_node PUBLIC cxx_std_17)
ament_target_dependencies(goal_follower_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

add_executable(fleet_visualization_node src/fleet_visualization_node.cpp)
target_compile_features(fleet_visualization_node PUBLIC cxx_std_17)
ament_target_dependencies(fleet_visualization_node
  fleet_msgs
  geometry_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

install(TARGETS
  goal_follower_node
  fleet_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
