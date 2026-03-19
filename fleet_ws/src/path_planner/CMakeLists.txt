cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(path_planner_node src/path_planner_node.cpp)
target_compile_features(path_planner_node PUBLIC cxx_std_17)
ament_target_dependencies(path_planner_node rclcpp fleet_msgs)

install(TARGETS path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
