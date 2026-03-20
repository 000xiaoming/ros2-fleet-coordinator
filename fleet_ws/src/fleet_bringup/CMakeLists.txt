cmake_minimum_required(VERSION 3.8)
project(fleet_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/submit_demo_task.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
