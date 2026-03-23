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
  scripts/check_planner_failure_path.sh
  scripts/check_conflict_reservation.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
