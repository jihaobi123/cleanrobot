cmake_minimum_required(VERSION 3.5)
project(cleanrobot_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/coverage_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_waypoints
)

install(PROGRAMS
  scripts/fake_base.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
