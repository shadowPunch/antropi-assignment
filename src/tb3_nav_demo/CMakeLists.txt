cmake_minimum_required(VERSION 3.20)
project(tb3_nav_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds maps config rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/send_goal.py
  scripts/explore_and_map.py
  scripts/run_trial.py
  scripts/waypoints.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES
  scripts/nav_utils.py
  scripts/world_geometry.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
