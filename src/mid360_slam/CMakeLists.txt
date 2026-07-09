cmake_minimum_required(VERSION 3.8)
project(mid360_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/relocalization.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/coordinate_monitor.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/coordinate_display.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/save_coordinates.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/print_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
