cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenMP QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  /opt/ros/humble/include
  include
)

add_executable(fastlio_mapping src/laserMapping_relocalization.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
ament_target_dependencies(fastlio_mapping
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions pcl_ros livox_ros_driver2 std_srvs
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${Sophus_LIBRARIES} fmt)

install(TARGETS
  fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(PROGRAMS scripts/keyboard_catch_local.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
