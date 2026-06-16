cmake_minimum_required(VERSION 3.5)
project(ekf_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(landmark_detector_node
  src/landmark_detector_node.cpp
  src/LandmarkDetector.cpp)
ament_target_dependencies(landmark_detector_node
  rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs
  robmovil_msgs tf2 tf2_ros tf2_geometry_msgs angles)

add_executable(ekf_localizer
  src/ekf_localizer_node.cpp
  src/EkfLocalizer.cpp)
ament_target_dependencies(ekf_localizer
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  robmovil_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles)
target_link_libraries(ekf_localizer Eigen3::Eigen)

install(TARGETS landmark_detector_node ekf_localizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/
  DESTINATION include/)

ament_package()
