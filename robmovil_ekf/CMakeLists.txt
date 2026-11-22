cmake_minimum_required(VERSION 3.5)
project(robmovil_ekf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(kfilter REQUIRED)

add_executable(localizer src/localizer_node.cpp src/localizer.cpp src/localizer_ekf.cpp)
ament_target_dependencies(localizer rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs angles kfilter)

add_executable(landmark_detector_node src/landmark_detector_node.cpp src/landmark_detector.cpp)
ament_target_dependencies(landmark_detector_node rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs angles)

install(TARGETS
  localizer
  landmark_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
