cmake_minimum_required(VERSION 3.5)
project(lazo_cerrado)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

include_directories(include)

# Librería interna: la base abstracta HolonomicTrajectoryFollower.
add_library(trajectory_controller src/HolonomicTrajectoryFollower.cpp)
ament_target_dependencies(trajectory_controller
  rclcpp geometry_msgs nav_msgs robmovil_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs angles
)
target_include_directories(trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Ejecutable: controlador a lazo cerrado (deriva de HolonomicTrajectoryFollower).
add_executable(trajectory_follower
  src/trajectory_follower_node.cpp
  src/KinematicHolonomicController.cpp
)
ament_target_dependencies(trajectory_follower
  rclcpp geometry_msgs nav_msgs robmovil_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs angles
)
target_link_libraries(trajectory_follower trajectory_controller)

# Ejecutable: generador de la trayectoria cuadrada (publica robmovil_msgs/Trajectory + nav_msgs/Path).
add_executable(trajectory_generator src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator
  rclcpp geometry_msgs nav_msgs robmovil_msgs std_msgs tf2 tf2_geometry_msgs
)

# Instalación.
install(TARGETS trajectory_controller
  EXPORT export_trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS trajectory_follower trajectory_generator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_export_targets(export_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs angles)

ament_package()
