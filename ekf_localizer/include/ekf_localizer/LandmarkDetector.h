#pragma once

// LandmarkDetector: convierte /robot/front_laser/scan en /landmarks (range, bearing
// en frame base_link). Detector stateless respecto al mapa — no conoce /posts.
// Frame de entrada: front_laser. Frame de salida: base_link.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <vector>
#include <string>

namespace robmovil
{

class LandmarkDetector : public rclcpp::Node
{
public:

  LandmarkDetector();

  void on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:

  bool updateLaserTf();

  void clusterAndPublish(const std::vector<tf2::Vector3>& pts,
                         const builtin_interfaces::msg::Time& stamp);

  void publishMarkers(const std::vector<tf2::Vector3>& centroids,
                      const builtin_interfaces::msg::Time& stamp);

  // Comunicacion ROS 2
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_;
  rclcpp::Publisher<robmovil_msgs::msg::LandmarkArray>::SharedPtr   landmarks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // TF
  std::unique_ptr<tf2_ros::Buffer>             tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
  tf2::Transform                               laser_transform_;

  // Parametros
  std::string robot_frame_;
  std::string laser_frame_;
  std::string scan_topic_;

  double cluster_distance_threshold_;
  double max_range_;
  double landmark_diameter_;

  int min_cluster_points_;
  int max_cluster_points_;
};

}  // namespace robmovil
