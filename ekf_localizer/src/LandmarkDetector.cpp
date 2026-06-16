#include "ekf_localizer/LandmarkDetector.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>

using namespace robmovil;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LandmarkDetector::LandmarkDetector()
  : Node("landmark_detector")
{
  // Parametros
  robot_frame_                = this->declare_parameter<std::string>("robot_frame",  "base_link");
  laser_frame_                = this->declare_parameter<std::string>("laser_frame",  "front_laser");
  scan_topic_                 = this->declare_parameter<std::string>("scan_topic",   "/robot/front_laser/scan");
  landmark_diameter_          = this->declare_parameter<double>("landmark_diameter",          0.1);
  cluster_distance_threshold_ = this->declare_parameter<double>("cluster_distance_threshold", 0.15);
  min_cluster_points_         = this->declare_parameter<int>("min_cluster_points",  2);
  max_cluster_points_         = this->declare_parameter<int>("max_cluster_points",  30);
  max_range_                  = this->declare_parameter<double>("max_range",         8.0);

  // TF
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriber y Publishers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(10),
      std::bind(&LandmarkDetector::on_laser_scan, this, std::placeholders::_1));

  landmarks_pub_ = this->create_publisher<robmovil_msgs::msg::LandmarkArray>(
      "/landmarks", rclcpp::QoS(10));

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/landmark_markers", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(),
      "[landmark_detector] Iniciado. "
      "robot_frame=%s laser_frame=%s scan_topic=%s "
      "landmark_diameter=%.3f cluster_threshold=%.3f "
      "min_pts=%d max_pts=%d max_range=%.1f",
      robot_frame_.c_str(), laser_frame_.c_str(), scan_topic_.c_str(),
      landmark_diameter_, cluster_distance_threshold_,
      min_cluster_points_, max_cluster_points_, max_range_);
}

// ---------------------------------------------------------------------------
// updateLaserTf
// ---------------------------------------------------------------------------

bool LandmarkDetector::updateLaserTf()
{
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
        robot_frame_,        // frame destino: base_link
        laser_frame_,        // frame origen:  front_laser
        tf2::TimePointZero); // TF mas reciente disponible (estatica)
    tf2::fromMsg(tf_stamped.transform, laser_transform_);
    return true;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s->%s no disponible aun: %s",
        laser_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }
}

// ---------------------------------------------------------------------------
// on_laser_scan — callback principal
// ---------------------------------------------------------------------------

void LandmarkDetector::on_laser_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (!updateLaserTf()) return;

  // Convertir puntos validos a cartesiano en frame robot_frame_
  std::vector<tf2::Vector3> cartesian;
  cartesian.reserve(scan->ranges.size());

  const double range_min     = static_cast<double>(scan->range_min);
  const double range_max_eff = std::min(
      static_cast<double>(scan->range_max), max_range_);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double r = static_cast<double>(scan->ranges[i]);
    if (!std::isfinite(r) || r < range_min || r > range_max_eff) continue;

    double theta = static_cast<double>(scan->angle_min)
                 + static_cast<double>(i) * static_cast<double>(scan->angle_increment);
    double xl = r * std::cos(theta);
    double yl = r * std::sin(theta);

    tf2::Vector3 p_laser(xl, yl, 0.0);
    tf2::Vector3 p_robot = laser_transform_ * p_laser;
    cartesian.push_back(p_robot);
  }

  RCLCPP_DEBUG(this->get_logger(),
      "Scan recibido: %zu puntos validos en %s",
      cartesian.size(), robot_frame_.c_str());

  clusterAndPublish(cartesian, scan->header.stamp);
}

// ---------------------------------------------------------------------------
// clusterAndPublish — clustering secuencial y publicacion de /landmarks
// ---------------------------------------------------------------------------

void LandmarkDetector::clusterAndPublish(
    const std::vector<tf2::Vector3>& pts,
    const builtin_interfaces::msg::Time& stamp)
{
  robmovil_msgs::msg::LandmarkArray landmarks_msg;
  landmarks_msg.header.stamp    = stamp;       // NO now()
  landmarks_msg.header.frame_id = robot_frame_;

  std::vector<tf2::Vector3> centroids; // centroide de cada cluster aceptado

  std::vector<tf2::Vector3> cluster;

  auto emit_landmark = [&](const std::vector<tf2::Vector3>& c_pts)
  {
    if (static_cast<int>(c_pts.size()) < min_cluster_points_) return;
    if (static_cast<int>(c_pts.size()) > max_cluster_points_) return;

    // Centroide
    tf2::Vector3 centroid(0.0, 0.0, 0.0);
    for (const auto& p : c_pts) centroid += p;
    centroid /= static_cast<double>(c_pts.size());

    robmovil_msgs::msg::Landmark lm;
    lm.range   = static_cast<float>(std::hypot(centroid.x(), centroid.y()));
    lm.bearing = static_cast<float>(std::atan2(centroid.y(), centroid.x()));
    landmarks_msg.landmarks.push_back(lm);
    centroids.push_back(centroid);
  };

  for (const auto& p : pts) {
    if (cluster.empty()) {
      cluster.push_back(p);
    } else {
      double d = (p - cluster.back()).length();
      if (d < cluster_distance_threshold_) {
        cluster.push_back(p);
      } else {
        emit_landmark(cluster);
        cluster.clear();
        cluster.push_back(p);
      }
    }
  }
  // Procesar el ultimo cluster
  if (!cluster.empty()) emit_landmark(cluster);

  // Publicar
  landmarks_pub_->publish(landmarks_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Landmarks detectados: %zu", landmarks_msg.landmarks.size());

  publishMarkers(centroids, stamp);
}

// ---------------------------------------------------------------------------
// publishMarkers — MarkerArray en /landmark_markers
// ---------------------------------------------------------------------------

void LandmarkDetector::publishMarkers(
    const std::vector<tf2::Vector3>& centroids,
    const builtin_interfaces::msg::Time& stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // DELETEALL para limpiar los markers del scan anterior
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.stamp    = stamp;
  delete_all.header.frame_id = robot_frame_;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  for (size_t i = 0; i < centroids.size(); ++i) {
    const tf2::Vector3& c = centroids[i];

    visualization_msgs::msg::Marker m;
    m.header.stamp    = stamp;        // stamp del scan, nunca now()
    m.header.frame_id = robot_frame_; // "base_link"
    m.ns              = "landmarks";
    m.id              = static_cast<int>(i);
    m.type            = visualization_msgs::msg::Marker::CYLINDER;
    m.action          = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x    = c.x();
    m.pose.position.y    = c.y();
    m.pose.position.z    = 0.3;  // altura visual sobre el suelo
    m.pose.orientation.w = 1.0;

    m.scale.x = landmark_diameter_;
    m.scale.y = landmark_diameter_;
    m.scale.z = 0.5;

    // Color verde
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.9f;

    m.lifetime = rclcpp::Duration::from_seconds(0.2);

    marker_array.markers.push_back(m);
  }

  markers_pub_->publish(marker_array);
}
