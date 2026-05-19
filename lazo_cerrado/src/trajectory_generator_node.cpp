#include <rclcpp/rclcpp.hpp>
#include <robmovil_msgs/msg/trajectory.hpp>
#include <robmovil_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <cmath>
#include <string>

// Genera la trayectoria cuadrada de 2 m de lado con orientación "opuesta al
// centro" (yaw = atan2(y, x) en cada waypoint) que pide la consigna del TP.
// Recorre CCW partiendo de (+s, +s).
//
// Pub:
//   /robot/trajectory  (robmovil_msgs/Trajectory) — consumido por el follower.
//   /desired_path      (nav_msgs/Path)            — visualización en RViz.
// Ambos con QoS TransientLocal: el follower y RViz se enganchan cuando arranquen.

static void build_square_trajectory(double s, double step,
                                    robmovil_msgs::msg::Trajectory& traj,
                                    nav_msgs::msg::Path& path)
{
  // Vértices CCW + cierre explícito en (+s, +s) para que el pursuit alcance fin
  // de trayectoria con tolerancia.
  const std::array<std::pair<double, double>, 5> verts = {{
      { s,  s},
      {-s,  s},
      {-s, -s},
      { s, -s},
      { s,  s},
  }};

  auto push_waypoint = [&](double x, double y, double yaw, double t_from_start) {
    robmovil_msgs::msg::TrajectoryPoint p;
    p.transform.translation.x = x;
    p.transform.translation.y = y;
    p.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    p.transform.rotation = tf2::toMsg(q);
    // velocity y acceleration se dejan en cero: el modo PURSUIT no los usa.
    p.time_from_start = rclcpp::Duration::from_seconds(t_from_start);
    traj.points.push_back(p);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = p.transform.rotation;
    path.poses.push_back(pose);
  };

  // dt_dummy sólo se guarda para que time_from_start sea monótono; ningún
  // consumidor lo usa en pursuit.
  const double dt_dummy = 0.1;
  double t_acc = 0.0;

  for (size_t i = 0; i + 1 < verts.size(); ++i) {
    double ax = verts[i].first,  ay = verts[i].second;
    double bx = verts[i + 1].first, by = verts[i + 1].second;
    double seg_len = std::hypot(bx - ax, by - ay);
    int n_steps = std::max(1, (int)std::ceil(seg_len / step));

    for (int k = 0; k < n_steps; ++k) {
      double u = double(k) / double(n_steps);
      double x = ax + u * (bx - ax);
      double y = ay + u * (by - ay);
      double yaw = std::atan2(y, x);
      push_waypoint(x, y, yaw, t_acc);
      t_acc += dt_dummy;
    }
  }

  // Waypoint de cierre: vértice final (+s, +s) con yaw radial.
  double yaw_end = std::atan2(s, s);  // = π/4
  push_waypoint(s, s, yaw_end, t_acc);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();

  auto node = rclcpp::Node::make_shared("trajectory_generator");

  double s          = node->declare_parameter("square_half_side", 2.0);
  double step       = node->declare_parameter("waypoint_step", 0.05);
  std::string frame = node->declare_parameter("frame_id", std::string("map"));

  auto traj_pub = node->create_publisher<robmovil_msgs::msg::Trajectory>(
      "/robot/trajectory", qos);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>(
      "/desired_path", qos);

  robmovil_msgs::msg::Trajectory traj;
  nav_msgs::msg::Path path;
  traj.header.stamp    = node->now();
  traj.header.frame_id = frame;
  path.header          = traj.header;

  build_square_trajectory(s, step, traj, path);

  RCLCPP_INFO(node->get_logger(),
              "Trayectoria cuadrada generada: %zu waypoints, lado=%.2f m, paso=%.3f m, frame=%s",
              traj.points.size(), 2.0 * s, step, frame.c_str());

  traj_pub->publish(traj);
  path_pub->publish(path);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
