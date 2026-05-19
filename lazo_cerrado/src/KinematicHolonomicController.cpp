#include "lazo_cerrado/KinematicHolonomicController.h"

#include <angles/angles.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>
#include <string>

namespace robmovil
{

KinematicHolonomicController::KinematicHolonomicController()
  : HolonomicTrajectoryFollower()
{
  // Parámetros de la ley de control.
  kp_xy_              = this->declare_parameter("kp_xy", 0.8);
  kp_yaw_             = this->declare_parameter("kp_yaw", 1.5);
  lookahead_distance_ = this->declare_parameter("lookahead_distance", 0.30);
  position_tolerance_ = this->declare_parameter("position_tolerance", 0.05);
  yaw_tolerance_      = this->declare_parameter("yaw_tolerance", 0.05);
  map_frame_          = this->declare_parameter("map_frame", std::string("map"));
  base_frame_         = this->declare_parameter("base_frame", std::string("base_link"));

  // Parámetros del modo de selección de goal.
  std::string mode_str = this->declare_parameter("goal_mode", std::string("PURSUIT"));
  if (mode_str == "FIXED") {
    goal_mode_ = GoalMode::FIXED;
  } else if (mode_str == "PURSUIT") {
    goal_mode_ = GoalMode::PURSUIT;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "goal_mode='%s' no reconocido, usando PURSUIT.", mode_str.c_str());
    goal_mode_ = GoalMode::PURSUIT;
  }
  fixed_goal_x_   = this->declare_parameter("fixed_goal_x", 2.0);
  fixed_goal_y_   = this->declare_parameter("fixed_goal_y", 2.0);
  fixed_goal_yaw_ = this->declare_parameter("fixed_goal_yaw", M_PI_4);

  // TF buffer + listener para el feedback de pose.
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publicaciones para visualizar en RViz.
  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10));
  real_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/real_path", rclcpp::QoS(10));
  real_path_msg_.header.frame_id = map_frame_;

  RCLCPP_INFO(this->get_logger(),
              "Controller listo. goal_mode=%s kp_xy=%.3f kp_yaw=%.3f lookahead=%.3f map=%s base=%s",
              (goal_mode_ == GoalMode::FIXED ? "FIXED" : "PURSUIT"),
              kp_xy_, kp_yaw_, lookahead_distance_,
              map_frame_.c_str(), base_frame_.c_str());
}

bool KinematicHolonomicController::control(
    const rclcpp::Time& t, double& vx, double& vy, double& wz)
{
  vx = vy = wz = 0.0;

  // 1) Pose actual desde TF map → base_link.
  double x, y, yaw;
  if (!getCurrentPose(x, y, yaw)) {
    // TF todavía no lista (arranque); el timer reintenta al próximo tick.
    return true;
  }

  // 2) Goal actual (FIXED o PURSUIT). Si PURSUIT devuelve false, terminamos.
  double gx, gy, gyaw;
  if (!getCurrentGoal(x, y, yaw, gx, gy, gyaw)) {
    return false;
  }

  publishGoalMarker(gx, gy, gyaw, t);
  appendAndMaybePublishRealPath(x, y, yaw, t);

  // 3) Errores en mundo y rotación a frame body.
  double ex_w = gx - x;
  double ey_w = gy - y;
  double eth  = angles::normalize_angle(gyaw - yaw);

  double cth = std::cos(yaw);
  double sth = std::sin(yaw);
  double ex_b =  cth * ex_w + sth * ey_w;
  double ey_b = -sth * ex_w + cth * ey_w;

  // 4) Ley P por GDL.
  vx = kp_xy_  * ex_b;
  vy = kp_xy_  * ey_b;
  wz = kp_yaw_ * eth;

  return true;
}

bool KinematicHolonomicController::getCurrentPose(double& x, double& y, double& yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "TF %s → %s no disponible: %s",
                         map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }

  if (!tf_ready_logged_) {
    RCLCPP_INFO(this->get_logger(), "TF %s → %s disponible, iniciando control.",
                map_frame_.c_str(), base_frame_.c_str());
    tf_ready_logged_ = true;
  }

  x   = tf.transform.translation.x;
  y   = tf.transform.translation.y;
  yaw = tf2::getYaw(tf.transform.rotation);
  return true;
}

bool KinematicHolonomicController::getCurrentGoal(
    double cur_x, double cur_y, double cur_yaw,
    double& gx, double& gy, double& gyaw)
{
  switch (goal_mode_) {
    case GoalMode::FIXED:
      gx   = fixed_goal_x_;
      gy   = fixed_goal_y_;
      gyaw = fixed_goal_yaw_;
      return true;
    case GoalMode::PURSUIT:
      return getPursuitGoal(cur_x, cur_y, cur_yaw, gx, gy, gyaw);
  }
  return false;
}

bool KinematicHolonomicController::getPursuitGoal(
    double cur_x, double cur_y, double cur_yaw,
    double& gx, double& gy, double& gyaw)
{
  const auto& traj = getTrajectory();
  const size_t N = traj.points.size();
  if (N == 0) {
    // Aún no llegó la trayectoria del generator: setear goal = pose actual
    // hace que el error sea cero y la ley P publique velocidades nulas
    // mientras esperamos.
    gx   = cur_x;
    gy   = cur_y;
    gyaw = cur_yaw;
    return true;
  }

  // Primer goal: arrancar desde el waypoint más cercano al robot.
  if (!first_goal_set_) {
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < N; ++i) {
      double dx = traj.points[i].transform.translation.x - cur_x;
      double dy = traj.points[i].transform.translation.y - cur_y;
      double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; last_goal_idx_ = i; }
    }
    first_goal_set_ = true;
  }

  // Avanzar el cursor mientras el waypoint actual esté dentro del lookahead.
  for (size_t i = last_goal_idx_; i < N; ++i) {
    double wx = traj.points[i].transform.translation.x;
    double wy = traj.points[i].transform.translation.y;
    double d  = std::hypot(wx - cur_x, wy - cur_y);
    if (d > lookahead_distance_) {
      last_goal_idx_ = i;
      gx   = wx;
      gy   = wy;
      gyaw = tf2::getYaw(traj.points[i].transform.rotation);
      return true;
    }
  }

  // No quedan waypoints más allá del lookahead → estamos cerca del final.
  const auto& last = traj.points.back();
  double last_x   = last.transform.translation.x;
  double last_y   = last.transform.translation.y;
  double last_yaw = tf2::getYaw(last.transform.rotation);
  double d_last   = std::hypot(last_x - cur_x, last_y - cur_y);
  double yaw_err  = std::abs(angles::normalize_angle(last_yaw - cur_yaw));

  if (d_last < position_tolerance_ && yaw_err < yaw_tolerance_) {
    if (!trajectory_finished_logged_) {
      RCLCPP_INFO(this->get_logger(),
                  "Trayectoria completada en (%.3f, %.3f, %.3f).",
                  cur_x, cur_y, cur_yaw);
      trajectory_finished_logged_ = true;
    }
    return false;
  }

  // Todavía no llegamos: el último waypoint queda como goal (cierre de vuelta).
  last_goal_idx_ = N - 1;
  gx   = last_x;
  gy   = last_y;
  gyaw = last_yaw;
  return true;
}

void KinematicHolonomicController::publishGoalMarker(
    double gx, double gy, double gyaw, const rclcpp::Time& t)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp    = t;
  pose.header.frame_id = map_frame_;
  pose.pose.position.x = gx;
  pose.pose.position.y = gy;
  pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, gyaw);
  pose.pose.orientation = tf2::toMsg(q);
  goal_pose_pub_->publish(pose);
}

void KinematicHolonomicController::appendAndMaybePublishRealPath(
    double x, double y, double yaw, const rclcpp::Time& t)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp    = t;
  pose.header.frame_id = map_frame_;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  real_path_msg_.poses.push_back(pose);

  // Publicar cada ~10 ticks (≈3 Hz a 30 Hz de loop) para no inundar el bus.
  if (++ticks_since_real_path_pub_ >= 10) {
    ticks_since_real_path_pub_ = 0;
    real_path_msg_.header.stamp = t;
    real_path_pub_->publish(real_path_msg_);
  }
}

}  // namespace robmovil
