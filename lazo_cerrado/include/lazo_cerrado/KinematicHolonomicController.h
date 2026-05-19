#pragma once

#include "lazo_cerrado/HolonomicTrajectoryFollower.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace robmovil
{

// Controlador a lazo cerrado para el robot Mecanum (módulo 2 del TP).
//
// Ley de control proporcional por GDL aplicada en frame body:
//   ex_w = gx - x ; ey_w = gy - y ; eθ = normalize(gθ - θ)
//   ex_b =  cosθ·ex_w + sinθ·ey_w
//   ey_b = -sinθ·ex_w + cosθ·ey_w
//   vx, vy = clamp_norm(kp_xy·ex_b, kp_xy·ey_b, vxy_max)
//   wz     = clamp(kp_yaw·eθ, wz_max)
//
// Feedback: TF map → base_link (cambiable a base_link_ekf vía param `base_frame`
// cuando entre el módulo 3). Goal selection: FIXED para sanity checks, PURSUIT
// con cursor monotónico para el cuadrado.
class KinematicHolonomicController : public HolonomicTrajectoryFollower
{
public:
  enum class GoalMode { FIXED, PURSUIT };

  KinematicHolonomicController();

protected:
  bool control(const rclcpp::Time& t,
               double& vx, double& vy, double& wz) override;

private:
  // Feedback por TF.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool getCurrentPose(double& x, double& y, double& yaw);

  // Goal selection.
  bool getCurrentGoal(double cur_x, double cur_y, double cur_yaw,
                      double& gx, double& gy, double& gyaw);
  bool getPursuitGoal(double cur_x, double cur_y, double cur_yaw,
                      double& gx, double& gy, double& gyaw);
  size_t last_goal_idx_ = 0;
  bool first_goal_set_ = false;
  bool trajectory_finished_logged_ = false;

  // Visualización en RViz.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_path_pub_;
  nav_msgs::msg::Path real_path_msg_;
  size_t ticks_since_real_path_pub_ = 0;
  void publishGoalMarker(double gx, double gy, double gyaw, const rclcpp::Time& t);
  void appendAndMaybePublishRealPath(double x, double y, double yaw, const rclcpp::Time& t);

  // Parámetros.
  GoalMode goal_mode_;
  double fixed_goal_x_, fixed_goal_y_, fixed_goal_yaw_;
  double kp_xy_, kp_yaw_;
  double lookahead_distance_;
  double position_tolerance_, yaw_tolerance_;
  std::string map_frame_, base_frame_;

  // Throttle de warns por TF no lista al arranque.
  bool tf_ready_logged_ = false;
};

}  // namespace robmovil
