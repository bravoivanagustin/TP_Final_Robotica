#include "lazo_cerrado/HolonomicTrajectoryFollower.h"

#include <chrono>

namespace robmovil
{

HolonomicTrajectoryFollower::HolonomicTrajectoryFollower()
  : Node("holonomic_trajectory_follower")
{
  // Frecuencia del loop de control. 30 Hz alcanza para un robot a < 0.5 m/s.
  double rate_hz = this->declare_parameter("control_rate_hz", 30.0);

  // Publicación de cmd_vel: el módulo 1 (mecanum_odometry) lo consume.
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/robot/cmd_vel", rclcpp::QoS(10));

  // Suscripción a la trayectoria objetivo. TransientLocal para que el
  // controller reciba el último mensaje aunque arranque después del generator.
  rclcpp::QoS traj_qos(rclcpp::KeepLast(10));
  traj_qos.reliable();
  traj_qos.transient_local();
  trajectory_sub_ = this->create_subscription<robmovil_msgs::msg::Trajectory>(
      "/robot/trajectory", traj_qos,
      std::bind(&HolonomicTrajectoryFollower::handleNewTrajectory, this, std::placeholders::_1));

  t0_ = this->now();

  // Timer del loop de control. Arranca acá (no al recibir trayectoria) para
  // que los modos sin trayectoria del derivado funcionen sin generator activo.
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HolonomicTrajectoryFollower::timerCallback, this));
}

void HolonomicTrajectoryFollower::handleNewTrajectory(
    const robmovil_msgs::msg::Trajectory& msg)
{
  RCLCPP_INFO(this->get_logger(), "Trayectoria recibida (%zu waypoints, frame=%s)",
              msg.points.size(), msg.header.frame_id.c_str());

  current_trajectory_ = msg;
  // Reanclamos t0 al instante en que llega la trayectoria, igual que el
  // diferencial (relevante sólo si una clase derivada usa time_from_start).
  t0_ = this->now();
}

void HolonomicTrajectoryFollower::timerCallback()
{
  rclcpp::Time t = this->now();

  double vx = 0.0, vy = 0.0, wz = 0.0;
  bool ok = this->control(t, vx, vy, wz);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = ok ? vx : 0.0;
  cmd.linear.y  = ok ? vy : 0.0;
  cmd.linear.z  = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = ok ? wz : 0.0;
  cmd_pub_->publish(cmd);

  if (!ok) {
    RCLCPP_INFO(this->get_logger(), "Trayectoria finalizada, deteniendo timer.");
    timer_->cancel();
  }
}

bool HolonomicTrajectoryFollower::nextPointIndex(
    const rclcpp::Time& time, size_t& next_point_idx) const
{
  size_t idx = 0;
  for (const robmovil_msgs::msg::TrajectoryPoint& point : current_trajectory_.points)
  {
    if (time < t0_ + point.time_from_start) {
      next_point_idx = idx;
      return true;
    }
    ++idx;
  }
  return false;
}

}  // namespace robmovil
